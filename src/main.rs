#![no_main]
#![no_std]

use core::mem::MaybeUninit;

use open_cm as _;
use open_cm::tecmp::ED_NUM;

use fdcan::Instance;
use fdcan::config::FdCanConfig;
use stm32h7xx_hal::ethernet;

/// Ethernet descriptor rings are a global singleton
#[unsafe(link_section = ".axisram.eth")]
static mut DES_RING: MaybeUninit<ethernet::DesRing<ED_NUM, ED_NUM>> = MaybeUninit::uninit();

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0, EXTI1])]
mod app {
    use fdcan::config::{Interrupt, Interrupts};
    use open_cm::can::CanStateWrapper;
    use open_cm::tecmp::LOCAL_MAC_ADDRESS;
    use open_cm::tecmp::TecmpHandler;
    use rtic_monotonics::stm32::prelude::*;
    use stm32h7xx_hal::{
        can::Can,
        ethernet::{self, PHY},
        gpio::{self, Speed},
        pac::{FDCAN1, FDCAN2},
        prelude::*,
        rcc::{
            PllConfigStrategy,
            rec::{Fdcan, FdcanClkSel},
        },
    };

    use super::*;

    #[shared]
    struct SharedResources {
        tecmp_handler: TecmpHandler,
        can1: CanStateWrapper<Can<FDCAN1>>,
        can2: CanStateWrapper<Can<FDCAN2>>,
    }
    #[local]
    struct LocalResources {
        lan8742a: ethernet::phy::LAN8742A<ethernet::EthernetMAC>,
        link_led: gpio::gpioc::PC3<gpio::Output<gpio::PushPull>>,
    }

    stm32_tim2_monotonic!(Mono, 1_000_000);

    #[init]
    fn init(mut ctx: init::Context) -> (SharedResources, LocalResources) {
        // Initialise power...
        let pwr = ctx.device.PWR.constrain();
        let pwrcfg = pwr.smps().freeze();

        // Initialise clocks...
        let rcc = ctx.device.RCC.constrain();
        let ccdr = rcc
            .sys_ck(200.MHz())
            .hclk(200.MHz())
            .pll1_strategy(PllConfigStrategy::Iterative)
            .pll1_q_ck(24.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        // Check clocks
        // CAN
        assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 24_000_000);
        // Ethernet
        assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
        assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

        // Initialise system...
        ctx.core.SCB.invalidate_icache();
        ctx.core.SCB.enable_icache();
        // TODO: ETH DMA coherence issues
        // ctx.core.SCB.enable_dcache(&mut ctx.core.CPUID);
        ctx.core.DWT.enable_cycle_counter();

        // Initialise IO...
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpioc = ctx.device.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpiog = ctx.device.GPIOG.split(ccdr.peripheral.GPIOG);
        let gpioh = ctx.device.GPIOH.split(ccdr.peripheral.GPIOH);

        // Display
        let mut lcd_bl_ctrl = gpiog.pg15.into_push_pull_output();
        lcd_bl_ctrl.set_low();

        let mut lcd_rst = gpioh.ph6.into_push_pull_output();
        lcd_rst.set_low();

        let mut link_led = gpioc.pc3.into_push_pull_output(); // USR LED1
        link_led.set_high();

        // CAN
        let can1_rx = gpioh.ph14.into_alternate().speed(Speed::VeryHigh);
        let can1_tx = gpioh.ph13.into_alternate().speed(Speed::VeryHigh);

        let can2_rx = gpiob.pb5.into_alternate().speed(Speed::VeryHigh);
        let can2_tx = gpiob.pb6.into_alternate().speed(Speed::VeryHigh);

        // Ethernet
        let rmii_ref_clk = gpioa.pa1.into_alternate();
        let rmii_mdio = gpioa.pa2.into_alternate();
        let rmii_mdc = gpioc.pc1.into_alternate();
        let rmii_crs_dv = gpioa.pa7.into_alternate();
        let rmii_rxd0 = gpioc.pc4.into_alternate();
        let rmii_rxd1 = gpioc.pc5.into_alternate();
        let rmii_tx_en = gpiob.pb11.into_alternate();
        let rmii_txd0 = gpiob.pb12.into_alternate();
        let rmii_txd1 = gpiob.pb13.into_alternate();

        // Initialize CAN...
        let fdcan_prec1 = ccdr.peripheral.FDCAN.kernel_clk_mux(FdcanClkSel::Pll1Q);
        // Since I've found not safe way to clone or otherwise get Fdcan mux
        let fdcan_prec2 = unsafe { (&fdcan_prec1 as *const Fdcan).read() };
        let mut can1 = ctx.device.FDCAN1.fdcan(can1_tx, can1_rx, fdcan_prec1);
        let mut can2 = ctx.device.FDCAN2.fdcan(can2_tx, can2_rx, fdcan_prec2);

        can1.enable_interrupts(Interrupts::all());
        can2.enable_interrupts(Interrupts::all());

        // Initialise ethernet...
        let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&LOCAL_MAC_ADDRESS.0);
        let (eth_dma, eth_mac) = unsafe {
            #[allow(static_mut_refs)] // TODO: Fix this
            DES_RING.write(ethernet::DesRing::new());

            ethernet::new(
                ctx.device.ETHERNET_MAC,
                ctx.device.ETHERNET_MTL,
                ctx.device.ETHERNET_DMA,
                (
                    rmii_ref_clk,
                    rmii_mdio,
                    rmii_mdc,
                    rmii_crs_dv,
                    rmii_rxd0,
                    rmii_rxd1,
                    rmii_tx_en,
                    rmii_txd0,
                    rmii_txd1,
                ),
                #[allow(static_mut_refs)] // TODO: Fix this
                DES_RING.assume_init_mut(),
                mac_addr,
                ccdr.peripheral.ETH1MAC,
                &ccdr.clocks,
            )
        };

        // Initialise ethernet PHY...
        let mut lan8742a = ethernet::phy::LAN8742A::new(eth_mac);
        lan8742a.phy_reset();
        lan8742a.phy_init();
        // The eth_dma should not be used until the PHY reports the link is up

        unsafe { ethernet::enable_interrupt() };

        let tecmp_handler = TecmpHandler::new(eth_dma);

        Mono::start(200_000_000);

        (
            SharedResources {
                tecmp_handler,
                can1: CanStateWrapper::Config(can1),
                can2: CanStateWrapper::Config(can2),
            },
            LocalResources { lan8742a, link_led },
        )
    }

    #[idle(local = [lan8742a, link_led])]
    fn idle(ctx: idle::Context) -> ! {
        loop {
            // Ethernet
            match ctx.local.lan8742a.poll_link() {
                true => ctx.local.link_led.set_low(),
                _ => ctx.local.link_led.set_high(),
            }
        }
    }

    #[task(priority = 2, shared = [can1])]
    async fn enable_can1(mut ctx: enable_can1::Context) {
        ctx.shared.can1.lock(|can| {
            let wrapper = core::mem::take(can);
            core::mem::replace(can, wrapper.enable(FdCanConfig::default(), false))
        });
    }

    #[task(priority = 2, shared = [can2])]
    async fn enable_can2(mut ctx: enable_can2::Context) {
        ctx.shared.can2.lock(|can| {
            let wrapper = core::mem::take(can);
            core::mem::replace(can, wrapper.enable(FdCanConfig::default(), false))
        });
    }

    #[task(priority = 2, shared = [can1])]
    async fn disable_can1(mut ctx: disable_can1::Context) {
        ctx.shared.can1.lock(|can| {
            let wrapper = core::mem::take(can);
            core::mem::replace(can, wrapper.disable())
        });
    }

    #[task(priority = 2, shared = [can2])]
    async fn disable_can2(mut ctx: disable_can2::Context) {
        ctx.shared.can2.lock(|can| {
            let wrapper = core::mem::take(can);
            core::mem::replace(can, wrapper.disable())
        });
    }

    #[task(binds = ETH, shared = [tecmp_handler])]
    fn ethernet_event(mut ctx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }

        ctx.shared
            .tecmp_handler
            .lock(|tecmp_handler| tecmp_handler.receive());
    }

    fn handle_can_event<I: Instance>(can: &mut CanStateWrapper<I>, id: &'static str) {
        match can {
            CanStateWrapper::BusMonitoring(fd_can) => {
                defmt::info!(
                    "{} has new message: {}",
                    id,
                    fd_can.has_interrupt(Interrupt::RxFifo0NewMsg)
                );
                fd_can.clear_interrupts(Interrupts::all());
            }
            CanStateWrapper::NormalOperation(fd_can) => {
                defmt::info!(
                    "{}, has new message: {}",
                    id,
                    fd_can.has_interrupt(Interrupt::RxFifo0NewMsg)
                );
                fd_can.clear_interrupts(Interrupts::all());
            }
            _ => {}
        };
    }

    #[task(binds = FDCAN1_IT0, shared = [can1])]
    fn can1_event(mut ctx: can1_event::Context) {
        ctx.shared.can1.lock(|can1| handle_can_event(can1, "CAN1"));
    }

    #[task(binds = FDCAN2_IT0, shared = [can2])]
    fn can2_event(mut ctx: can2_event::Context) {
        ctx.shared.can2.lock(|can2| handle_can_event(can2, "CAN2"));
    }
}
