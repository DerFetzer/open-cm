#![no_main]
#![no_std]

use core::mem::MaybeUninit;

use open_cm_fw as _;
use open_cm_fw::tecmp::ED_NUM;

use stm32h7xx_hal::ethernet;

/// Ethernet descriptor rings are a global singleton
#[unsafe(link_section = ".axisram.eth")]
static mut DES_RING: MaybeUninit<ethernet::DesRing<ED_NUM, ED_NUM>> = MaybeUninit::uninit();

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0, EXTI1])]
mod app {
    use open_cm_common::tecmp::{CM_LOCAL_MAC_ADDRESS, InterfaceId, TecmpConfig, TecmpEvent};
    use open_cm_fw::can::CanHandler;
    use open_cm_fw::can::Fifo;
    use open_cm_fw::tecmp::TECMP_CHANNEL_SIZE;
    use open_cm_fw::tecmp::TecmpHandler;
    use rtic_monotonics::stm32::prelude::*;
    use rtic_sync::channel::Receiver;
    use rtic_sync::channel::Sender;
    use rtic_sync::make_channel;
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
    use tecmp_rs::TecmpData;

    use super::*;

    #[shared]
    struct SharedResources {
        tecmp_handler: TecmpHandler,
        can1: CanHandler<Can<FDCAN1>>,
        can2: CanHandler<Can<FDCAN2>>,
    }
    #[local]
    struct LocalResources {
        lan8742a: ethernet::phy::LAN8742A<ethernet::EthernetMAC>,
        link_led: gpio::gpioc::PC3<gpio::Output<gpio::PushPull>>,
        tecmp_sender_can1: Sender<'static, TecmpData, TECMP_CHANNEL_SIZE>,
        tecmp_sender_can2: Sender<'static, TecmpData, TECMP_CHANNEL_SIZE>,
        tecmp_event_sender: Sender<'static, TecmpEvent, TECMP_CHANNEL_SIZE>,
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
            .use_hse(25.MHz())
            .sys_ck(200.MHz())
            .hclk(200.MHz())
            .pll1_strategy(PllConfigStrategy::Iterative)
            .pll1_q_ck(20.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);

        // Check clocks
        // CAN
        assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 20_000_000);
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
        let can1 = ctx.device.FDCAN1.fdcan(can1_tx, can1_rx, fdcan_prec1);
        let can2 = ctx.device.FDCAN2.fdcan(can2_tx, can2_rx, fdcan_prec2);

        // Initialise ethernet...
        let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&CM_LOCAL_MAC_ADDRESS.0);
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

        // Channels
        let (tecmp_s, tecmp_r) = make_channel!(TecmpData, TECMP_CHANNEL_SIZE);
        let (tecmp_event_s, tecmp_event_r) = make_channel!(TecmpEvent, TECMP_CHANNEL_SIZE);

        // Spawn tasks
        tecmp_sender::spawn(tecmp_r).unwrap();
        tecmp_event_dispatcher::spawn(tecmp_event_r).unwrap();

        Mono::start(200_000_000);

        (
            SharedResources {
                tecmp_handler,
                can1: CanHandler::new(can1, Fifo::Fifo0, InterfaceId(0x1), FDCAN1::ptr() as _),
                can2: CanHandler::new(can2, Fifo::Fifo1, InterfaceId(0x2), FDCAN2::ptr() as _),
            },
            LocalResources {
                lan8742a,
                link_led,
                tecmp_sender_can1: tecmp_s.clone(),
                tecmp_sender_can2: tecmp_s,
                tecmp_event_sender: tecmp_event_s,
            },
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

    #[task(binds = ETH, shared = [tecmp_handler], local = [tecmp_event_sender], priority = 3)]
    fn ethernet_event(mut ctx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }

        ctx.shared
            .tecmp_handler
            .lock(|tecmp_handler| tecmp_handler.receive(ctx.local.tecmp_event_sender));
    }

    #[task(shared = [tecmp_handler], priority = 1)]
    async fn tecmp_sender(
        mut ctx: tecmp_sender::Context,
        mut receiver: Receiver<'static, TecmpData, TECMP_CHANNEL_SIZE>,
    ) {
        while let Ok(data) = receiver.recv().await {
            defmt::info!("Sender: got tecmp data");
            ctx.shared.tecmp_handler.lock(|handler| handler.send(&data));
        }
    }

    #[task(shared = [can1, can2], priority = 1)]
    async fn tecmp_event_dispatcher(
        mut ctx: tecmp_event_dispatcher::Context,
        mut receiver: Receiver<'static, TecmpEvent, TECMP_CHANNEL_SIZE>,
    ) {
        while let Ok(event) = receiver.recv().await {
            match event {
                TecmpEvent::Data(data) => {
                    defmt::info!("Receiver: got tecmp data");
                    ctx.shared.can1.lock(|can| {
                        if can.interface_id.0 == data.interface_id
                            && !can.transmit_tecmp_data(&data)
                        {
                            defmt::error!(
                                "Could not send can data for interface {}",
                                can.interface_id
                            );
                        }
                    });
                    ctx.shared.can2.lock(|can| {
                        if can.interface_id.0 == data.interface_id
                            && !can.transmit_tecmp_data(&data)
                        {
                            defmt::error!(
                                "Could not send can data for interface {}",
                                can.interface_id
                            );
                        }
                    });
                }
                TecmpEvent::Config(config) => match config {
                    TecmpConfig::Can(can_channel_config) => {
                        defmt::info!("Receiver: got can channel config: {:?}", can_channel_config);

                        ctx.shared.can1.lock(|can| {
                            if can.interface_id == can_channel_config.interface_id {
                                can.configure(can_channel_config);
                            }
                        });
                        ctx.shared.can2.lock(|can| {
                            if can.interface_id == can_channel_config.interface_id {
                                can.configure(can_channel_config);
                            }
                        });
                    }
                },
            }
        }
    }

    #[task(binds = FDCAN1_IT0, shared = [can1], local = [tecmp_sender_can1], priority = 3)]
    fn can1_event(mut ctx: can1_event::Context) {
        defmt::info!("Got FDCAN1_IT0 interrupt");
        let tecmp_sender = ctx.local.tecmp_sender_can1;
        ctx.shared.can1.lock(|can1| {
            can1.handle_can_event(Mono::now().ticks() * 1000, tecmp_sender);
        });
    }

    #[task(binds = FDCAN2_IT0, shared = [can2], local = [tecmp_sender_can2], priority = 3)]
    fn can2_event(mut ctx: can2_event::Context) {
        defmt::info!("Got FDCAN2_IT0 interrupt");
        let tecmp_sender = ctx.local.tecmp_sender_can2;
        ctx.shared.can2.lock(|can2| {
            can2.handle_can_event(Mono::now().ticks() * 1000, tecmp_sender);
        });
    }
}
