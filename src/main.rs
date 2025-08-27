#![no_main]
#![no_std]

use open_cm as _;

use heapless::Vec;
use smoltcp::wire::{EthernetAddress, EthernetFrame, EthernetProtocol};
use tecmp_rs::deku::no_std_io::Cursor;
use tecmp_rs::deku::writer::Writer;
use tecmp_rs::deku::{DekuContainerRead, DekuContainerWrite, DekuRead, DekuWrite, DekuWriter};
use tecmp_rs::{
    CanData, CanDataFlags, Data, DataType, DeviceFlags, MessageType, Tecmp, TecmpData,
    TecmpGlobalHeader,
};

use core::cmp::max;
use core::mem::MaybeUninit;

use smoltcp::phy::{Device, RxToken, TxToken};
use smoltcp::time::Instant;

use stm32h7xx_hal::ethernet;

/// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x6f, 0xbd, 0x3d, 0x4a, 0x04];

/// Ethernet descriptor rings are a global singleton
#[unsafe(link_section = ".axisram.eth")]
static mut DES_RING: MaybeUninit<ethernet::DesRing<4, 4>> = MaybeUninit::uninit();

pub struct Net {
    ethdev: ethernet::EthernetDMA<4, 4>,
}

impl Net {
    pub fn new(ethdev: ethernet::EthernetDMA<4, 4>) -> Self {
        Net { ethdev }
    }

    pub fn poll(&mut self, now: u64) {
        let timestamp = Instant::from_micros(now as i64);

        while let Some((rx_token, _)) = self.ethdev.receive(timestamp) {
            rx_token.consume(|buf| {
                let eth = EthernetFrame::new_checked(buf).unwrap();
                defmt::info!(
                    "Received ethernet frame -> src: {}, dst: {}, ethertype: {}",
                    eth.src_addr(),
                    eth.dst_addr(),
                    eth.ethertype()
                );
            })
        }
        // self.iface
        //     .poll(timestamp, &mut self.ethdev, &mut self.sockets);
    }

    pub fn transmit(&mut self, buf: &[u8]) {
        if let Some(tx_token) = self.ethdev.transmit(Instant::from_micros_const(0)) {
            tx_token.consume(buf.len(), |tx_buf| tx_buf.copy_from_slice(buf))
        } else {
            defmt::error!("Could not get tx token!");
        }
    }
}

#[derive(Debug, Copy, Clone, Default)]
pub struct TecmpSender {
    tx_counter: u16,
}

impl TecmpSender {
    pub fn new() -> Self {
        Self { tx_counter: 0 }
    }

    pub fn send(&mut self, data: &TecmpData, net: &mut Net) {
        let tecmp = Tecmp {
            header: TecmpGlobalHeader {
                device_id: 0x0001,
                counter: self.tx_counter,
                version: 3,
                message_type: MessageType::LoggingStream,
                data_type: data.data.data_type(),
                reserved: 0,
                device_flags: DeviceFlags {
                    eos: false,
                    sos: false,
                    spy: false,
                    multi_frame: false,
                    device_overflow: false,
                },
            },
        };

        let buf_len = max(64, tecmp.len() + data.len());

        let buf = [0_u8; 1500];
        let mut eth = EthernetFrame::new_unchecked(buf);

        eth.set_dst_addr(EthernetAddress([0x01, 0x00, 0x5e, 0x00, 0x00, 0x00]));
        eth.set_src_addr(EthernetAddress(MAC_ADDRESS));
        eth.set_ethertype(EthernetProtocol::Unknown(0x99fe));

        let payload = eth.payload_mut();
        let data_offset = tecmp.to_slice(payload).unwrap();
        let mut writer = Writer::new(Cursor::new(&mut payload[data_offset..]));
        data.to_writer(&mut writer, data.data.data_type()).unwrap();

        net.transmit(&buf[..buf_len]);

        (self.tx_counter, _) = self.tx_counter.overflowing_add(1);
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use rtic_monotonics::stm32::prelude::*;
    use stm32h7xx_hal::{ethernet, ethernet::PHY, gpio, prelude::*};

    use super::*;

    #[shared]
    struct SharedResources {}
    #[local]
    struct LocalResources {
        net: Net,
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
            .freeze(pwrcfg, &ctx.device.SYSCFG);

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

        let mut lcd_bl_ctrl = gpiog.pg15.into_push_pull_output();
        lcd_bl_ctrl.set_low();

        let mut lcd_rst = gpioh.ph6.into_push_pull_output();
        lcd_rst.set_low();

        let mut link_led = gpioc.pc3.into_push_pull_output(); // USR LED1
        link_led.set_high();

        let rmii_ref_clk = gpioa.pa1.into_alternate();
        let rmii_mdio = gpioa.pa2.into_alternate();
        let rmii_mdc = gpioc.pc1.into_alternate();
        let rmii_crs_dv = gpioa.pa7.into_alternate();
        let rmii_rxd0 = gpioc.pc4.into_alternate();
        let rmii_rxd1 = gpioc.pc5.into_alternate();
        let rmii_tx_en = gpiob.pb11.into_alternate();
        let rmii_txd0 = gpiob.pb12.into_alternate();
        let rmii_txd1 = gpiob.pb13.into_alternate();

        // Initialise ethernet...
        assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
        assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
        assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

        let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
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

        let net = Net::new(eth_dma);

        Mono::start(200_000_000);

        (
            SharedResources {},
            LocalResources {
                net,
                lan8742a,
                link_led,
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

    #[task(binds = ETH, local = [net])]
    fn ethernet_event(ctx: ethernet_event::Context) {
        unsafe { ethernet::interrupt_handler() }

        let time = Mono::now().duration_since_epoch().to_micros();
        ctx.local.net.poll(time);
    }
}
