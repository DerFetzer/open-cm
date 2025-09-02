use std::cmp::max;
use std::error::Error;
use std::num::NonZero;

use clap::Parser;
use open_cm_common::can::{CanBitTiming, CanChannelConfig};
use open_cm_common::tecmp::{InterfaceId, TecmpConfig};
use open_cm_host_lib::write_config_packet;
use pcap::{Capture, Device};

/// Locally administered MAC address
pub const LOCAL_MAC_ADDRESS: [u8; 6] = [0x02, 0x6f, 0xbd, 0x3d, 0x4a, 0x00];

#[derive(Parser, Debug)]
struct Args {
    interface_id: u32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let device = Device::lookup()?.unwrap();
    let mut cap = Capture::from_device(device)?.open()?;

    let mut buf = [0; 1500];

    let channel_config = CanChannelConfig {
        interface_id: InterfaceId(args.interface_id),
        enabled: true,
        monitoring: false,
        nominal_bit_timing: CanBitTiming {
            prescaler: NonZero::new(8).unwrap(),
            seg1: NonZero::new(3).unwrap(),
            seg2: NonZero::new(1).unwrap(),
            sync_jump_width: NonZero::new(1).unwrap(),
        },
        data_bit_timing: CanBitTiming {
            prescaler: NonZero::new(4).unwrap(),
            seg1: NonZero::new(3).unwrap(),
            seg2: NonZero::new(1).unwrap(),
            sync_jump_width: NonZero::new(1).unwrap(),
        },
        transceiver_delay_compensation: false,
        automatic_retransmit: false,
        protocol_exception_handling: true,
    };
    let config = TecmpConfig::Can(channel_config);

    let bytes_written = write_config_packet(&mut buf, LOCAL_MAC_ADDRESS, config)?;

    let tx_len = max(64, bytes_written);

    cap.sendpacket(&buf[..tx_len])?;
    Ok(())
}
