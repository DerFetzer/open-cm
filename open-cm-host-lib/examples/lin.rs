use std::cmp::max;
use std::env::args;
use std::error::Error;
use std::num::NonZero;
use std::time::Duration;

use clap::Parser;
use open_cm_common::can::{CanBitTiming, CanChannelConfig};
use open_cm_common::lin::{
    LinChannelConfig, LinChecksum, LinConfig, LinScheduleSlot, LinScheduleTable,
    ProtectedIdentifier,
};
use open_cm_common::tecmp::{InterfaceId, TecmpConfig};
use open_cm_host_lib::{write_config_packet, write_tecmp_replay_packet};
use pcap::{Capture, Device};
use tecmp_rs::{LinData, LinDataFlags, heapless};

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

    let channel_config = LinConfig::Config(LinChannelConfig {
        bitrate: 19200,
        checksum: LinChecksum::Classic,
    });
    let config = TecmpConfig::Lin(channel_config);

    let bytes_written = write_config_packet(&mut buf, LOCAL_MAC_ADDRESS, config)?;
    let tx_len = max(64, bytes_written);
    cap.sendpacket(&buf[..tx_len])?;

    buf.fill(0);
    let schedule_table_config =
        LinConfig::ScheduleTable(LinScheduleTable(heapless::Vec::from_array([
            LinScheduleSlot {
                pid: ProtectedIdentifier::from_id(0x2),
                delay_ms: 500,
            },
            LinScheduleSlot {
                pid: ProtectedIdentifier::from_id(0x3),
                delay_ms: 300,
            },
        ])));
    let config = TecmpConfig::Lin(schedule_table_config);

    let bytes_written = write_config_packet(&mut buf, LOCAL_MAC_ADDRESS, config)?;
    let tx_len = max(64, bytes_written);
    cap.sendpacket(&buf[..tx_len])?;

    std::thread::sleep(Duration::from_secs(5));

    buf.fill(0);
    let data = tecmp_rs::Data::Lin(LinData {
        flags: LinDataFlags::default(),
        lin_id: 0x2,
        payload_length: 8,
        payload: tecmp_rs::heapless::Vec::from_array([0, 1, 2, 3, 4, 5, 6, 7]),
        checksum: 0,
    });

    let bytes_written =
        write_tecmp_replay_packet(&mut buf, LOCAL_MAC_ADDRESS, data, args.interface_id)?;
    let tx_len = max(64, bytes_written);
    cap.sendpacket(&buf[..tx_len])?;

    buf.fill(0);
    let data = tecmp_rs::Data::Lin(LinData {
        flags: LinDataFlags::default(),
        lin_id: 0x3,
        payload_length: 0,
        payload: tecmp_rs::heapless::Vec::new(),
        checksum: 0,
    });

    let bytes_written =
        write_tecmp_replay_packet(&mut buf, LOCAL_MAC_ADDRESS, data, args.interface_id)?;
    let tx_len = max(64, bytes_written);
    cap.sendpacket(&buf[..tx_len])?;

    Ok(())
}
