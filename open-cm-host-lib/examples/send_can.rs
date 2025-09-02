use std::cmp::max;
use std::error::Error;
use std::time::Duration;

use clap::Parser;
use open_cm_host_lib::write_tecmp_replay_packet;
use pcap::{Capture, Device};
use tecmp_rs::{CanData, CanDataFlags};

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

    let data = tecmp_rs::Data::Can(CanData {
        flags: CanDataFlags::default(),
        can_id: 0x100,
        payload_length: 8,
        payload: tecmp_rs::heapless::Vec::from_array([0, 1, 2, 3, 4, 5, 6, 7]),
        crc: [0; 2],
    });

    let bytes_written =
        write_tecmp_replay_packet(&mut buf, LOCAL_MAC_ADDRESS, data, args.interface_id)?;

    let tx_len = max(64, bytes_written);

    loop {
        cap.sendpacket(&buf[..tx_len])?;
        std::thread::sleep(Duration::from_millis(500));
    }
}
