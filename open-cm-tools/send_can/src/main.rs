use std::cmp::max;
use std::error::Error;
use std::time::Duration;

use pcap::{Capture, Device};
use smoltcp::wire::{EthernetAddress, EthernetFrame, EthernetProtocol};
use tecmp_rs::{CanData, CanDataFlags, DataType, deku::DekuContainerWrite};
use tecmp_rs::{
    MessageType, Tecmp, TecmpData, TecmpGlobalHeader,
    deku::{DekuUpdate as _, DekuWriter as _, no_std_io::Cursor, writer::Writer},
};

/// Locally administered MAC address
pub const LOCAL_MAC_ADDRESS: EthernetAddress =
    EthernetAddress([0x02, 0x6f, 0xbd, 0x3d, 0x4a, 0x00]);
/// Default TECMP multicast MAC address
pub const TECMP_DST_MAC_ADDRESS: EthernetAddress =
    EthernetAddress([0x01, 0x00, 0x5e, 0x00, 0x00, 0x00]);

pub const TECMP_ETHERTYPE: EthernetProtocol = EthernetProtocol::Unknown(0x99fe);

fn main() -> Result<(), Box<dyn Error>> {
    let device = Device::lookup()?.unwrap();
    let mut cap = Capture::from_device(device)?.open()?;

    let mut buf = [0; 1500];

    let mut packet = EthernetFrame::new_checked(&mut buf)?;
    packet.set_src_addr(LOCAL_MAC_ADDRESS);
    packet.set_dst_addr(EthernetAddress::BROADCAST);
    packet.set_ethertype(TECMP_ETHERTYPE);

    let payload = packet.payload_mut();

    let tecmp = Tecmp {
        header: TecmpGlobalHeader {
            device_id: 1,
            counter: 0,
            version: 3,
            message_type: MessageType::ReplayData,
            data_type: DataType::Can,
            reserved: 0,
            device_flags: Default::default(),
        },
    };

    let mut data = TecmpData {
        interface_id: 1,
        timestamp: 0,
        length: 0,
        data: tecmp_rs::Data::Can(CanData {
            flags: CanDataFlags::default(),
            can_id: 0x100,
            payload_length: 8,
            payload: tecmp_rs::heapless::Vec::from_array([0, 1, 2, 3, 4, 5, 6, 7]),
            crc: [0; 2],
        }),
    };
    data.update()?;

    let data_offset = tecmp.to_slice(payload).unwrap();
    let mut writer = Writer::new(Cursor::new(&mut payload[data_offset..]));
    data.to_writer(&mut writer, data.data.data_type()).unwrap();

    let tx_len = max(
        64,
        EthernetFrame::<&[u8]>::buffer_len(tecmp.len() + data.len()),
    );
    cap.sendpacket(&buf[..tx_len])?;
    Ok(())
    // loop {
    //     cap.sendpacket(&buf[..tx_len])?;
    //     std::thread::sleep(Duration::from_micros(10_000));
    // }
}
