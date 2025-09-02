use std::cmp::max;
use std::error::Error;
use std::io::Cursor;
use std::num::NonZero;
use std::time::Duration;

use open_cm_common::can::{CanBitTiming, CanChannelConfig};
use open_cm_common::tecmp::{CONTROL_MESSAGE_CONFIG_ID, InterfaceId, TECMP_ETHERTYPE, TecmpConfig};
use pcap::{Capture, Device};
use smoltcp::wire::{EthernetAddress, EthernetFrame};
use tecmp_rs::deku::writer::Writer;
use tecmp_rs::{CanData, CanDataFlags, DataType, deku::DekuContainerWrite};
use tecmp_rs::{
    MessageType, Tecmp, TecmpData, TecmpGlobalHeader,
    deku::{DekuUpdate as _, DekuWriter as _},
};

/// Locally administered MAC address
pub const LOCAL_MAC_ADDRESS: EthernetAddress =
    EthernetAddress([0x02, 0x6f, 0xbd, 0x3d, 0x4a, 0x00]);

fn main() -> Result<(), Box<dyn Error>> {
    let device = Device::lookup()?.unwrap();
    let mut cap = Capture::from_device(device)?.open()?;

    let mut buf = [0; 1500];

    let mut config_packet = EthernetFrame::new_checked(&mut buf)?;
    config_packet.set_src_addr(LOCAL_MAC_ADDRESS);
    config_packet.set_dst_addr(EthernetAddress::BROADCAST);
    config_packet.set_ethertype(TECMP_ETHERTYPE);

    let payload = config_packet.payload_mut();

    let tecmp = Tecmp {
        header: TecmpGlobalHeader {
            device_id: 1,
            counter: 0,
            version: 3,
            message_type: MessageType::ControlMessage,
            data_type: DataType::NoData,
            reserved: 0,
            device_flags: Default::default(),
        },
    };

    let mut data = TecmpData {
        interface_id: 1,
        timestamp: 0,
        length: 0,
        data: tecmp_rs::Data::NoData,
    };

    let mut channel_config = CanChannelConfig {
        interface_id: InterfaceId(2),
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
    let config_bytes = serde_json::to_vec(&config)?;
    data.length = config_bytes.len() as u16 + 4;

    let data_offset = tecmp.to_slice(payload).unwrap();
    let mut writer = Writer::new(Cursor::new(&mut payload[data_offset..]));
    data.to_writer(&mut writer, data.data.data_type()).unwrap();
    assert_eq!(writer.bits_written % 8, 0);
    let bytes_written = writer.bits_written / 8;

    payload[data_offset + bytes_written + 4..data_offset + bytes_written + 6]
        .copy_from_slice(&CONTROL_MESSAGE_CONFIG_ID.to_be_bytes());

    // + 2 bytes Data Flags + 2 bytes Device Id + 2 bytes Control Message ID
    let config_offset = data_offset + bytes_written + 6;
    payload[config_offset..config_offset + config_bytes.len()]
        .copy_from_slice(config_bytes.as_slice());

    let tx_len = max(
        64,
        EthernetFrame::<&[u8]>::buffer_len(config_offset + config_bytes.len()),
    );

    cap.sendpacket(&buf[..tx_len])?;
    std::thread::sleep(Duration::from_secs(1));

    buf.fill(0);
    let mut replay_packet = EthernetFrame::new_checked(&mut buf)?;
    replay_packet.set_src_addr(LOCAL_MAC_ADDRESS);
    replay_packet.set_dst_addr(EthernetAddress::BROADCAST);
    replay_packet.set_ethertype(TECMP_ETHERTYPE);

    let payload = replay_packet.payload_mut();

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
