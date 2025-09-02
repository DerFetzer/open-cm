use std::{error::Error, io::Cursor};

use open_cm_common::tecmp::{CONTROL_MESSAGE_CONFIG_ID, TECMP_ETHERTYPE, TecmpConfig};
use smoltcp::wire::{EthernetAddress, EthernetFrame};
use tecmp_rs::{
    DataType, MessageType, Tecmp, TecmpData, TecmpGlobalHeader,
    deku::{DekuContainerWrite as _, DekuUpdate as _, DekuWriter as _, writer::Writer},
};

pub fn write_tecmp_replay_packet(
    buf: &mut [u8],
    src_mac: [u8; 6],
    data: tecmp_rs::Data,
    interface_id: u32,
) -> Result<usize, Box<dyn Error>> {
    let mut replay_packet = EthernetFrame::new_checked(buf)?;
    replay_packet.set_src_addr(EthernetAddress(src_mac));
    replay_packet.set_dst_addr(EthernetAddress::BROADCAST);
    replay_packet.set_ethertype(TECMP_ETHERTYPE);

    let payload = replay_packet.payload_mut();

    let tecmp = Tecmp {
        header: TecmpGlobalHeader {
            device_id: 1,
            counter: 0,
            version: 3,
            message_type: MessageType::ReplayData,
            data_type: data.data_type(),
            reserved: 0,
            device_flags: Default::default(),
        },
    };

    let mut data = TecmpData {
        interface_id,
        timestamp: 0,
        length: 0,
        data,
    };
    data.update()?;

    let data_offset = tecmp.to_slice(payload)?;
    let mut writer = Writer::new(Cursor::new(&mut payload[data_offset..]));
    data.to_writer(&mut writer, data.data.data_type())?;
    writer.finalize()?;

    Ok(EthernetFrame::<&[u8]>::buffer_len(tecmp.len() + data.len()))
}

pub fn write_config_packet(
    buf: &mut [u8],
    src_mac: [u8; 6],
    config: TecmpConfig,
) -> Result<usize, Box<dyn Error>> {
    let mut config_packet = EthernetFrame::new_checked(buf)?;
    config_packet.set_src_addr(EthernetAddress(src_mac));
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
        interface_id: 0,
        timestamp: 0,
        length: 0,
        data: tecmp_rs::Data::NoData,
    };

    let config_bytes = serde_json::to_vec(&config)?;
    data.length = config_bytes.len() as u16 + 4;

    let data_offset = tecmp.to_slice(payload)?;

    let mut writer = Writer::new(Cursor::new(&mut payload[data_offset..]));
    data.to_writer(&mut writer, data.data.data_type())?;

    assert_eq!(writer.bits_written % 8, 0);
    let bytes_written = writer.bits_written / 8;

    payload[data_offset + bytes_written + 4..data_offset + bytes_written + 6]
        .copy_from_slice(&CONTROL_MESSAGE_CONFIG_ID.to_be_bytes());

    // + 2 bytes Data Flags + 2 bytes Device Id + 2 bytes Control Message ID
    let config_offset = data_offset + bytes_written + 6;
    payload[config_offset..config_offset + config_bytes.len()]
        .copy_from_slice(config_bytes.as_slice());

    Ok(EthernetFrame::<&[u8]>::buffer_len(
        config_offset + config_bytes.len(),
    ))
}
