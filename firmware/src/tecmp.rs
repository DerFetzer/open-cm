use core::cmp::max;

use rtic_sync::channel::Sender;
use smoltcp::{
    phy::{Device, RxToken, TxToken},
    time::Instant,
    wire::{EthernetAddress, EthernetFrame, EthernetProtocol},
};
use stm32h7xx_hal::ethernet;
use tecmp_rs::{
    DataIterator, DataType,
    deku::{DekuContainerRead, DekuContainerWrite},
};
use tecmp_rs::{
    DeviceFlags, MessageType, Tecmp, TecmpData, TecmpGlobalHeader,
    deku::{DekuWriter as _, no_std_io::Cursor, writer::Writer},
};

use crate::can::CanChannelConfig;

/// Locally administered MAC address
pub const LOCAL_MAC_ADDRESS: EthernetAddress =
    EthernetAddress([0x02, 0x6f, 0xbd, 0x3d, 0x4a, 0x04]);
/// Default TECMP multicast MAC address
pub const TECMP_DST_MAC_ADDRESS: EthernetAddress =
    EthernetAddress([0x01, 0x00, 0x5e, 0x00, 0x00, 0x00]);

pub const TECMP_ETHERTYPE: EthernetProtocol = EthernetProtocol::Unknown(0x99fe);

pub const CONTROL_MESSAGE_CONFIG_ID: u16 = 0x00f0;

pub const ED_NUM: usize = 8;
pub const TECMP_CHANNEL_SIZE: usize = 10;

#[derive(
    Debug, Copy, Clone, PartialEq, Eq, defmt::Format, serde::Serialize, serde::Deserialize,
)]
pub struct InterfaceId(pub u32);

pub struct TecmpHandler {
    ethdev: ethernet::EthernetDMA<ED_NUM, ED_NUM>,
    tx_counter: u16,
}

#[derive(Debug, Clone, defmt::Format)]
pub enum TecmpEvent {
    Data(TecmpData),
    Config(TecmpConfig),
}

#[derive(Debug, Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
pub enum TecmpConfig {
    Can(CanChannelConfig),
}

impl TecmpHandler {
    pub fn new(ethdev: ethernet::EthernetDMA<ED_NUM, ED_NUM>) -> Self {
        Self {
            ethdev,
            tx_counter: 0,
        }
    }

    pub fn send(&mut self, data: &TecmpData) {
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

        let buf_len = max(
            64,
            EthernetFrame::<&[u8]>::header_len() + tecmp.len() + data.len(),
        );

        if let Some(tx_token) = self.ethdev.transmit(Instant::from_micros_const(0)) {
            tx_token.consume(buf_len, |buf| {
                buf.fill(0);

                let mut eth = EthernetFrame::new_unchecked(buf);

                eth.set_dst_addr(TECMP_DST_MAC_ADDRESS);
                eth.set_src_addr(LOCAL_MAC_ADDRESS);
                eth.set_ethertype(TECMP_ETHERTYPE);

                let payload = eth.payload_mut();
                let data_offset = tecmp.to_slice(payload).unwrap();
                let mut writer = Writer::new(Cursor::new(&mut payload[data_offset..]));
                data.to_writer(&mut writer, data.data.data_type()).unwrap();
            })
        } else {
            defmt::error!("Could not get tx token!");
        }

        (self.tx_counter, _) = self.tx_counter.overflowing_add(1);
    }

    pub fn receive(&mut self, sender: &mut Sender<'static, TecmpEvent, TECMP_CHANNEL_SIZE>) {
        while let Some((rx_token, _)) = self.ethdev.receive(Instant::from_micros(0)) {
            rx_token.consume(|buf| {
                let eth = EthernetFrame::new_unchecked(buf);
                defmt::trace!(
                    "Received ethernet frame -> src: {}, dst: {}, ethertype: {}",
                    eth.src_addr(),
                    eth.dst_addr(),
                    eth.ethertype()
                );
                if eth.ethertype() == TECMP_ETHERTYPE {
                    defmt::info!("Received TECMP message (len={})", buf.len());
                    match Tecmp::from_bytes((eth.payload(), 0)) {
                        Ok(((buf, _), tecmp)) => {
                            if tecmp.header.message_type == MessageType::ReplayData
                                && (tecmp.header.data_type == DataType::Can
                                    || tecmp.header.data_type == DataType::CanFd)
                            {
                                let data_iterator = DataIterator::new(buf, tecmp.header.data_type);
                                for tecmp_data in data_iterator {
                                    defmt::info!("Got tecmp data: {:?}", tecmp_data);

                                    if sender.try_send(TecmpEvent::Data(tecmp_data)).is_err() {
                                        defmt::error!("Could not send tecmp data");
                                    }
                                }
                            } else if tecmp.header.message_type == MessageType::ControlMessage {
                                // Data Flags + Device ID
                                let message_id_offset = 2 + 2;
                                let message_id = u16::from_be_bytes(
                                    buf[message_id_offset..message_id_offset + 2]
                                        .try_into()
                                        .unwrap(),
                                );
                                if message_id != CONTROL_MESSAGE_CONFIG_ID {
                                    defmt::info!("Ignore tecmp control message due to config id");
                                } else {
                                    // + Control Message ID
                                    let payload_offset = message_id_offset + 2;
                                    let payload = &buf[payload_offset..];

                                    match serde_json_core::from_slice::<TecmpConfig>(payload) {
                                        Ok((config, _)) => {
                                            if sender.try_send(TecmpEvent::Config(config)).is_err()
                                            {
                                                defmt::error!("Could not send tecmp data");
                                            }
                                        }
                                        Err(e) => defmt::error!(
                                            "Could not convert payload to config: {}",
                                            e
                                        ),
                                    }
                                }
                            } else {
                                defmt::info!("Ignore tecmp message due to message or data type");
                            }
                        }
                        Err(e) => defmt::error!(
                            "Could not parse ethernet payload as tecmp: {}",
                            defmt::Display2Format(&e)
                        ),
                    }
                }
            })
        }
    }
}
