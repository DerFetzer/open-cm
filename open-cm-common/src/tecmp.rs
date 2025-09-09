use smoltcp::wire::{EthernetAddress, EthernetProtocol};
use tecmp_rs::TecmpData;

use crate::{can::CanChannelConfig, lin::LinConfig};

/// Locally administered MAC address
pub const CM_LOCAL_MAC_ADDRESS: EthernetAddress =
    EthernetAddress([0x02, 0x6f, 0xbd, 0x3d, 0x4a, 0x04]);
/// Default TECMP multicast MAC address
pub const TECMP_DST_MAC_ADDRESS: EthernetAddress =
    EthernetAddress([0x01, 0x00, 0x5e, 0x00, 0x00, 0x00]);

pub const TECMP_ETHERTYPE: EthernetProtocol = EthernetProtocol::Unknown(0x99fe);

pub const CONTROL_MESSAGE_CONFIG_ID: u16 = 0x00f0;

#[derive(Debug, Copy, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterfaceId(pub u32);

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TecmpEvent {
    Data(TecmpData),
    Config(TecmpConfig),
}

#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TecmpConfig {
    Can(CanChannelConfig),
    Lin(LinConfig),
}
