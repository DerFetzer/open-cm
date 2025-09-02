use core::num::{NonZeroU8, NonZeroU16};

use crate::tecmp::InterfaceId;

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CanBitTiming {
    pub prescaler: NonZeroU16,
    pub seg1: NonZeroU8,
    pub seg2: NonZeroU8,
    pub sync_jump_width: NonZeroU8,
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CanChannelConfig {
    pub interface_id: InterfaceId,
    pub enabled: bool,
    pub monitoring: bool,
    pub nominal_bit_timing: CanBitTiming,
    pub data_bit_timing: CanBitTiming,
    pub transceiver_delay_compensation: bool,
    pub automatic_retransmit: bool,
    pub protocol_exception_handling: bool,
}
