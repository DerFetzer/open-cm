use core::num::{NonZeroU8, NonZeroU16};

use fdcan::{
    BusMonitoringMode, ConfigMode, FdCan, Instance, NormalOperationMode, PoweredDownMode,
    config::{
        DataBitTiming, FrameTransmissionConfig, Interrupt, InterruptLine, Interrupts,
        NominalBitTiming,
    },
    filter::{ExtendedFilter, StandardFilter},
    frame::{FrameFormat, RxFrameInfo, TxFrameHeader},
    id::{ExtendedId, Id, StandardId},
};
use tecmp_rs::TecmpData;

use crate::tecmp::InterfaceId;

pub struct CanHandler<I: Instance> {
    state_wrapper: CanStateWrapper<I>,
    pub fifo: Fifo,
    pub interface_id: InterfaceId,
}

#[derive(Default)]
pub enum CanStateWrapper<I: Instance> {
    PoweredDown(FdCan<I, PoweredDownMode>),
    Config(FdCan<I, ConfigMode>),
    BusMonitoring(FdCan<I, BusMonitoringMode>),
    NormalOperation(FdCan<I, NormalOperationMode>),
    #[default]
    Dummy,
}

#[derive(Debug, Copy, Clone)]
pub enum Fifo {
    Fifo0,
    Fifo1,
}

#[derive(Debug, Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
pub struct CanBitTiming {
    pub prescaler: NonZeroU16,
    pub seg1: NonZeroU8,
    pub seg2: NonZeroU8,
    pub sync_jump_width: NonZeroU8,
}

#[derive(Debug, Clone, Copy, defmt::Format, serde::Serialize, serde::Deserialize)]
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

impl<I: Instance> CanHandler<I> {
    pub fn new(mut can: FdCan<I, ConfigMode>, fifo: Fifo, interface_id: InterfaceId) -> Self {
        match fifo {
            Fifo::Fifo0 => {
                can.set_standard_filter(
                    fdcan::filter::StandardFilterSlot::_0,
                    StandardFilter::accept_all_into_fifo0(),
                );
                can.set_extended_filter(
                    fdcan::filter::ExtendedFilterSlot::_0,
                    ExtendedFilter::accept_all_into_fifo0(),
                );
            }
            Fifo::Fifo1 => {
                can.set_standard_filter(
                    fdcan::filter::StandardFilterSlot::_0,
                    StandardFilter::accept_all_into_fifo1(),
                );
                can.set_extended_filter(
                    fdcan::filter::ExtendedFilterSlot::_0,
                    ExtendedFilter::accept_all_into_fifo1(),
                );
            }
        }
        can.set_automatic_retransmit(false);
        can.set_frame_transmit(FrameTransmissionConfig::AllowFdCanAndBRS);

        can.enable_interrupt_line(InterruptLine::_0, true);
        can.enable_interrupts(Interrupts::all());

        Self {
            state_wrapper: CanStateWrapper::Config(can),
            fifo,
            interface_id,
        }
    }

    pub fn enable(&mut self, monitoring: bool) {
        let state_wrapper = core::mem::take(&mut self.state_wrapper);
        self.state_wrapper = state_wrapper.enable(monitoring);
    }

    pub fn disable(&mut self) {
        let state_wrapper = core::mem::take(&mut self.state_wrapper);
        self.state_wrapper = state_wrapper.disable();
    }

    pub fn configure(&mut self, config: CanChannelConfig) {
        self.disable();
        let state_wrapper = core::mem::take(&mut self.state_wrapper);
        self.state_wrapper = state_wrapper.configure(config);
    }

    pub fn transmit(&mut self, header: TxFrameHeader, buf: &[u8]) -> bool {
        match &mut self.state_wrapper {
            CanStateWrapper::NormalOperation(fd_can) => fd_can.transmit(header, buf).is_ok(),
            CanStateWrapper::Config(_)
            | CanStateWrapper::PoweredDown(_)
            | CanStateWrapper::BusMonitoring(_) => {
                defmt::warn!("Cannot send in passive modes");
                false
            }
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
                false
            }
        }
    }

    pub fn transmit_tecmp_data(&mut self, data: &TecmpData) -> bool {
        if data.interface_id != self.interface_id.0 {
            defmt::warn!(
                "Tecmp data not for interface {} but {}",
                data.interface_id,
                self.interface_id.0
            );
            return false;
        }

        let (header, buf) = match &data.data {
            tecmp_rs::Data::Can(can_data) => (
                TxFrameHeader {
                    len: can_data.payload_length,
                    frame_format: FrameFormat::Standard,
                    id: if can_data.flags.ide || can_data.can_id & (1 << 31) != 0 {
                        Id::Extended(unsafe {
                            ExtendedId::new_unchecked(can_data.can_id & 0x3FFF_FFFF)
                        })
                    } else {
                        Id::Standard(unsafe {
                            StandardId::new_unchecked(can_data.can_id as u16 & 0x7FF)
                        })
                    },
                    bit_rate_switching: false,
                    marker: None,
                },
                can_data.payload.as_slice(),
            ),
            tecmp_rs::Data::CanFd(can_fd_data) => (
                TxFrameHeader {
                    len: can_fd_data.payload_length,
                    frame_format: FrameFormat::Standard,
                    id: if can_fd_data.flags.ide || can_fd_data.can_id & (1 << 31) != 0 {
                        Id::Extended(unsafe {
                            ExtendedId::new_unchecked(can_fd_data.can_id & 0x3FFF_FFFF)
                        })
                    } else {
                        Id::Standard(unsafe {
                            StandardId::new_unchecked(can_fd_data.can_id as u16 & 0x7FF)
                        })
                    },
                    bit_rate_switching: can_fd_data.flags.brs,
                    marker: None,
                },
                can_fd_data.payload.as_slice(),
            ),
            _ => return false,
        };
        self.transmit(header, buf)
    }

    pub fn receive(&mut self, buf: &mut [u8]) -> Option<RxFrameInfo> {
        match &mut self.state_wrapper {
            CanStateWrapper::BusMonitoring(fd_can) => match match self.fifo {
                Fifo::Fifo0 => fd_can.receive0(buf),
                Fifo::Fifo1 => fd_can.receive1(buf),
            } {
                Ok(or) => Some(or.unwrap()),
                Err(_) => None,
            },
            CanStateWrapper::NormalOperation(fd_can) => match match self.fifo {
                Fifo::Fifo0 => fd_can.receive0(buf),
                Fifo::Fifo1 => fd_can.receive1(buf),
            } {
                Ok(or) => Some(or.unwrap()),
                Err(_) => None,
            },

            CanStateWrapper::Config(_) | CanStateWrapper::PoweredDown(_) => {
                defmt::warn!("Cannot send in passive modes");
                None
            }
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
                None
            }
        }
    }

    pub fn has_interrupt(&mut self, interrupt: Interrupt) -> bool {
        match &mut self.state_wrapper {
            CanStateWrapper::PoweredDown(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::Config(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::BusMonitoring(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::NormalOperation(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
                false
            }
        }
    }

    pub fn clear_interrupts(&mut self, interrupts: Interrupts) {
        match &mut self.state_wrapper {
            CanStateWrapper::PoweredDown(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::Config(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::BusMonitoring(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::NormalOperation(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
            }
        }
    }
}

impl<I: Instance> CanStateWrapper<I> {
    pub fn enable(self, monitoring: bool) -> Self {
        match self {
            CanStateWrapper::PoweredDown(fd_can) => {
                let fd_can = fd_can.into_config_mode();
                if !monitoring {
                    Self::NormalOperation(fd_can.into_normal())
                } else {
                    Self::BusMonitoring(fd_can.into_bus_monitoring())
                }
            }
            CanStateWrapper::Config(fd_can) => {
                if !monitoring {
                    Self::NormalOperation(fd_can.into_normal())
                } else {
                    Self::BusMonitoring(fd_can.into_bus_monitoring())
                }
            }
            CanStateWrapper::BusMonitoring(_) | CanStateWrapper::NormalOperation(_) => {
                defmt::warn!("Already enabled");
                self
            }
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot enable dummy");
                self
            }
        }
    }

    pub fn disable(self) -> Self {
        match self {
            CanStateWrapper::BusMonitoring(fd_can) => Self::Config(fd_can.into_config_mode()),
            CanStateWrapper::NormalOperation(fd_can) => Self::Config(fd_can.into_config_mode()),
            CanStateWrapper::PoweredDown(_) | CanStateWrapper::Config(_) => {
                defmt::warn!("Already disabled");
                self
            }
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot enable dummy");
                self
            }
        }
    }

    pub fn configure(self, config: CanChannelConfig) -> Self {
        match self {
            CanStateWrapper::Config(mut fd_can) => {
                fd_can.set_nominal_bit_timing(NominalBitTiming {
                    prescaler: config.nominal_bit_timing.prescaler,
                    seg1: config.nominal_bit_timing.seg1,
                    seg2: config.nominal_bit_timing.seg2,
                    sync_jump_width: config.nominal_bit_timing.sync_jump_width,
                });
                fd_can.set_data_bit_timing(DataBitTiming {
                    transceiver_delay_compensation: config.transceiver_delay_compensation,
                    prescaler: NonZeroU8::new(
                        (config.data_bit_timing.prescaler.get() & 0xFF) as u8,
                    )
                    .unwrap(),
                    seg1: config.data_bit_timing.seg1,
                    seg2: config.data_bit_timing.seg2,
                    sync_jump_width: config.nominal_bit_timing.sync_jump_width,
                });
                fd_can.set_automatic_retransmit(config.automatic_retransmit);
                fd_can.set_protocol_exception_handling(config.protocol_exception_handling);

                if config.enabled {
                    CanStateWrapper::Config(fd_can).enable(config.monitoring)
                } else {
                    CanStateWrapper::Config(fd_can)
                }
            }
            CanStateWrapper::PoweredDown(_)
            | CanStateWrapper::BusMonitoring(_)
            | CanStateWrapper::NormalOperation(_) => {
                defmt::warn!("Already disabled");
                self
            }
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot enable dummy");
                self
            }
        }
    }

    pub fn transmit(&mut self, header: TxFrameHeader, buf: &[u8]) -> bool {
        match self {
            CanStateWrapper::NormalOperation(fd_can) => fd_can.transmit(header, buf).is_ok(),
            CanStateWrapper::Config(_)
            | CanStateWrapper::PoweredDown(_)
            | CanStateWrapper::BusMonitoring(_) => {
                defmt::warn!("Cannot send in passive modes");
                false
            }
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
                false
            }
        }
    }

    pub fn receive(&mut self, buf: &mut [u8], fifo: Fifo) -> Option<RxFrameInfo> {
        match self {
            CanStateWrapper::BusMonitoring(fd_can) => match match fifo {
                Fifo::Fifo0 => fd_can.receive0(buf),
                Fifo::Fifo1 => fd_can.receive1(buf),
            } {
                Ok(or) => Some(or.unwrap()),
                Err(_) => None,
            },
            CanStateWrapper::NormalOperation(fd_can) => match match fifo {
                Fifo::Fifo0 => fd_can.receive0(buf),
                Fifo::Fifo1 => fd_can.receive1(buf),
            } {
                Ok(or) => Some(or.unwrap()),
                Err(_) => None,
            },

            CanStateWrapper::Config(_) | CanStateWrapper::PoweredDown(_) => {
                defmt::warn!("Cannot send in passive modes");
                None
            }
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
                None
            }
        }
    }

    pub fn has_interrupt(&mut self, interrupt: Interrupt) -> bool {
        match self {
            CanStateWrapper::PoweredDown(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::Config(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::BusMonitoring(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::NormalOperation(fd_can) => fd_can.has_interrupt(interrupt),
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
                false
            }
        }
    }

    pub fn clear_interrupts(&mut self, interrupts: Interrupts) {
        match self {
            CanStateWrapper::PoweredDown(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::Config(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::BusMonitoring(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::NormalOperation(fd_can) => fd_can.clear_interrupts(interrupts),
            CanStateWrapper::Dummy => {
                defmt::error!("Cannot send in dummy state");
            }
        }
    }
}
