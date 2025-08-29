use fdcan::{
    BusMonitoringMode, ConfigMode, FdCan, Instance, NormalOperationMode, PoweredDownMode,
    config::{FrameTransmissionConfig, Interrupt, InterruptLine, Interrupts},
    filter::{ExtendedFilter, StandardFilter},
    frame::{RxFrameInfo, TxFrameHeader},
};

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

impl<I: Instance> CanStateWrapper<I> {
    pub fn new(mut can: FdCan<I, ConfigMode>, fifo: Fifo) -> Self {
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

        Self::Config(can)
    }
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
