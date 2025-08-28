use fdcan::{
    BusMonitoringMode, ConfigMode, FdCan, Instance, NormalOperationMode, PoweredDownMode,
    config::FdCanConfig,
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

impl<I: Instance> CanStateWrapper<I> {
    pub fn enable(self, config: FdCanConfig, monitoring: bool) -> Self {
        match self {
            CanStateWrapper::PoweredDown(fd_can) => {
                let fd_can = fd_can.into_config_mode();
                if !monitoring {
                    Self::NormalOperation(fd_can.into_normal())
                } else {
                    Self::BusMonitoring(fd_can.into_bus_monitoring())
                }
            }
            CanStateWrapper::Config(mut fd_can) => {
                fd_can.apply_config(config);
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
}
