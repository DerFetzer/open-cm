use tecmp_rs::heapless::Vec;

#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinConfig {
    Config(LinChannelConfig),
    ScheduleTable(LinScheduleTable),
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinChannelConfig {
    pub bitrate: u16,
    pub checksum: LinChecksum,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinScheduleTable(pub Vec<LinScheduleSlot, 64>);

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LinScheduleSlot {
    pub pid: ProtectedIdentifier,
    pub delay_ms: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LinChecksum {
    Classic,
    Enhanced,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ProtectedIdentifier(pub u8);

impl ProtectedIdentifier {
    pub fn from_id(id: u8) -> Self {
        let mut pid = Self(id);
        pid.set_parity_bits();
        pid
    }

    pub fn is_valid(&self) -> bool {
        let mut valid_pid = *self;
        valid_pid.set_parity_bits();
        &valid_pid == self
    }

    pub fn set_parity_bits(&mut self) {
        let p0 = ((self.0 & 0b1)
            ^ ((self.0 >> 1) & 0b1)
            ^ ((self.0 >> 2) & 0b1)
            ^ ((self.0 >> 4) & 0b1))
            & 0b1;
        let p1 = !(((self.0 >> 1) & 0b1)
            ^ ((self.0 >> 3) & 0b1)
            ^ ((self.0 >> 4) & 0b1)
            ^ ((self.0 >> 5) & 0b1))
            & 0b1;
        self.0 = (self.0 & 0x3f) | (p0 << 6) | (p1 << 7);
    }

    pub fn get_id(&self) -> u8 {
        self.0 & 0x3f
    }
}

pub fn calc_lin_chksum(buf: &[u8], start: u8) -> u8 {
    let mut chksum: u8 = start;
    let mut overflow;

    for byte in buf {
        (chksum, overflow) = chksum.overflowing_add(*byte);
        if overflow {
            chksum += 1;
        }
    }
    !chksum
}
