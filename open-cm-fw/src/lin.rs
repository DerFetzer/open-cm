use core::convert::Infallible;

use heapless::Vec;
use open_cm_common::tecmp::InterfaceId;
use rtic_sync::channel::Sender;
use stm32h7xx_hal::{
    hal_02::serial::{Read, Write},
    nb,
    pac::UART7,
    serial::{
        self, Event, Rx, Serial, Tx,
        config::{BitOrder, Config, FifoThreshold, Parity, StopBits},
    },
    time::Hertz,
};
use tecmp_rs::{LinData, LinDataFlags, TecmpData, deku::DekuUpdate};

use crate::tecmp::TECMP_CHANNEL_SIZE;

const INTERFACE_ID: InterfaceId = InterfaceId(0x3);

pub struct LinHandler {
    serial: Serial<UART7>,
    config: Config,
    slave_data: [Vec<u8, 9>; 64],
    current_pid: Option<ProtectedIdentifier>,
}

pub enum LinChecksum {
    Classic,
    Enhanced(ProtectedIdentifier),
}

#[derive(Copy, Clone, PartialEq, Eq, defmt::Format)]
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

impl LinHandler {
    pub fn new(serial: Serial<UART7>, config: Config) -> Self {
        assert!(config.stopbits == StopBits::Stop1);
        assert!(config.parity == Parity::ParityNone);
        assert!(config.bitorder == BitOrder::LsbFirst);
        assert!(!config.swaptxrx && !config.invertrx && !config.inverttx && !config.halfduplex);
        // Interrupt has to fire when two words are in it to detect master requests
        assert!(config.rxfifothreshold == FifoThreshold::Eighth);

        Self::modify_serial();

        Self {
            serial,
            config,
            slave_data: [const { Vec::new() }; 64],
            current_pid: None,
        }
    }

    fn modify_serial() {
        let register_block = unsafe { &*UART7::ptr() };

        // Disable peripheral, enable LIN mode and interrupts and enable again
        register_block.cr1.modify(|_, w| w.ue().disabled());
        register_block
            .cr2
            .modify(|_, w| w.linen().enabled().lbdie().enabled());
        register_block.cr1.modify(|_, w| w.ue().enabled());
    }

    pub fn set_bitrate(&mut self, bitrate: Hertz) {
        while !self.serial.is_idle() {}
        self.config.baudrate(bitrate);
        self.serial.reconfigure(self.config);
        Self::modify_serial();
    }

    pub fn set_slave_data(
        &mut self,
        id: u8,
        mut data: Vec<u8, 9>,
        calc_chksum: Option<LinChecksum>,
    ) {
        if calc_chksum.is_some() {
            let start_value = match calc_chksum {
                Some(LinChecksum::Enhanced(pid)) => pid.0,
                None | Some(LinChecksum::Classic) => 0,
            };
            let data_len = data.len();
            data[data_len - 1] = calc_lin_chksum(&data[..data_len - 2], start_value);
        }
        self.slave_data[id as usize] = data;
    }

    pub fn send_master_request(&mut self, pid: ProtectedIdentifier) -> nb::Result<(), Infallible> {
        defmt::info!("Send master request for pid={}(id={})", pid, pid.get_id());
        // Wait for active transmit to finish
        while !self.serial.is_txe() {}
        // Send break
        unsafe { &*UART7::ptr() }.rqr.write(|w| w.sbkrq().set_bit());
        let mut tx: Tx<UART7> = unsafe { core::mem::zeroed() };
        // Send Sync
        tx.write(0x55)?;
        // Send PID
        tx.write(pid.0)?;
        Ok(())
    }

    pub fn handle_break_detected(&mut self) {
        let registers = unsafe { &*UART7::ptr() };
        if registers.isr.read().lbdf().bit_is_set() {
            defmt::info!("Break detected");
            // Clear interrupt flag
            registers.icr.write(|w| w.lbdcf().set_bit());

            self.serial.unlisten(Event::Rxftie);
            registers.icr.write(|w| w.idlecf().set_bit());
            self.serial.unlisten(Event::Idle);

            if let Some(current_pid) = self.current_pid.take() {
                defmt::warn!(
                    "Break detected during waiting for bytes for id {:#x}",
                    current_pid
                );
            }

            // Clear receive FIFO
            let mut rx: Rx<UART7> = unsafe { core::mem::zeroed() };
            while rx.is_rxne() {
                let _ = rx.read();
            }

            // Interrupt fires after two received bytes
            self.serial.listen(Event::Rxftie);
        }
    }

    pub fn handle_idle(
        &mut self,
        timestamp: u64,
        tecmp_s: &mut Sender<'static, TecmpData, TECMP_CHANNEL_SIZE>,
    ) -> nb::Result<u8, serial::Error> {
        let registers = unsafe { &*UART7::ptr() };
        if registers.isr.read().idle().bit_is_set() {
            defmt::info!("Idle detected");

            // Clear interrupt flag
            registers.icr.write(|w| w.idlecf().set_bit());
            self.serial.unlisten(Event::Idle);

            if let Some(current_pid) = self.current_pid.take() {
                // Read receive FIFO
                let mut rx: Rx<UART7> = unsafe { core::mem::zeroed() };

                let mut data: Vec<u8, 9> = Vec::new();
                while rx.is_rxne() {
                    if data.push(rx.read()?).is_err() {
                        defmt::warn!("More than 9 bytes in FIFO");
                    }
                }
                defmt::info!(
                    "Read {} bytes for id {:#x}: {:?}",
                    data.len(),
                    current_pid,
                    data
                );

                let mut tecmp_data = TecmpData {
                    interface_id: INTERFACE_ID.0,
                    timestamp,
                    length: 0, // Set later
                    data: tecmp_rs::Data::Lin(LinData {
                        flags: LinDataFlags {
                            parity_err: !current_pid.is_valid(),
                            no_slave_response: data.is_empty(),
                            // checksum_err: todo!(),
                            ..Default::default()
                        },
                        lin_id: current_pid.get_id(),
                        payload_length: (data.len() as u8) - 1,
                        payload: Vec::from_slice(&data[..data.len() as usize - 1]).unwrap(),
                        checksum: data[data.len() - 1],
                    }),
                };
                tecmp_data.update().unwrap();
                if tecmp_s.try_send(tecmp_data).is_err() {
                    defmt::warn!("Could not send tecmp data to channel");
                }
            } else {
                defmt::error!("Unexpected Idle interrupt");
            }
        }
        Ok(0)
    }

    pub fn handle_rx_fifo_threshold(&mut self) -> nb::Result<(), Infallible> {
        let registers = unsafe { &*UART7::ptr() };
        if registers.isr.read().rxft().bit_is_set() && registers.cr3.read().rxftie().bit_is_set() {
            defmt::info!("Received 2 bytes in serial FIFO");

            self.serial.unlisten(Event::Rxftie);

            let mut rx: Rx<UART7> = unsafe { core::mem::zeroed() };
            let sync = rx.read().unwrap();
            let pid = ProtectedIdentifier(rx.read().unwrap());

            defmt::info!(
                "Received sync={:#x} and pid={:#x}({:#x})]",
                sync,
                pid,
                pid.get_id()
            );
            if sync != 0x55 {
                defmt::warn!("Invalid sync value: {:#x}", sync);
                return Ok(());
            }
            self.current_pid = Some(pid);

            let slave_data = &self.slave_data[pid.get_id() as usize];
            if !slave_data.is_empty() {
                defmt::info!(
                    "Send LIN slave data for id={}: {:?}",
                    pid.get_id(),
                    slave_data
                );

                let mut tx: Tx<UART7> = unsafe { core::mem::zeroed() };
                for byte in slave_data {
                    tx.write(*byte)?;
                }
            }

            registers.icr.write(|w| w.idlecf().set_bit());
            self.serial.listen(Event::Idle);
        }
        Ok(())
    }
}
