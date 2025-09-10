use core::convert::Infallible;

use heapless::Vec;
use open_cm_common::{
    lin::{
        LinChannelConfig, LinChecksum, LinScheduleSlot, LinScheduleTable, ProtectedIdentifier,
        calc_lin_chksum,
    },
    tecmp::InterfaceId,
};
use rtic_monotonics::fugit::RateExtU32;
use rtic_sync::channel::Sender;
use stm32h7xx_hal::{
    hal_02::serial::{Read, Write},
    nb,
    pac::UART7,
    serial::{
        self, Event, Rx, Serial, Tx,
        config::{BitOrder, Config, FifoThreshold, Parity, StopBits},
    },
};
use tecmp_rs::{LinData, LinDataFlags, TecmpData, deku::DekuUpdate};

use crate::tecmp::TECMP_CHANNEL_SIZE;

pub const LIN_INTERFACE_ID: InterfaceId = InterfaceId(0x3);

pub struct LinHandler {
    serial: Serial<UART7>,
    config: Config,
    checksum_mode: LinChecksum,
    slave_data: [Vec<u8, 9>; 64],
    current_frame: Option<(ProtectedIdentifier, u64)>,
    schedule_table: LinScheduleTable,
    next_schedule_index: usize,
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
            checksum_mode: LinChecksum::Classic,
            slave_data: [const { Vec::new() }; 64],
            current_frame: None,
            schedule_table: LinScheduleTable(Vec::new()),
            next_schedule_index: 0,
        }
    }

    fn modify_serial() {
        let register_block = unsafe { &*UART7::ptr() };

        // Disable peripheral, enable LIN mode and interrupts and enable again
        register_block.cr1.modify(|_, w| w.ue().disabled());
        register_block
            .cr2
            .modify(|_, w| w.linen().enabled().lbdie().enabled());
        // Set receiver timeout to 2 full bytes + Start/Stop bits
        register_block.rtor.modify(|_, w| w.rto().bits(20));
        register_block.cr2.modify(|_, w| w.rtoen().set_bit());
        register_block.cr1.modify(|_, w| w.ue().enabled());
    }

    pub fn set_config(&mut self, config: LinChannelConfig) {
        while !self.serial.is_idle() {}
        self.checksum_mode = config.checksum;
        self.config.baudrate((config.bitrate as u32).Hz());
        self.serial.reconfigure(self.config);
        Self::modify_serial();
    }

    pub fn get_checksum_mode(&self) -> LinChecksum {
        self.checksum_mode
    }

    pub fn set_slave_data(
        &mut self,
        pid: ProtectedIdentifier,
        mut data: Vec<u8, 9>,
        calc_checksum: Option<LinChecksum>,
    ) {
        if calc_checksum.is_some() && !data.is_empty() {
            let start_value = match calc_checksum {
                Some(LinChecksum::Enhanced) => pid.0,
                None | Some(LinChecksum::Classic) => 0,
            };
            let data_len = data.len();
            data[data_len - 1] = calc_lin_chksum(&data[..data_len - 2], start_value);
        }
        self.slave_data[pid.get_id() as usize] = data;
    }

    pub fn set_schedule_table(&mut self, schedule_table: LinScheduleTable) {
        self.schedule_table = schedule_table;
        self.next_schedule_index = 0;
    }

    pub fn next_master_slot(&mut self) -> Option<LinScheduleSlot> {
        let next_slot = self.schedule_table.0.get(self.next_schedule_index);
        self.next_schedule_index += 1;
        if self.next_schedule_index >= self.schedule_table.0.len() {
            self.next_schedule_index = 0;
        }
        next_slot.copied()
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

            // Clear and disable timeout interrupt
            registers.icr.write(|w| w.rtocf().set_bit());
            registers.cr1.modify(|_, w| w.rtoie().clear_bit());

            if let Some(current_pid) = self.current_frame.take() {
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

    pub fn handle_receiver_timeout(
        &mut self,
        tecmp_s: &mut Sender<'static, TecmpData, TECMP_CHANNEL_SIZE>,
    ) -> nb::Result<u8, serial::Error> {
        let registers = unsafe { &*UART7::ptr() };
        if registers.isr.read().rtof().bit_is_set() {
            defmt::info!("Receiver timeout detected");

            // Clear and disable timeout interrupt
            registers.icr.write(|w| w.rtocf().set_bit());
            registers.cr1.modify(|_, w| w.rtoie().clear_bit());

            if let Some((current_pid, timestamp)) = self.current_frame.take() {
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

                let (payload_length, payload, checksum) = if data.is_empty() {
                    (0, Vec::new(), 0)
                } else {
                    (
                        (data.len() as u8) - 1,
                        Vec::from_slice(&data[..data.len() as usize - 1]).unwrap(),
                        data[data.len() - 1],
                    )
                };

                let mut tecmp_data = TecmpData {
                    interface_id: LIN_INTERFACE_ID.0,
                    timestamp,
                    length: 0, // Set later
                    data: tecmp_rs::Data::Lin(LinData {
                        flags: LinDataFlags {
                            parity_err_use_parity: !current_pid.is_valid(),
                            no_slave_response: data.is_empty(),
                            // checksum_err: todo!(),
                            ..Default::default()
                        },
                        lin_id: current_pid.get_id(),
                        payload_length,
                        payload,
                        checksum,
                    }),
                };
                tecmp_data.update().unwrap();
                if tecmp_s.try_send(tecmp_data).is_err() {
                    defmt::warn!("Could not send tecmp data to channel");
                }
            } else {
                defmt::error!("Unexpected receiver timeout interrupt");
            }
        }
        Ok(0)
    }

    pub fn handle_rx_fifo_threshold(&mut self, timestamp: u64) -> nb::Result<(), Infallible> {
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
            self.current_frame = Some((pid, timestamp));

            defmt::trace!("Slave data: {:?}", self.slave_data);
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

            registers.icr.write(|w| w.rtocf().set_bit());
            registers.cr1.modify(|_, w| w.rtoie().set_bit());
        }
        Ok(())
    }
}
