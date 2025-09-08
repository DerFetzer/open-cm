#![no_main]
#![no_std]

pub mod can;
pub mod lin;
pub mod tecmp;

use defmt_rtt as _; // global logger

use stm32h7xx_hal as _;

use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes a semihosting-capable debug tool exit
/// with status code 0.
pub fn exit() -> ! {
    semihosting::process::exit(0);
}

/// Hardfault handler.
///
/// Terminates the application and makes a semihosting-capable debug tool exit
/// with an error. This seems better than the default, which is to spin in a
/// loop.
#[cortex_m_rt::exception]
unsafe fn HardFault(_frame: &cortex_m_rt::ExceptionFrame) -> ! {
    semihosting::process::exit(1);
}

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::{assert, assert_eq};

    use crate::lin::{ProtectedIdentifier, calc_lin_chksum};

    #[test]
    fn lin_pid() {
        let valid_pid = ProtectedIdentifier(0x42);
        assert!(valid_pid.is_valid());
        assert_eq!(valid_pid.get_id(), 0x02);

        let valid_pid = ProtectedIdentifier(0x1A);
        assert!(valid_pid.is_valid());
        assert_eq!(valid_pid.get_id(), 0x1A);
    }

    #[test]
    fn lin_chksum() {
        let buf = [0x4A, 0x55, 0x93, 0xE5];
        assert_eq!(calc_lin_chksum(&buf, 0), 0xe6);
    }
}
