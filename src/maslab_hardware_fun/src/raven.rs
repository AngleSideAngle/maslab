// Raven driver stuff

use std::time::Duration;

use serialport::SerialPort;

pub enum MotorChannel {
    CH1 = 0,
    CH2 = 1,
    CH3 = 2,
    CH4 = 3,
    CH5 = 4,
}

pub enum MotorMode {
    Disable = 0x00,
    Direct = 0x01,
    Position = 0x02,
    Velocity = 0x03,
}

pub enum ServoChannel {
    CH1 = 0,
    CH2 = 1,
    CH3 = 2,
    CH4 = 3,
}

pub enum MessageType {
    ServoValue = 0 << 3,
    MotorMode = 1 << 3,
    MotorPID = 2 << 3,
    MotorTarget = 3 << 3,
    MotorVoltage = 4 << 3,
    MotorCurrent = 5 << 3,
    EncoderValue = 6 << 3,
    MotorVelocityValue = 7 << 3, // Read-only
    MotorVoltageValue = 8 << 3,  // Read-only
    MotorCurrentValue = 9 << 3,  // Read-only
    Reset = 10 << 3,             // Write-only
}

pub enum ReadWrite {
    Read = 0x00,
    Write = 0x80,
}

// """
// /***
// * Message format
// * Start: 1 byte
// *   [7-0] start byte
// * Header: 1 byte
// *   [7] ack
// *   [6-2] length
// *   [1-0] PID (unused)
// * Data: at least 1 byte
// *   header: 1 byte
// *     [7] 0: read, 1: write
// *     [6-0] header type
// *   data bytes : any number of bytes
// *    ...
// * CRC: 1 byte
// *   [7-0] 8-bit CRC
// */
// """

const SERIAL_START: u32 = 0xAA;
const RAVEN_BAUD_RATE: u32 = 460800;
const SMBUS_POLY: u32 = 0x07;

struct CRC8Encoder(Vec<u32>);

impl CRC8Encoder {
    fn new(poly: Option<u32>) -> Self {
        let poly = poly.unwrap_or(SMBUS_POLY);
        let mut table = Vec::new();
        for i in 0..256 {
            let mut remainder = i;
            for _bit in 0..8 {
                remainder = if remainder & 0x80 != 0 {
                    (remainder << 1) ^ poly
                } else {
                    remainder << 1
                };
            }
            table.push(remainder & 0xFF);
        }
        Self(table)
    }

    fn crc(data: [u32]) -> u32 {
        let remainder = 0;
    }
}

struct RavenSerial {}

fn test() {
    let mut port =
        serialport::new("/dev/ttyUSB0", RAVEN_BAUD_RATE).timeout(Duration::from_millis(1));
}
