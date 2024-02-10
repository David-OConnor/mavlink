//! Common types used by both the Sail firmware, and its ground control station.
//! todo: Consider moving this to a separate lib or A/R, either for Sail common,
//! or Mavlink.

#![no_std]

use core::sync::atomic::{AtomicU8, Ordering};

pub const MAVLINK_MSG_START: u8 = 0xfd;

pub const MAVLINK_SIZE: usize = 12; // includes trailing CRC
pub const MAVLINK_PAYLOAD_START_I: usize = 10;

const X25_INIT_CRC: u16 = 0xffff;
const _X25_VALIDATE_CRC: u16 = 0xf0b8;

const SYSTEM_ID: u8 = 110;
const COMPONENT_ID: u8 = 110;

pub struct ParseError {}

pub mod messages;
pub use messages::*;

static SEQUENCE_NUMBER: AtomicU8 = AtomicU8::new(0);

#[repr(u32)]
#[derive(Clone, Copy)]
pub enum MsgType {
    VehicleToGc = 1_000,
    GcToVehicle = 1_001,
    SystemStatus = 1_002,
    GimbalDeviceSetAttitude = 284,
    GimbalDeviceAttitudeStatus = 285,
}

impl MsgType {
    pub const fn payload_size(&self) -> usize {
        match self {
            Self::VehicleToGc => 48,
            Self::GcToVehicle => 29,
            Self::SystemStatus => 2,
            Self::GimbalDeviceSetAttitude => 99,
            Self::GimbalDeviceAttitudeStatus => 137,
        }
    }

    pub const fn crc_extra(&self) -> u8 {
        match self {
            Self::VehicleToGc => 0,
            Self::GcToVehicle => 0,
            Self::SystemStatus => 0,
            // todo!
            Self::GimbalDeviceSetAttitude => 69,
            Self::GimbalDeviceAttitudeStatus => 69,
        }
    }

    /// Assists in patern-matching IDs received in serial communications.
    pub fn from_id(id: u32) -> Result<Self, ParseError> {
        Ok(match id {
            1_000 => Self::VehicleToGc,
            1_001 => Self::GcToVehicle,
            1_002 => Self::SystemStatus,
            284 => Self::GimbalDeviceSetAttitude,
            285 => Self::GimbalDeviceAttitudeStatus,

            _ => return Err(ParseError {}),
        })
    }
}

// #[repr(u32)]
// /// Only the ones we use. u32 repr is the message id.
// pub enum MavLinkMessage {
//     NavWaypoint = 16,
// }
//
// impl MavLinkMessage {
//     /// https://github.com/mavlink/c_library_v2/blob/master/all/all.h#L26
//     pub fn base_crc(&self) -> u8 {
//         match self {
//             Self::NavWaypoint => 0,
//         }
//     }
// }

// todo: Security, later.

/// https://github.com/mavlink/c_library_v2/blob/master/checksum.h
fn crc_accumulate(data: u8, crc_accum: &mut u16) {
    // Accumulate one byte of data into the CRC
    let mut tmp = data as u16 ^ *crc_accum;
    tmp ^= tmp << 4;
    *crc_accum = (*crc_accum >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}

pub fn crc_calculate(buf: &[u8], length: u16, extra: u8) -> u16 {
    let mut result: u16 = X25_INIT_CRC;
    for i in 0..length {
        crc_accumulate(buf[(length - 1 - i) as usize], &mut result);
    }
    crc_accumulate(extra, &mut result);
    result
}

// fn crc_accumulate_buffer(crcAccum: &mut u16, pBuffer: &[u8], length: u16) {
//     for i in 0..length {
//         crc_accumulate(pBuffer[(length - 1 - i) as usize], crcAccum);
//     }
// }

pub struct MavlinkPacket {
    /// Flags that must be understood for MAVLink compatibility (implementation discards packet if
    /// it does not understand flag).
    pub incompat_flags: u8,
    /// Flags that can be ignored if not understood (implementation can still handle packet even if
    /// it does not understand flag).
    pub compat_flags: u8,
    /// Used to detect packet loss. Components increment value for each message sent.
    pub sequence_number: u8,
    /// ID of system (vehicle) sending the message. Used to differentiate systems on network.
    /// Note that the broadcast address 0 may not be used in this field as it is an invalid source address.
    pub sysid: u8,
    /// ID of component sending the message. Used to differentiate components in a system (e.g. autopilot and a camera).
    /// Use appropriate values in MAV_COMPONENT. Note that the broadcast address MAV_COMP_ID_ALL may not be
    /// used in this field as it is an invalid source address.
    pub compid: u8,
    /// ID of message type in payload. Used to decode data back into message object.
    /// 24 bytes.
    pub message_type: MsgType,
}

impl MavlinkPacket {
    pub fn new(message_type: MsgType) -> Self {
        Self {
            incompat_flags: 0,
            compat_flags: 0,
            // `fetch_add` wraps on overflow.
            sequence_number: SEQUENCE_NUMBER.fetch_add(1, Ordering::Relaxed),
            sysid: SYSTEM_ID,
            compid: COMPONENT_ID,
            message_type,
        }
    }

    pub fn to_buf(&self, buf: &mut [u8], payload: &[u8]) {
        // todo: Error handling on buffer len.

        let payload_len = self.message_type.payload_size();

        let checksum = crc_calculate(payload, payload_len as u16, self.message_type.crc_extra());

        buf[0] = MAVLINK_MSG_START;
        buf[1] = payload_len as u8;
        buf[2] = self.incompat_flags;
        buf[3] = self.compat_flags;
        buf[4] = self.sequence_number;
        buf[5] = self.sysid;
        buf[6] = self.compid;
        // todo: is this order right? "low, middle, high". Seems to be from tests.
        buf[7..10].clone_from_slice(&(self.message_type as u32).to_le_bytes()[..3]);
        buf[10..10 + payload_len as usize].clone_from_slice(&payload[..payload_len as usize]);
        buf[10 + payload_len as usize..12 + payload_len as usize]
            .clone_from_slice(&checksum.to_le_bytes());
    }

    // /// Calculate a 8-bit checksum of the key fields of a message, so we
    // /// can detect incompatible XML changes
    // /// https://mavlink.io/en/guide/serialization.html#checksum
    // fn calc_checksum(&self) {
    //     let crc = x25crc();
    //     crc.accumulate_str(msg.name + ' ');
    //     // in order to allow for extensions the crc does not include
    //     // any field extensions
    //     crc_end = msg.base_fields();
    //     for i in 0..crc_end {
    //         let f = msg.ordered_fields[i];
    //         crc.accumulate_str(f.type + ' ');
    //         crc.accumulate_str(f.name + ' ');
    //         if f.array_length {
    //             crc.accumulate([f.array_length])
    //         }
    //    (crc.crc&0xFF) ^ (crc.crc>>8)
    // }
}
//
// fn mavlink_header() -> MavHeader {
//     MavHeader {
//         system_id: 1,
//         component_id: 1,
//         sequence: 42,
//     }
// }
//
// pub fn mavlink_heartbeat_message() -> MavMessage {
//     MavMessage::HEARTBEAT(mavlink::common::HEARTBEAT_DATA {
//         custom_mode: 0,
//         mavtype: mavlink::common::MavType::MAV_TYPE_SUBMARINE,
//         autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
//         base_mode: mavlink::common::MavModeFlag::empty(),
//         system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
//         mavlink_version: 0x3,
//     })
// }

/// todo: Can't find data types for this type.
/// Mavlink message.
pub struct NavWaypoint {
    pub hold_time: f32,
    pub accept_radius: f32,
    pub pass_radius: f32,
    pub yaw: f32,
    pub latitude: i32,
    pub longitude: i32,
    pub altitude: f32,
}

/// Mavlink message
pub struct Attitude {
    pub q1: f32,         // w,
    pub q2: f32,         // x,
    pub q3: f32,         // y,
    pub q4: f32,         // z,
    pub roll_speed: f32, //rad/s
    pub pitch_speed: f32,
    pub yaw_speed: f32,
    /// Likely N/A
    pub repr_offset: [f32; 4],
}

// /// Decoupled from, but similar to that returned by Semtech radios.
// /// Raw format for ease of transmission.
// pub struct LinkStats {
//     /// Divide this by 2 and negate.
//     /// todo: Which of the 2 returned by the radio should we use here?
//     pub rssi: u8,
//     /// Divide this by 4.
//     pub snr: u8,
//     /// Portion of receive transmitted packets received, out of 255.
//     pub link_quality: u8,
// }

// pub struct LinkNodeStatus {
//
// }
