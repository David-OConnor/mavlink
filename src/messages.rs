use crate::MessageType;

use lin_alg::f32::Quaternion;

#[derive(Default)]
pub struct GimbalDeviceSetAttitude {
    pub target_system: u8,
    pub target_component: u8,
    pub flags: u16,
    // w, x, y, z
    pub q: Quaternion,
    pub angular_velocity_x: f32,
    pub angular_velocity_y: f32,
    pub angular_velocity_z: f32,
}

impl GimbalDeviceSetAttitude {
    /// Helper function to init with some defaults, like no angular velocity.
    pub fn new(target_system: u8, target_component: u8, q: Quaternion) -> Self {
        Self {
            target_system,
            target_component,
            q,
            ..Default::default()
        }
    }

    pub fn to_bytes(&self) -> [u8; MessageType::GimbalDeviceSetAttitude.payload_size()] {
        let mut result = [0; MessageType::GimbalDeviceSetAttitude.payload_size()];

        result[0] = self.target_system;
        result[1] = self.target_component;
        result[2..4].clone_from_slice(&self.flags.to_le_bytes());
        result[4..8].clone_from_slice(&self.q.w.to_le_bytes());
        result[8..12].clone_from_slice(&self.q.x.to_le_bytes());
        result[12..16].clone_from_slice(&self.q.y.to_le_bytes());
        result[16..20].clone_from_slice(&self.q.z.to_le_bytes());
        result[20..24].clone_from_slice(&self.q.z.to_le_bytes());
        result[24..28].clone_from_slice(&self.q.z.to_le_bytes());
        result[28..32].clone_from_slice(&self.q.z.to_le_bytes());
        result
    }
}