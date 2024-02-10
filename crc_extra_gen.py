# Uses the

from pymavlink.dialects.v20.common import *
from pymavlink.generator.mavcrc import x25crc

# Create an instance of the GimbalDeviceSetAttitude message
# msg = MAVLink_gimbal_device_set_attitude_message(
#     target_system=0,
#     target_component=0,
#     flags=0,
#     q=[0, 0, 0, 0],
#     angular_velocity_x=0,
#     angular_velocity_y=0,
#     angular_velocity_z=0
# )

# todo: Not a valid message?
msg = MAVLink_gimbal_device_attitude_status_message(
    target_system=0,
    target_component=0,
    time_boot_ms=0,
    flags=0,
    q=[0, 0, 0, 0],
    angular_velocity_x=0,
    angular_velocity_y=0,
    angular_velocity_z=0,
    failure_flags=0,
)

print(f"CRC: {msg.crc_extra}")