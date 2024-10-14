## Stroam Manager Package
ROS2 package contains a manager node for routing controller inputs.
Turret button presses are sent to turret_button, turret joy-stick moves sent
to turret_joy, and motors joy-stick moves to motor_controller nodes.

# Dependencies
- [`stroam_interfaces`] (../stroam_interfaces) : Contains definitions for Stroam's instructions (actions) and Xbox controller inputs (message)
- [`turret`] (../../../)
