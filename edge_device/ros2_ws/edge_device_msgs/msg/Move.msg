# Move command message for controlling robot motion.
# Contains the target robot's ID and movement parameters.
uint32 robot_id   # Target robot ID
float32 dx        # X velocity in m/s
float32 dy        # Y velocity in m/s
float32 dturn     # Rotation speed in rad/s