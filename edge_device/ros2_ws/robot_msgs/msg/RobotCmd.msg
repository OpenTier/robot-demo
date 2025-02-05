# CombinedCommand.msg

# Command type constants
uint8 COMMAND_MOVE = 0
uint8 COMMAND_BEEP = 1
uint8 COMMAND_KICK = 2
uint8 COMMAND_LEDS = 3

# Command identifier to indicate which command is active.
uint8 command_id

# Raw payload containing the command data.
#
# The interpretation of this 12-byte payload depends on the value of command_id:
#
# - If command_id == COMMAND_MOVE (0):
#     The payload should contain three 32-bit floating point values (total 12 bytes):
#         dx    - X velocity in m/s.
#         dy    - Y velocity in m/s.
#         dturn - Rotation speed in rad/s.
#
# - If command_id == COMMAND_BEEP (1):
#     The payload should contain two 32-bit unsigned integers (total 8 bytes):
#         frequency - Beep frequency in Hz.
#         duration  - Beep duration in milliseconds.
#     The remaining 4 bytes are unused or reserved.
#
# - If command_id == COMMAND_KICK (2):
#     The payload should contain one 32-bit floating point value (4 bytes total):
#         power - Kick power (range 0.0 to 1.0).
#     The remaining 8 bytes are unused or reserved.
#
# - If command_id == COMMAND_LEDS (3):
#     The payload should contain three 8-bit unsigned integers (total 3 bytes):
#         red   - Red intensity (0-255).
#         green - Green intensity (0-255).
#         blue  - Blue intensity (0-255).
#     The remaining 9 bytes are unused or reserved.
uint8[12] payload
