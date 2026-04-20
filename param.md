# Mechanical
## Frame:
    Outter diameters: 1x1m
    Wheel positions: {(x, y) | x = 640 mm, y = 640 mm}
---
## Swerve module:
    Wheel diameter: 145 mm
    Frame height: 219.25 mm
---
# Electronics
    -> Control Board
    - Logic level shifter -> ESP32
    - ODrive -> ESP32 UART
    - Step Driver (DM556) -> ESP32 Motion controller (Step/dir)
    - 2 Dir Encoders -> ESP32 interrupts 
    - USB to Serial converter (?)
---
# Electrical
## Sensors
    - Encoder: {600 PPR incremental, 600 PPR incremental, 600 PPR inc, 600 PPR inc} (2400 CPR)
    - IMU: {NBo055}
## Power
    - [MCU power](https://wiki.st.com/stm32mcu/wiki/Basics_of_power_supply_design_for_MCU)
---

## Battery
 - 3 x 12V Battery
---

# Control & Program
<!--  features --> [5], [6] 
## Kinematics Model 
## Speed Normalization
> Pre-normalize &rarr; (Needs maximum motor RPM)
> Post-normalize &rarr; (Needs maximum robot speed)
## Direction Flipping
## Stray Module Problem
## Control pannel UI

# REF
[1] [ODrive Documentation](https://docs.odriverobotics.com/)
[2] [Odrive Arduino](https://github.com/odriverobotics/ODriveArduino)
[3] [Control system](https://ironclaw972.org/wp-content/uploads/2023/09/Control-System-Components.pdf)
[4] [ESP32 board schematics](https://www.reddit.com/r/PrintedCircuitBoard/comments/1nc4710/schematic_review_request_esp32_s3/#lightbox)
[5] [Servedrive math](https://dominik.win/blog/programming-swerve-drive/)
[6] [Serve drive math #2](https://github.com/Blue-Ignition3526/SWERVE-DRIVE)
[7] [ESP tutorial](https://deepbluembedded.com/esp32-wifi-library-examples-tutorial-arduino/)