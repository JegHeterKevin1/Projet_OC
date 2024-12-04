# Connected Garden Project

This project was developed as part of our 4th year of engineering studies in **electronics and embedded systems**. The project was carried out by a team of two members:  [@JegHeterKevin1](https://github.com/JegHeterKevin1).  and [@Joliaus](https://github.com/Joliaus). The system leverages the **NRF52 microcontroller** and various sensors to create a **connected garden** solution with PWM control, environmental sensing, and mist generation.

## Project Overview

The goal of this project was to design and implement a connected garden system using the NRF52 microcontroller. The project features environmental monitoring, GPIO control for peripheral devices like a brumisateur (mist maker), and light intensity measurement using the VEML6030 sensor.

### Key Features

- **PWM and GPIO Control**: Controls peripherals such as a brumisateur (mist maker).
- **I2C Communication**: Interfaces with sensors like the VEML6030 for light intensity measurement.
- **Modular Sensor Integration**: Designed for additional sensors for enhanced monitoring capabilities.
- **Wireless Connectivity**: NRF52 provides Bluetooth Low Energy (BLE) communication for remote control and monitoring (if implemented).

## Components Used

- **Microcontroller**: NRF52
- **Sensors**:
  - VEML6030 (Light intensity measurement over I2C)
  - Additional sensors (optional based on project configuration)
- **Brumisateur (Mist Maker)**: Controlled via GPIO for on/off states

## Getting Started

### Prerequisites

- **Development Environment**: Nordic Semiconductor's **nRF Connect SDK** and tools.
- **Hardware**: NRF52 development kit, VEML6030 sensor, brumisateur, and necessary power supply.
- **Dependencies**: Ensure the correct libraries for PWM, I2C, and GPIO control are installed.

### Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/JegHeterKevin1/Projet_OC.git
   cd Projet_OC
   ```

2. Build and flash the code onto the NRF52 microcontroller using Nordic's development tools.

### Usage

- **Brumisateur Control**: The system includes functions to turn the mist maker on and off via GPIO.
- **Light Sensing**: Light intensity data is acquired using the VEML6030 sensor over I2C communication.
- **PWM Control**: Implemented for additional control functionalities.

## Contributions

- ** [@Joliaus](https://github.com/Joliaus).**: Focused on hardware integration, sensor configuration, and system design.
- ** [@JegHeterKevin](https://github.com/JegHeterKevin1).**: Implemented software development for GPIO, I2C communication, PWM control, and project documentation.

## Future Work

- Expand the system with more environmental sensors.
- Add wireless connectivity for remote monitoring and control.
- Optimize power consumption for longer operation.

## Access to PCB Design and Additional Documents

The PCB design files and other related documents for this project are available **free of charge** upon request.  

To request access, please contact me at:  

📧 **Kévin Pottier**    
GitHub: [https://github.com/JegHeterKevin1](https://github.com/JegHeterKevin1)

📧 **Johann Raineteau**

GitHub: [https://github.com/Joliaus](https://github.com/Joliaus)

## License

This project is licensed under a **permissive license**. You are free to use, modify, distribute, and build upon this project in any way you see fit, for any purpose.
