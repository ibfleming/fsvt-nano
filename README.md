![Sample Image](https://github.com/ibfleming/fsvt-nano/blob/master/imgs/sample.png)

# Free Stream Velocity Team - Arduino Nano Programs

**Version:** 3.4

**Author:** Ian Fleming

**Senior Capstone Project - University of Idaho 2023**

Welcome to the repository containing the software uploaded to the Arduino Nano microcontrollers for our Senior Design Capstone Project at the University of Idaho in 2023. This project, led by the Free Stream Velocity Team, aimed to develop a robust and efficient solution for real-time data transmission and control using Arduino Nano microcontrollers.

## Overview

The project utilizes two Arduino Nano microcontrollers: a primary (master) and a secondary (slave). Each microcontroller is equipped with an HC-12 wireless transceiver and a TDS probe. Additionally, the primary Nano features an HM-10 Bluetooth module. The software's primary function is to acquire Total Dissolved Solids (TDS) readings at specified intervals and transmit them wirelessly to an [Android application](https://github.com/ibfleming/fsvt-app) for processing. Furthermore, the microcontrollers facilitate seamless communication of user commands and inputs between each other.

## Key Features

- **Battery Indicator:** Provides interval monitoring of battery levels to ensure uninterrupted operation.
- **Responsive TDS Data:** Ensures accurate and timely transmission of TDS readings from both devices.
- **Bluetooth Integration:** Facilitates seamless integration with Android applications via Bluetooth connectivity.
- **Low Latency:** Optimized for minimal latency to ensure real-time data transmission.
- **Error Handling:** Implements robust error validation and handling mechanisms to address various edge cases.

## Usage

To clone the repository to your local environment, use the following command:

```bash
git clone https://github.com/ibfleming/fsvt-nano.git
```

Please note that the usage and installation of the software are restricted to authorized clients who have access to the Capstone Design documentation.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
