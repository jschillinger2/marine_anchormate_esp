# Marine Anchor Controller

## Overview
The Marine Anchormate Controller is an experimental project designed to interface with Signal K, controlling a FireBeetle board to operate relays for an anchor windlass system. Additionally, it captures rotation counts from a windlass rotation sensor (e.g., a reed switch) and pushes this data back to Signal K. This project aims to provide a bridge between marine data networks and physical anchor control systems, enhancing automation and monitoring capabilities.
<br>
This is the hardware unit to work with the project <a href="https://github.com/jschillinger2/marine_anchormate">here</a> 

## Features
- **Signal K Listener**: Integrates with the Signal K network to receive commands and data.
- **Windlass Control**: Manipulates pins on a FireBeetle board to control relays for the anchor windlass, enabling automatic operation.
- **Rotation Count Monitoring**: Listens to a rotation sensor attached to the windlass, counting rotations and feeding this information back to Signal K for accurate monitoring.

## Requirements
- FireBeetle Board (ESP32)
- Signal K Server
- Anchor Windlass System with rotation Sensor / reed switch

## Installation

### Hardware Setup
1. Connect the FireBeetle board to the anchor windlass system via relays.
2. Attach the rotation sensor to the windlass, ensuring accurate detection of rotations.

### Software Setup
1. Navigate to the project directory:
   ```bash
   cd marine_anchormate
   ```
2. Update the contstants in the cpp file, eg pins and wifi credentials.  
3. Upload `main.cpp` to your FireBeetle board using your preferred IDE or the Arduino CLI.

## Usage
After installation, the system listens for Signal K messages to control the anchor windlass and sends rotation count data back to Signal K. Ensure your Signal K server is correctly configured to communicate with the controller.

## Disclaimer
This software is experimental and is provided "as is", without warranty of any kind. It's intended for testing and development purposes. The author takes no responsibility for any damage or loss caused by the use of this software. Users are advised to proceed with caution and at their own risk.

## Contributing
Contributions to the Marine Anchormate Controller project are welcome. Please submit issues and pull requests through GitHub.

## License
This project is licensed under the MIT License - see the LICENSE file for details.
