# Fall-Detection-System-using-GSM-on-STM32

## Introduction

This project is designed to provide safety to aged people or those with need of monitoring by detecting a fall and triggering an emergency SOS message via GSM. The system was originally designed to keep a check on patients on wheelchairs but can be extended to general users by modifying the threshold of triggering a message. It is built using the STM32G030F6P6 microcontroller and an IMU sensor to monitor roll and pitch angles which define the threshold. A GSM module sends an SOS when this is triggered. 

---

## Components Used

- **STM32G030F6P6** Microcontroller
- **IMU Sensor** (for Euler angle measurement)
- **SIM800L GSM Module**
- **DC-DC Buck Converter** (to provide 3.4V–4.5V to SIM800L)
  
**Schematic Diagram**

  ![image](https://github.com/user-attachments/assets/2c96e0b2-9a2f-4403-b073-a4e1da282a51)
---

## System Logic

1. The IMU continuously tracks **Euler angles** (Roll and Pitch).
2. If Roll or Pitch of IMU exceeds **±60°**, the microcontroller triggers an emergency routine.
3. An **SOS message** is sent via GSM to a predefined contact number.
4. The GSM module is configured via **AT commands** and communicates over serial at **115200 bps**.

**Block Diagram**
![image](https://github.com/user-attachments/assets/bf0de497-9a8a-4fc9-bc3c-acbb23460519)




---

## GSM Module Configuration

| Command              | Purpose                          |
|----------------------|----------------------------------|
| `AT`                 | Attention command                |
| `AT+CPIN`            | Check if SIM is ready            |
| `AT+CMGF=1`          | Set SMS format to text mode      |
| `AT+CMGS="+918171612523"` | Send SMS to specified number |
| `HELP! SOS!  Fallen` | SMS Body               |

---

## Specifications

- **SIM800L Voltage**: 3.4V – 4.5V
- **Power Supply**: A **buck converter** is used to step down voltage to GSM-safe levels.
- **Baud Rate**: Communication with the GSM module is configured at **115200 bps** via the serial monitor.

---

## Setup Summary

- The IMU provides orientation data.
- STM32 checks for threshold breach.
- On breach, SIM800L sends an emergency message.
- Ensure proper voltage regulation for GSM.
---
## Result
![image](https://github.com/user-attachments/assets/90a77364-74d3-4256-b042-8c7b6ff856ac)

