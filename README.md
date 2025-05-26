# 4-Wheel mobile Robot for pick up and place operations
UMD Project for ENPM661 : Autonomous Robotics- Spring 2025, special thanks to professor 

![full build](https://github.com/user-attachments/assets/49b294b8-bce4-47a4-84d3-bde0f857e8a1)

Check out the full video story: https://youtu.be/_-neVhMx8bU?si=svva_gK9aAoRBGkA

##  Bill of materials 
| Item | Qty |
|------|-----|
| Raspberry Pi 3 - Model B+ or Model 4 | 1 |
| Raspberry Pi Camera Board v2 - 8 Megapixels | 1 |
| 16GB Card with NOOBS 3.1 for Raspberry Pi Computers including 4 | 1 |
| Half-sized breadboard | 2 |
| Ultrasonic distance sensor | 2 |
| 5V 2.5A Switching Power Supply with 20AWG MicroUSB Cable | 1 |
| 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055 | 1 |
| Micro B USB to USB C Adapter | 1 |
| USB cable - USB A to Micro-B 3 foot long | 1 |
| 4WD Arduino compatible robot platform w/ encoders | 1 |
| Omars Battery Pack Power Bank 10000mAh USB C Battery Bank | 1 |
| Pack of 12 inch 40 pin male to female dupont wire, etc. | 1 |
| L298N motor drive controller board module, dual H-bridge, 2-Pack | 1 |
| Rechargeable battery charger | 1 |
| Rechargeable batteries | 1 |
| Sterilite 17631706 27 Quart/26 Liter ClearView Latch Box, Clear with Sweet Plum Latches, 6-Pack | 1 |
| Hitec 31311S HS-311 Servo Standard Universal | 1 |
| Arduino Nano | 1 |
| GANA Micro HDMI to HDMI Adapter Cable, Micro HDMI to HDMI Cable (Male to Female) (*Only need 1 per kit) | 0.5 |
| Antrader 1-Feet Mini USB 2.0 Cable A-Male to Mini-B 30cm Length 12pcs/lot (*Only need 1 per kit) | 0.08 |
| Hilitchi 120pcs M2 Male Female Nylon Hex Spacer | 1 |
| Steel machine screw hex nut, #4-40 (*Only need 4 per kit) | 0.04 |
| #4-40 x 1/2" stainless pan head phillips machine screw (*Only need 4 per kit) | 0.04 |
| Standoff, male to female, 3/16" OD hex, 4-40 x 1" L (*Only need 2 per kit) | 0.2 |
| 1000 Ohm resistors | 5 |

## Introduction 

This project was built by using raspberry pi 4 , arduino UNO and a variety of electronic modules as a raspberry pi camera V2 and the 9 axis BNO055.

As operating system raspberry pi OS system was used. For more information how to install it into a SD Card 

[https://www.raspberrypi.com/software/operating-systems/](https://www.raspberrypi.com/software/)

The objective of the mobile robot is able to idnetify a series of Red, Green and blue blocks and place them in a designated construction zone autonomously.

![explanation image](https://github.com/user-attachments/assets/e121adf3-4765-4a3c-a9c9-9a0290bc4769)

## Working with the code

### Some important dependencies to confirm installation

The language version is python 3.9.2. To install these dependecies use pip ver. 25 

| Package             | Version    |
|---------------------|------------|
| `opencv-python`     | 0.29.1     |
| `picamera2`         | 1.9.1      |
| `RPi.GPIO`          | 0.7.0      |
| `numpy`             | 1.26.4     |
| `matplotlib`        | 3.9.4      |
| `numpy`             | 1.26.3     |
            

The **modules** folder contains suggested names for every core functionality block intented to excecute the main script **running_main.py**.
To execute it it's important to have the same main hardware provided as it runs inside a raspeberry pi.

By placing the robot in a safe location
Turn on the switch for the motors
Verify that the arduino UNO is connected to the serial port of the raspberry pi4 
and locate the folder , tun in terminal:

```sh
python3 running_main.py 
```

## Future Improvements
- Recognition of QR Codes
- Recognition of features like orientation or other hapes
- Mapping?
- Depth Estimation?
- Application of other control laws
- RL or ML models
- Using linear accel measurement for better position tracking
