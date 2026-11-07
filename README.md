# Tandem Flight Controller 

![pcb](./imgs/pcb.png)

![3d model](./imgs/model.png)

## What is it ?
This is an esp32-s3 based flight controller built for a tandem aircraft with 2 bldc motors and 2 servos that controll the rotors angle
And stablizes the assembly using an IMU and a Barometer

## Why does it exist ?
I was motivated to make this to challenge myself to see how dense i could make a PCB also im super intersted in PID and TVC systems

## How does it Work ?
When the board powers on it captures its current state as the baseline, then it rises 1.5m off the ground stays there whilst running a webserver REST API that tells the controller where to go next. On idle it just tries to stay stable in its orientation and position

## How would I use it ?
First you'd flash the firmware to the esp32 using an ftdi programmer and maybe calibrate the imu then you'd set it programmatically in flight mode reflash the firmwware and put it on the ground, then power it up> it will boot fly ~1.5m then respond to the clients requests and move accordingly.

## Bill of Materials
