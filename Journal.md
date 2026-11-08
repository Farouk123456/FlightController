Day 1:

Woke up bright and early lets get to work i need to do this in the time span in 2 days
I think i want to pivot to the esp32-c3 because its more familiar to me and it has wifi ble support and its cheaper

i dont think i can implement an esc in my board ill have to get some 
im also going to switch the IMU ic for a more familiar and simpler one
the esp32-c3 doesnt have enough pins i need to switsch it with the s3

This is propably the densest pcb i've evcer made

OK im done with the weekend party and i can finish what i started today
tomorrow its software time

Day 2:

Bad start im trying to import my pcb model int fusion 360 to make a sketch that will definitely fit but its hanging instead of being turned to a body because it has too many triangles

Wow this took much longer than i expected i had to dig theough the internet to find the dimentsions of the rotor and servos a want to use but in the end i have something that'll hopefully work and not immediately break

Now i need to Code:

4 -> SRV 1
5 -> SRV 2
6 -> BLDC 1
7 -> BLDC 2
15 -> BMPINT
16 -> IMUINT1
17 -> IMUINT2
18 -> IMUCS
8 -> BMPSDI
9 -> BMPSCK
10 -> SDCS
11 -> SD/IMU MOSI
12 -> SD/IMU SCK
13 -> SD/IMU MISO

Cant really use beta flight as id need a very expensive imu to meet the requirements
I'll just try to implement a naive approach and then maybe try to use acctual flight controller firmware later

Progrtamm Flow:

Boot

Setup Webserver
Capture Aircraft state values 
Write Log to SD Card


Loop

Check altitude, and orientation

4 PIDS:
    -> Altitude
        -> BLDC
    -> Orientation X
        -> BLDC
    -> Orientation Y
        -> Servo
    -> Orientation Z
        -> Servo

Found very similar code so i adopted it instead of what i was doing since i dont really know a lot about coding such a projects