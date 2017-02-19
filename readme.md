Quadcopter
==========

The goal of this project was to learn about Arduino, Raspberry Pi and Robotics by building and programming a Quadcopter, implementing as much as possible from scratch without using kits or fully prepared quadcopter control libraries.

Status of the project is currently unfinished, the current code can fully control the quadcopter and it did achieve a few tethered flight, but a lot more work is required in tuning the PID controls and optimising the code before it could be fully controlled.

The aim was for an automated system to control the quad using set routines from the command line without need for a RC unit, however it quickly became clear that until things worked very smoothly, the command line was too clunky to control or stop engines quickly.

Photos of the build and a summary of the code is below

Arduino
----------
The majority of the code sits in here. This includes
	
* CalibRXTX.ino - Initial Calibration of the Remote Control
* CalibSensors.ino - Sample the sensors before starting to calibarate them
* GetAlt.ino - Get altitude using an ultrasonic sensor
* ReadRXTX.ino - Read inputs from the Remote Control
* SetMotorSpeed.ino - Control the speed of the motors
* SetupParamaters.ino
* quadcopter.ino - Main script including PID controller and Kalhman Filters

Processing
----------
* sketch_3d_Quadcopter.pde - A processing script to test the sensorts and simulate the quadcopter's attitude

Video:

[![Quadcopter Processing Script](http://img.youtube.com/vi/7WJHeJ8LGKk/0.jpg)](http://www.youtube.com/watch?v=7WJHeJ8LGKk)

HTML
----------
index.html - The intended UI for the quadcopter

Python
----------
script_serial.py - A script to interface between the raspberry pi command line and the arduino to send commands
script_socket.py - A websocket script to interface between the web UI and the arduino


Photos
----------

Some photos of the build

<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image_1.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image_2.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image_3.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image_4.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image_5.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image_6.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image_7.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/image.jpeg" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1033.JPG" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1034.JPG" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1038.JPG" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1039.JPG" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1040.JPG" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1043.JPG" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1045.JPG" />
<img width = 300 height = 300 src="https://github.com/GerHarte/Quadcopter/raw/master/Photos/IMG_1047.JPG" />

Some videos
----------

[![PID Tuning](http://img.youtube.com/vi/saGkzMU6aV0/0.jpg)](http://www.youtube.com/watch?v=saGkzMU6aV0)
[![PID Tuning2](http://img.youtube.com/vi/rQ5uXYcNxQ0/0.jpg)](http://www.youtube.com/watch?v=rQ5uXYcNxQ0)
