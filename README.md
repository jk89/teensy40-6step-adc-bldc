# teensy40-6step-adc-bldc
A BLDC ESC based off Teensy 4.0 using its ADC for voltage measurements periodically.

# Status quo

So the ESC is definitely not working as it should. NOISE WARNING: https://youtu.be/O-tqtvKpQ0g

The performance of the motor seems tied to the PWM frequency. High frequencies do not work, so this leads me to the following:

- The ADC trigger might not be correct.
- The ADC is out of sync with the PWM being off.
- The ADC chain takes to long to process.
- The internal logic of the teensy controller is too intensive and steps are missed.
- Where logging occurs, cli() and sei() are used to suppress interrupts, this could cause a syncronisation issue.

Contributions and suggestions are most welcome. The aim of this repository is to work towards a canonical example for the benefit of the community.

See the images directory:

	- ./images/output-with-power-circuit-off-20khz(GOOD SIGNAL).png is an example where VCC is disconnected so that the half bridges do not power the circuit. In this passive case the signal looks look, zero crossings are clear and it acts well as a positional sensor when rotated manually. Note there is some tearing of this image this is an artifact of screen capture. 
	- ./images/output-with-power-circuit-on-20khz(BAD SIGNAL - ALMOST NO MOTION).png is an example where VCC is connected to a 4s battery, in this case the zero crossing signals are full of noise.
	- ./images/output-with-power-circuit-on-1950hz(MODERATE SIGNAL - SOME CONTINUOUS MOTION).png via trial and error i discovered that by reducing the frequency to the magic number of 1950, the motor does make some motion, the signal is not very clear but some rotation is achieved (See the video above).

Unfortunately the motor fails to enforce changes of speed via the modification of the duty cycle.

# Circuit

The circuit is composed of a 3 phase inverter powered by 6 half bridges as well as IR2104 ic drivers which greatly simplifies the control design. BEMF measurements are taken by a voltage divider grid which allows for voltage measurements of the voltage on phase A, B, C phase channels, as well as a common virtual neutral. Zero crossing detection occurs when a phase signal crosses the virtual neutral signal.

You can see the circuit here: ./images/circuit.png

# Dependancies

1. jstest is what i've used to decode input from a PS4 controller. I tried pyPS4Controller but could not get it to work so built a parser for jstest output
	- sudo apt-get install joystick
2. pyserial is a library for sending command via usb to the teensy 4.0
	- (sudo [for global usage]) pip3 install -r pip.freeze 
3. SerialPlot
    -  sudo apt install build-essential qtbase5-dev libqt5serialport5-dev libqt5svg5-dev cmake mercurial
    - hg clone https://hg.sr.ht/~hyozd/serialplot
    -  cd serialplot/
    - mkdir build && cd build
    - cmake ..
 	- make
4. (Optional) Load the SerialPlot colour profile, select file menu "Load Settings" and choose this file ./SerialPlot-Profile.ini
	- Color schema in order:
  		0.  ADC delta phase A: ADC Signal Phase A subtract ADC Virtual Neutral Signal (RED)
  		1. 	ADC delta phase B: ADC Signal Phase B subtract ADC Virtual Neutral Signal (YELLOW)
  		2.  ADC delta phase C: ADC Signal Phase C subtract ADC Virtual Neutral Signal (BLACK)
  		3.  ADC Virtual Neutral Signal (BLUE)
  		4.  Zero crossing detection (+10 Rising detection, -10 Falling detection) for Phase A (RED)
  		5.  Zero crossing detection (+10 Rising detection, -10 Falling detection) for Phase B (YELLOW)
  		6.  Zero crossing detection (+10 Rising detection, -10 Falling detection) for Phase C (BLACK)
  		7.  Mechanical step counter (GREEN)
  		8.  Roll (will be used for actuator control in the future possibly) (PURPLE, more red than blue)
  		9.  Pitch (will be used for actuator control in the future possibly) (PURPLE more blue than red)
  		10. Duty Target (the thrust setting 8 bit) (PINK same level of red and blue but with some green)
  		11. Calculate RPM (updated every mechanical cycle). Will be used to help normalise duty to rpm for unique motors in the future (calibration) (BLACK)

# How to run

1. Start sending commands from ps4 controller to remote: python3 remoteCommand.py
2. Starting the teensy command server python3 commandServer.py
3. Open SerialPlot select ttyACM0 and click open.
4. Press triangle on the PS4 controller to startup.
5. Press square on the PS4 controller to stop.
6. Squeeze right trigger to control thrust setting (WARNING as this is not working currently you could burn out hardware by using the throttle )

# Instructions without a PS4 controller.

You can use serial plot to send commands to the controller directly. 

- Pressing triangle (start) corresponds to the command  b'\x14\x00\x00\x00\x00\x00\x00\x00\x00'
- Pressing x (start) corresponds to the command  b'\x00\x00\x00\x00\x00\x00\x00\x00\x00'

The first byte of a string of 9 for a full signal is the duty cycle setting. You can use this to increase the speed. Not you must start
with a thrust byte greater than that of the start command (20 in decimal, 14 in hex), lower than this and the ESC will use this as the signal to reset and shutdown the motor.

# Pin summary

The circuit has a female 13 pin header. Here is the configuration P1-x of the power circuit to teensy pin X:

	- P1-13 (A_IN): PIN 2
	- P1-12 (A_SD): PIN 1
	- P1-11 (B_IN): PIN 7
	- P1-10 (B_SD): PIN 0
	- P1-09 (C_IN): PIN 8
	- P1-08 (C_SD): PIN 22
	- P1-07: TEENSY40 GND
	- P1-06 (Phase A voltage divider signal): PIN 14
	- P1-05 (Phase B voltage divider signal): PIN 15
	- P1-04 (Phase C voltage divider signal): PIN 16
	- P1-03 (Virtual neutral voltage divider signal): PIN 17
	- P1-02 (Virtual neutral voltage divider signal): NOT USED (could be used if one uses the analog comparator)
	- P1-01 (Virtual neutral voltage divider signal): NOT USED (could be used if one uses the analog comparator)

# FlexPWM and ADC/ADC-ETC information table:

| PHASE       | A   | B      | VN    | C     |
|-------------|-----|--------|-------|-------|
| PIN         | 14  | 15     | 17    | 16    |
| COLOUR      | RED | YELLOW | GREEN | BLACK |
| TRIG        | 0   | 1      | 2     | 3     |
| DONE        | 0   | 1      | 0     | 1     |
| HW-CH       | 1   | 2      | 3     | 4     |
| ADC-CH      | 7   | 8      | 11    | 12    |

ADC has been set to trigger when each PWM channel counter reaches trigger point 3. This should theoretically syncronise ADC measurements such that they take place when the PWM signal is off. This is to prevent reading the channel when it is set high which would cause noisy BEMF measurements. 

# Motor information:

I am using the 1000KV Brushless Motor A2212 13T for testing. Other motors would work but you would have to tinker with the MIN_DUTY to ensure startup defeats the rolling resistance of the motor. Similarly you may have to experiment with the startup linear chirp to get it to start properly.

# WARNING
	This is an experimental project. I have destroyed a great deal of hardware (motors, transistors, ICs) and almost caused a fire once. Please be careful when using this code. ABSOLUTELY NO WARRANTY. If you do not agree to this then please do not download this software as this would violate the license which this code was distributed with, see ./LICENSE for details.

	Moreover this project lacks sensible precautions with the ESC to halt the motor when its in a faulty state (aka something is in the way of the motor spinning). I have cuts in my finger to prove it.

# Credits

This project was inspired greatly by the following arduino project: https://simple-circuit.com/arduino-sensorless-bldc-motor-controller-esc/
the circuit is largely the same but for some better available transistors and a modified BEMF voltage dividing grid to offer greater protection for teensy40(which is only 3v capable) when using larger motors.

https://www.youtube.com/watch?v=KxncxjNpMFI&

I have had great success with this project but wish to move to an ARM platform as it offers great features and power, hence the inception of this project.

# Improvements

Beyond the obvious of getting the current state working I have the following ideas for the future:

- In future I may modify the circuit to move to a FOC paradigm employing shunts to allow for current rather than voltage measurements but I cannot guarantee I will have the time to do this.

- The circuit is begging for some thermal and overdrive current protection 

- The ESC needs better fault protection logic, one should use the elapsed time between cycles to enforce some protection that if elapsed time deviates significantly from sensible values it could indicate that the motor has stalled or is obstructed, in this case current can surge and destroy hardware. Also this could maybe prevent it acting like a Dremel and risking your fingers with a higher kv motor.

# Code quality

This is just for demonstration purposes only, with a fully working example I shall refactor everything and remove the monolithic monstrosities. I did make a strong attempt to ensure that the teensy code is self described however and left details of register manipulation where I believed them to be tricky for unfamiliar readers.