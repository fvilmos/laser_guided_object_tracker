# Laser guided object tracker


## About

Proof of concept solution for tracking objects using a simple 2d image and a 2 axis joint kinematics which project a laser light marker. Tracking is realized without modeling the 2 axis joint system or pinhole camera, neighter geometric distorsions of the camera lenses. The motivation behind was to keep as simple possible, without using any have math for this approach.

<tr>
  <th>
  <a name="tracker" href=""><img src="./images/tracker.gif" alt="400" width="400"></a>
  </th>
</tr>

## System cocenpt

The system consists from two parts:
- Motor guidance, an Arduino board [pulsivo](www.pulsivo.com)
- Software running on the PC, does the object recognition using [OpenCV](https://opencv.org), estimate the coordinates with a Kalman filter and commands the motors.

### Motor guidance system

<tr>
  <th>
  <a name="tracker" href=""><img src="./images/mc.jpg" alt="400" width="400"></a>
  </th>
</tr>

Once the ```.ino``` file is written to the arduino board, can be accessed over serial. Send in a serial terminal window the ```ls,0``` to list all the avilable commands. It sould look like this.
```
m0mtp,val - stepper 0 move to position plus (clockwhise) n steps
m0mtm,val - stepper 0 move to position minus (anti-clockwhise) n steps
m1mtp,val - stepper 1 move plus (clockwhise) n steps
m1mtn,val - stepper 1 move minus (anti-clockwhise) n steps
pon,val - port x on
poff,val - port x off
m0p,val - stepper 0, current position plus n steps
m0m,val - stepper 0, current position minus n steps
m1p,val - stepper 1, current position plus n steps
m1m,val - stepper 1, current position minus n steps
ls,val - list commands over serial
sh,[0,1] - set home posiion for motor [0,1]
gh,[0,1] - go home motor [0,1]
```

## Usage

Admin righs will be needed since the sotware get access to the serial port. 
From Command line: 
```
sudo python3 laser_guidance_ot.py -cam 0 -ser /dev/ttyUSB0

usage: laser_guidance_ot.py [-h] -cam camID -ser serial

optional arguments:
  -h, --help   show this help message and exit
  -cam camID   Camera ID, default 0
  -ser serial  Serial port name

```

### Future work

Change color tracker to object detection using machine learning

## Resources

[Python OpenCV tutorial](https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html#converting-colorspaces).

[Arduino CheepStepper librarry](https://github.com/tyhenry/CheapStepper) - used to control the stepper motors

/Enjoy.
