VERSION 1.0

#################################
HATIRE HEAD TRACKER FOR OPENTRACK
#################################
 * 
This program uses the Rpi Pico microcontroller,
MPU6050 Inertial Measurement Unit, and the QMC5883L
magnetometer to estimate orientation and output HATIRE
data for OpenTrack
 * 
Customize the code as its fit to you and install the necessary dependencies
 * 
If the sensors or microcontroller above aren't being used
major code modification will be necessary, please refer to the comments below
 * 
!!!!!!!!!!!!!!!!!!!!!!!!!!!
For the orientation to work
 * 
Configure your vectors like this:
 * 
Right Hand Rule
 * 
X Y Z = Roll Pitch Yaw
X = FORWARDS
Y = RIGHTWARDS
Z = DOWN
 * 
!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * 
#################################
              USAGE
#################################
 * 
IN-GAME:
 * 
Press the button to reset the euler angles, do this once
before as you play and as you see fit
 * 
CALIBRATION:
 * 
Place the device where you intend to use it, for example your headphones
Ensure the device is square and aligned with your vectors, X or FORWARDS has to
your monitor and Z should be pointing DOWN
 * 
The device is sensitive to magnetic fields and metal, so make sure you calibrat
in the same conditions and position that you will be when playing
 * 
For these first steps ensure the device is still, DO NOT ROTATE OR MOVE IT
 * 
To calibrate the device hold the button as you connect the USB cable
 * 
The LED will flash quickly as it calibrates the gyroscope
 * 
Then slower as it calibrates the accelerometers
 * 
When the LED turns off or partially off, its time to calibrate the magnetometer
You can now move the device, spin the device around on ALL AXES
pretend to paint a sphere from the inside while you roll the device as well
 * 
The device will slowly blink the LED when the calibration is done
You can now use the device with HATIRE and OpenTrack
 * 
Configure the axes, mappings, and filter settings as you see fit
 * 
The calibration settings will be stored on the EEPROM of your microcontroller
so no new calibration has to be done every time you plug in the device
 * 
If the raw sensor data on OpenTrack seems erroneous or undersirable,
or you set up the device on new headphones or similar,
please re-do the calibration as many times as needed
 * 
LICENSE
 * 
O. Rangel Morales
(GitHub: OrlandoR4)
MIT license, all text above must be included in any redistribution