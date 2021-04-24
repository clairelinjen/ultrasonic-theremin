# ultrasonic-theremin
A theremin built using an ESP32-A1S audio development and two ultrasonic sensors. One sensor controls the pitch of the instrument while another controls volume.

### Materials
- ESP32-A1S + power supply
- 2 ultrasonic sensors
- aux cord and external speaker, or headphones

### Setup
Connect the first ultrasonic sensor by connecting the VCC pin to a 3.3V pin, connecting the 'Trig' pin to GPIO pin 22, connecting the 'Echo' pin to GPIO pin 21, and connecting ground to ground on the ESP32-A1S.

Connect the first ultrasonic sensor by connecting the VCC pin to the second 3.3V pin, connecting the 'Trig' pin to GPIO pin 19, connecting the 'Echo' pin to GPIO pin 23, and connecting ground to ground on the ESP32-A1S.

Connect the ESP32-A1S to a computer and upload theremin.ino to the device.

Leave the device connected (or connect to different power supply), and connect the device to a speaker or headphones with the headphone jack.

View the theremin in action [here](https://youtu.be/sgeZhqS_tHc).

