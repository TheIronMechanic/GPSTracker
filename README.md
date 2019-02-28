# GPSTracker
A GPS tracker using the ESP32 and an Adafruit Fona 808 module utilizing the ULP core

I connected an adafruit fast vibration sensor debounced with a monostable 555 timer (1 second pulse) in order to detect if the device xou are monitoring (for example a bike) has moved using really low power. If the device has moved, the ulp receives the signal and wakes up the main cores. First, they will check if a BLE devive advertising with a specific name is within range (I use the name because iPhones are randomizing the mac addresses. You can use an app like NRF connect to create an avertiser with a certain name. If the owner is out of range, they will check if the current GPS coordinates are farther away from the initial GPS coordinates then a threshhold. If so, they send the current GPS and GSM location (in the form of a handy google maps link), per SMS to your phone. It will work with a prepaid sim!!!

To Do:
- Better docuentation
- Using ULP to activate Fona module, in order to make it check incomming sms and, if received, pulse the Ri pin to wakeup the esp32
- Utilizing both cores. Did work in a previous version, but I somehow broke it...
- OTA updates
- Bargraph e-ink display for battery level 
- add BLE device distance detector

Keep in mind, I'm not a professional software developper, I learned everythin by myself. 
Suggestions are allways welcome.
