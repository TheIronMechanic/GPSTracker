# GPSTracker
A GPS tracker using the ESP32 and an Adafruit Fona 808 module utilizing the ULP core

I connected an adafruit fast vibration sensor debounced with a monostable 555 timer (1 second pulse) in order to detect if the device xou are monitoring (for example a bike) has moved using really low power. If the device has moved, the ulp receives the signal and wakes up the main cores.

To Do:
- Better docuentation
- Using ULP to activate Fona module, in order to make it check incomming sms and, if received, pulse the Ri pin to wakeup the esp32
- Utilizing both cores. Did work in a previous version, but I somehow broke it...
