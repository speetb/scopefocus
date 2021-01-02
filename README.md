# Scopefocus

To reduce the amount of wires and additional modules on the telescope, this focuser controller is implemented on the back of a stepper motor and requires only 1 connection (USB)
At the same time, it will allow the use of more power and the connection to a temperature sensor and a RC servo (for future work).
The stepper driver is powered from either the USB or from an 5.5 mm barrel connector (up to 24 V) selectable by a jumper.

Currently works:
- Stepper motor control
- Reading temperature
- Communication with EKOS using Moonlite protocol https://indilib.org/media/kunena/attachments/1/HighResSteppermotor107.pdf
- IR command recieving (NEC protocol) and setting speed & position

Using hardware:
- Seeeduino XIAO https://wiki.seeedstudio.com/Seeeduino-XIAO/
- Fysetc TMC2130 v1.1 stepper motor driver https://wiki.fysetc.com/TMC2130
- TMP36 temperature sensor connected via a three-pin connector https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf
- VS1838 IR reciever
- Nema 17 stepper motor

Using libraries:
- IRremote https://github.com/Arduino-IRremote/Arduino-IRremote
- Accelstepper https://www.airspayce.com/mikem/arduino/AccelStepper/index.html
- TMC2130 https://github.com/teemuatlut/TMC2130Stepper

TODO:
- Implement faster IR recieve library. Maybe: https://github.com/NicoHood/IRLremote
- Make temp reading fast (fast analog read) and non-blocking (example: BlinkWithOutDelay).
- Reduce/ eliminate floating point math.
- Create own fast stepper library using:
  - Direct port manipulation https://forum.seeedstudio.com/t/direct-port-access-in-arduino-ide/253807/6
  - 50 % duty cycle step pulsewidth as needed for TMC2130 mycroplyer https://download.mikroe.com/documents/datasheets/TMC220x_TMC222x_Datasheet.pdf#page=59
  - Microstepping to low value for fast moves and in the last phase of movement set the 'microstepping phase angle' using more micro-stepping.
- Eliminate backlash by always moving from one direction as the final move.
 
Not for now, but something to think about:
- Use the servo output and enable 'mirror lock' as a functionality (bool MirrorLock = true/ false) for Schmidt-Cassegrain telescopes.

Code originally adopted from https://github.com/orlyandico/arduino-focuser-moonlite
