## Firmware for an RC car + controller, powered with 2 ESP32c3 controllers, all written in Rust.

Used for a series of youtube videos, starting here: https://www.youtube.com/watch?v=-kRTRKL39pE

Tech stack
- Rust
- Embassy (Async runtime for embedded devices)
- Postcard (serialization protocol)
- WifiNow (Low latency transport protocol)

It's a workspace project. 'Car' is the the controller of the car, 'controller' is the controller to steer the car,
'protocol' is shared code, mostly for the transfer message format.

Car hardware:
Pretty much all 3d printed.
- Servo for steering
- BLDC drone motor + controller
- 2 cell LiPo battery
- Led head and tail lights
- 4 blinkers
- Reverse light
- Brake light

I created a separate dashboard project, to create a dashboard to add to the controller:
https://github.com/flyaruu/rc-car-dashboard

Some silly features that are still on my to-do:
- Parking sensor
- Horn
- Collision sensor to trigger alarm light
- Upload telemetry.

  


