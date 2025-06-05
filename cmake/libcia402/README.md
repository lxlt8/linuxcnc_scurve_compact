# Libcia402

## Hal component for ethercat servo drives using the Cia402 protocoll.

Feaures:
- Position mode.
- Home mode.
- Servo drive home sequence.

## Imporant Hal pins
hal_bit_pin, type IN
- `enable`                      : Enable servo drive. Servo drive should be ready for a position command.
- `home`                        : Starts internal home program of servo drive. Takes the `var-home-program` nr as home program input. For manual control.
- `index-enable-clear`          : Resets IO pin `index-enable` to zero.

hal_bit_pin, type OUT
- `stat-torque-home-stop`       : Signals high when machine is at hard stop and torque value is >= `var-max-torque-home-stop`.

hal_s32_pin, type IN
- `var-max-torque-home-stop`    : Set the max motor torque for homing to machine hard stop limit search.
- `var-home-program`            : See the pdf file page 802 for available home programs. 0=Default, 33 & 34=Home to index pulse.

hal_u32_pin, type IN
- `var-opmode`                  : Servo's opmode, 0=None, 8=Position, 9=Velocity, 6=Home
- `var-max-torque`              : Set the max motor torque at runtime.
- `var-max-torque-homing`       : Set the max motor torque when homing.
- `var-torque-release-delay-ms` : When `stat-torque-home-stop` is true, this value delays the torque-sensor from switching to off state.

hal_float_pin, type IN
- `pos-cmd`                     : Servo position command. Servo will run to this position.
hal_float_pin, type OUT
- `pos-fb`                      : Servo encoder position feedback.  

hal_bit_pin, type: IN-OUT
- `index-enable`                : Start homing to index z pulse. Like the `home` pin, but different hal pin type. Connected to linuxcnc homecia402.

Halfile example : ~/linuxcnc/cmake/libcia402/runtest
Halfile example : ~/linuxcnc/cmake/configs/axis_delta

## Tests

Tested on a Sponsored single Delta ASDA B3 servo drive. 

## Language

C code.

## License

This project is open-source and available under the **GPL2 License**.

---

## Author

**Michel Wijnja** (alias Grotius, Skynet)  
Copyright (c) 2024. All rights reserved.

---

## Author Contact

For inquiries, please contact:  
📧 [michelwijnja@gmail.com](mailto:michelwijnja@gmail.com)

---

## Supporting the Project

Developing and maintaining this codebase requires significant time, effort, and dedication. If you find this project useful, please consider making a donation to the following non-profit organization that helps donkeys. Your contribution will support both this project and a meaningful cause.

<a href="https://www.oscarsplace.org/welcome" target="_blank">
  <img src="https://img.shields.io/badge/Donate-Now-blue" alt="Donate Now">
</a>

---
