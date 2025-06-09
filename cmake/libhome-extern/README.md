# Homemod_extern

## Hal component for linuxcnc homing.
Homemod_extern.so is a linuxcnc hal component like homemod.so.
This component has a few edits to make it able to be used with servo drives using internal homing,
like home on index z pulse.

## Imporant Hal pins
hal_bit_pin, type IN
- `home_extern`                      : Enable external homing.

## Features

- Home extern aborts the latch_vel search where it looks for home switch release. homing.c, line 1097.
- Home extern aborts moving checks when at latch_vel search for home switch release. homing.c, line 1132.
- Home extern aborts moving checks when searching for index pulse. homing.c, line 1219. line 1242.
- Home extern sets the `joint->pos_cmd` and the `joint->free_tp.curr_pos` to servo `joint->pos_fb`, homing.c, line 1245.

## Tests

Home sequence test:
1. Home on machine hard stop with torque control, given a home torque value.
2. Move back to index z pulse position.

Ini file:
```bash
~/linuxcnc/cmake/configs/axis_delta
```
Where joint[0] is the joint connected to the Delta servo drive.

Tested on a Sponsored single Delta ASDA B3 servo drive. 

## Ini settings

```bash
[EMCMOT]
HOMEMOD = homemod_extern

[JOINT_0]
TYPE = LINEAR
HOME = 0
MAX_VELOCITY = 200
MAX_ACCELERATION = 1200
BACKLASH = 0.000
MIN_LIMIT = -100000
MAX_LIMIT = 100000
FERROR = 1000
MIN_FERROR = 1000
HOME_OFFSET = 0.0
HOME_SEARCH_VEL = 2
HOME_LATCH_VEL = -1
HOME_USE_INDEX = YES
HOME_IGNORE_LIMITS = YES
HOME_SEQUENCE = 1
HOME_IS_SHARED = 1
HOME_INDEX_NO_ENCODER_RESET = NO
```

## Problems
Joint following errors can be avoided by increasing MIN_FERROR & FERROR values.

## Todo
Figur out how following errors actually work.

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
