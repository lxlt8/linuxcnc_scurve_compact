# S-Curve Trajectory Generator Library

![x](https://img.shields.io/badge/language-C-blue) 

## Overview
This C library implements a **constant jerk S-curve trajectory generator** for smooth motion control in robotics, <br>
CNC machines, and other motion systems. The library provides functions to compute position, velocity, acceleration, <br>
and jerk profiles for S-curve transitions, ensuring smooth and impact-free motion. <br>

The S-curve trajectory is divided into **concave** and **convex** periods, <br> 
with optional **linear acceleration** phases for large speed changes. <br>

The library supports customizable parameters such as maximum jerk, <br>
acceleration, and target velocity, making it suitable for a wide range of motion control applications. <br>

---

## Acknowledgments

This library is based on the mathematical formulation of constant jerk S-curve trajectories, <br>
as described in the work: <br>

"Jerk Limited Velocity Profile Generation for High Speed Industrial Robot Trajectories" <br>
by Soon Yong Jeong, Yun Jong Choi, PooGyeon Park, and Seung Gap Choi. <br>

DOC: [Abstract_0](https://codeberg.org/skynet/scurve/src/branch/master/src/docs/science_paper_0.pdf) <br>
DOC: [Abstract_1](https://codeberg.org/skynet/scurve/src/branch/master/src/docs/science_paper_1.pdf) <br>

---

<div align="center">
  <img src="https://codeberg.org/skynet/scurve/raw/branch/master/src/pics/scurve_plot_small.png" alt="S-Curve Plot">
</div>

---

## Features

- **Ideal S-Curve Profiles**: Generate smooth S-curve trajectories with constant jerk for both acceleration and deceleration phases. <br>
- **Concave and Convex Transitions**: Compute position, velocity, and acceleration for concave (increasing acceleration) and convex (decreasing acceleration) periods. <br>
- **Linear Acceleration Phase**: Optional linear acceleration phase for large speed changes, reducing transition time. <br>
- **Customizable Parameters**: Set maximum jerk (`jm`), maximum acceleration (`as`), initial velocity (`vo`), and target velocity (`ve`). <br>
- **Motion Planning**: Predict time and distance required for transitions, enabling efficient path planning. <br>

- **Waypoint Planning**: Create trajectory's with multiple waypoints.  <br>
- **Endvel**: Trajectory's can use endvelocity.  <br>
- **Realtime**: Use this library in a realtime enviroment.  <br>

---
 
## Performance vs Ruckig and Reflexxes 

To calculate one scurve cycle it takes ~ 0.005 - 0.031 ms <br>
To calculate one scurve cycle it takes ~ 5 - 31 us - micro seconds. <br>

In relation the the Ruckig motion lib, LINK: [Ruckig motion library](https://github.com/pantor/ruckig) <br>

Benchmark: <br>
We find that Ruckig is more than twice as fast as Reflexxes Type IV for state-to-state motions <br> 
and well-suited for control cycles as low as 250 microseconds. <br>

The performance of this **scurve** lib is much faster. Around 8 times faster. <br>
It can also be stated that this library uses no non-free agreements to generate a trajectory using waypoints. <br>

---

## Extensive code example's running realtime complex trajectory's.

LINK: [linuxcnc motion planner](https://codeberg.org/skynet/linuxcnc_scurve_compact/src/branch/master/cmake/libplanner)

---

## Building the Project, library and test app.

1. Clone repository:

   ```bash
   git clone https://codeberg.org/skynet/scurve.git
   ```

2. Create a build directory:

   ```bash
   mkdir build
   cd build
   ```

3. Configure the project using CMake:

   ```bash
   cmake ..
   ```

4. Build the project:

   ```bash
   make
   ```

5. Install the libclothoid_3d.so library:

   ```bash
   sudo make install
   ```

---

## Code example

<small>

```bash
#include "scurve.h"
#include "stdio.h"

int main() {
    struct scurve_data scd;
    enum scurve_return_code ret;

    // Initialize motion constraints
    double max_jerk         = 100;
    double max_acceleration = 20;
    double max_velocity      = 120;
    double cycletime        = 0.001;

    // Set target state
    double endpos           = 100;
    double endvel           = 0;
    double endacc           = 0;
    int pausing             = 0;

    // Initialize the S-curve generator
    scurve_init(&scd, 
                max_jerk, 
                max_acceleration, 
                max_velocity, 
                cycletime);

    // Update the motion profile
    double time = 0;
    while (1) {
        scurve_set_target_state(&scd, 
                                endvel, 
                                endacc, 
                                endpos, 
                                pausing);
        ret = scurve_update(&scd);

        printf("Time: %f, 
                Position: %f, 
                Velocity: %f, 
                Acceleration: %f\n",
                time, 
                scd.curpos, 
                scd.curvel, 
                scd.curacc);

        time += cycletime;

        if (ret == RETURN_FINISHED) break;
    }

    return 0;
}
```

</small> 

---

## Usage
After building the project, you can run the executable to test the scurve.  <br>

`cd build/test` <br>
`./test` <br>

The test plots a scurve cycle as shown on the picture above.  <br>

---

## Attribution
This implementation is based on the work of : <br>
**Soon Yong Jeong, Yun Jong Choi, PooGyeon Park, Seung Gap Choi**. <br> 
    
Please refer to the original paper for a detailed explanation of the methodology and results. <br>

---

## Historal events
Over the years this scurve library is evolved in what it is today.  <br> 
C programming language is used as primary coding language, as it is easely supported in realtime enviroments. <br> 

Historical scurve implementation attempts for info: <br> 

LINK: [scurve_construct](https://github.com/grotius-cnc/scurve_construct) <br> 
LINK: [scurve_pro](https://github.com/grotius-cnc/scurve-pro) <br> 

---

## License
This project is open-source and available under the GPL2 License. <br>

---

## Author
Michel Wijnja, alias Grotius, Skynet. <br>
Copyright (c) 2024 All rights reserved. <br>

---

## Author contact
michelwijnja@gmail.com

---

## Supporting the Project

Developing and maintaining this codebase requires a significant amount of time, effort, and dedication. <br> 

If you find this project useful and would like to support it, <br> 
please consider making a donation <br> 
to the following non-profit organization that helps donkeys. <br> 

Your contribution will not only support this project but also make a meaningful <br> 
difference for donkeys in need. Thank you for your generosity! <br> 

<a href="https://www.oscarsplace.org/welcome" target="_blank">
  <img src="https://img.shields.io/badge/Donate-Now-blue" alt="Donate Now">
</a>
