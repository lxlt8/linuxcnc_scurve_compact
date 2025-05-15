# Clothoid 3D

![x](https://img.shields.io/badge/language-C_CPP-blue) 

**"Space corner smoothing of CNC machine tools through developing 3D general clothoid"** <br> 
by **Qun-Bao Xiao, Min Wan, Yang Liu, Xue-Bin Qin, and Wei-Hong Zhang**. <br> 
Published in *Robotics and Computer Integrated Manufacturing*, Volume 64, 2020. <br> 
DOI: [10.1016/j.rcim.2020.101949](https://doi.org/10.1016/j.rcim.2020.101949) <br> 
DOC: [Abstract](https://codeberg.org/skynet/libclothoid3d/src/branch/master/src/docs/clothoid_3d_1.pdf) <br> 

---

<div align="center">
  <img src="https://codeberg.org/skynet/clothoid_3d/raw/branch/master/src/pics/lcnc.jpg" alt="Clothoid 3D" width="50%">
</div>

---

## Abstract
Tool paths defined by G01/G02/G03 commands need to be smoothed to eliminate discontinuities in velocity, <br> 
acceleration, and jerk at junction points. <br> 
Traditional corner smoothing strategies are limited to planar corners due to the inherent limitations of curve fillets. <br> 
This paper presents a method to smooth space corners by blending the position, tangent, curvature, <br> 
and sharpness of adjacent trajectory segments using 3D general clothoid splines. <br> 
The proposed method extends the traditional clothoid from 2D to 3D, achieving higher continuity (G³) and maintaining properties <br> 
such as curve length parameterization and analytically expressed curvature. <br> 
The method is validated through simulations and experiments, demonstrating improved machining quality and efficiency. <br> 

---

## Programming Language

C C++

---

## Dependencies
To build and run this project, you need the following: <br> 

- **ceres**: A library for non-linear equatation solving. <br> 

```bash
	sudo apt-get install libceres-dev
```
   
Note :  In the ~/trash directory, there are example's using libraries: dogleg & gsl. <br> 
        During coding, it turned out the Ceres library is best to use. <br> 

- **eigen** a high-performance C++ linear algebra library. <br>
```bash
	sudo apt install libeigen3-dev
```
---

## Runtest
There are multiple clothoid tests available in: <br> 
~/test/main.c <br> 

---

## Key Concepts

### First Derivative: θ (Theta)
- Represents the tangent vector. <br> 
- Example: `(100, 0, 0)` is a tangent vector, and `(1, 0, 0)` is a unit tangent vector. <br> 

### Second Derivative: κ (Kappa)
- Represents the curvature of the curve. <br> 

### Third Derivative: c (Sharpness)
- Represents the rate of change of curvature. <br> 

---

## Abstract improvements

1. In eq45, page 7, the Delta Theta is calculated by: Thetai4 - Thetai0. <br> 
   The clothoid Delta Theta produced by this substraction, is not alway's correct. <br> 
   When the incorrect value is used, it shows a self intersecting clothoid. <br> 
  
   Eq.45 input Angles are in the range [0,-2*pi], [0,2*pi]. Substraction can lead to a incorrect  <br> 
   Delta Theta. <br> 
  
   To solve this issue, we convert back the thetai[x] into a xy vector. <br> 
   From here the signed angle between 2 vectors is calculated. <br>  
   This ensures a correct Delta Theta value. <br> 
  
   Problably this is also the reason why in the abstract forward & inverse transformations are used. <br> 
   In this codebase forward & inverse transformations are obsolete. <br> 
  
2. Some clothoid fit's can show a extra torsion turn. The clothoid fit then looks like a helix. <br> 
   we can request this state by calling the function : int eq45_extra_torsion_turns(). <br> 
   
   - The solution to prevent extra clothoid torsion turn is to <br> 
     add a tiny tollerance to the first or second segment coordinates. 
     A tollerance off 1e-6 for example. <br> 
     This results in a clothoid fit by out off plane segments. Non intersecting segments. <br> 
     
     For example a line-line clothoid fit is without extra torsion turn if the line-line do not intersect. <br> 
     More info can be found in the example : ~test/test_clothoid_line_line_remove_extra_torsion_turn <br> 
   
## Codebase improvements related to abstract

1. In this codebase the arc is upgraded to be used as arc-helix. <br> 
   Most gcode interpreters can use G2, G3 command's for arc and helix. <br> 
2. No need for forward & inverse transformation to do clothoid fitting. <br> 
3. Simplified algorithm1. (Find shortest distance from point in space to clothoid curve) <br> 
4. Eq19_curvature_extrema to calculate curvature extrema of clothoid compound curve. <br> 
5. Clothoid self intersecting check and endpoint fit validation. <br>
6. In ~test/test_clothoid_line_line_remove_extra_torsion_turn we remove extra unwanted torsion turns. <br>
  
## Performance

1. To find a clothoid xyz endpoint with Ceres solver takes about ~3ms. <br> 
2. To fit a entire clothoid takes around ~25ms. Here the Ceres non-linear solver is used 3 times: <br> 
    1. To find the clothoid xyz endpoint, using the 3 unknowns : s1, y11, y21. <br> 
    2. To find the closest distance to the clothoid compound curve, given a point in space, algorithm1. <br> 
    3. To fit the clothoid on curveA & curveB given the max path deviation. Filletizing a corner of 2 segments. <br> 
    
## Realtime usage

For realtime usage the clothoid fit operation has to be done inside a seperate thread because it takes <br>  
around 25 servo cycles to complete 1 clothoid fit operation. <br> 
    
---

## Building the Project, library and test app.

1. Clone repository:
   ```bash
   git clone https://codeberg.org/skynet/clothoid_3d.git
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

6. Run the library test app to verify the clothoid library is working ok:
   ```bash
   cd build/test
   ./test
   ```

---

## Usage
After building the project, you can run the executable to test the <br>  
implementation of the 3D clothoid smoothing algorithm. <br> 

---

## Code example
Create a line-line fillet, given max path deviation.

<small>

```bash

#include "time.h"
#include "../segment.h"
#include "../math/vector_math.h"
#include "../math/geometry_math.h"
#include "../solver/ceres_cppapi.h"
#include "../solver/residual_fillet_fit.h"
#include "eq19.h" 
#include "line3d.h"
#include "arc3d.h"
    
void main(){

    struct segment seg0;
    seg0.segment_type=LINE;
    double l0_start[3]={-100,0,0};
    double l0_end[3]={0,0,0};
    init_line(l0_start,l0_end,&seg0,0);

    struct segment seg1;
    seg1.segment_type=LINE;
    double l1_start[3]={0,0,0};
    double l1_end[3]={100,-100,-100};
    init_line(l1_start,l1_end,&seg1,0);

    struct segment seg2;
    seg2.segment_type=CLOTHOID;
    
    double max_deviation = 20;
    
    fit(&seg0,&seg1,&seg2,max_deviation);
    
    print_segment(&seg2);
}

```

</small> 

---

## Attribution
This implementation is based on the work of <br> 
**Qun-Bao Xiao, Min Wan, Yang Liu, Xue-Bin Qin, and Wei-Hong Zhang**. <br> 

Please refer to the original paper for a detailed explanation of the methodology and results. <br> 

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






