/********************************************************************
*   Written by Grotius, alias Skynet.
*   michelwijnja@gmail.com
*
* Author: Michel Wijnja
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2024 All rights reserved.
********************************************************************/
#include "gcode/line3d.h"
#include "gcode/arc3d.h"
#include "gcode/clothoid3d.h"
#include "gcode/fit.h"
#include "experimental/velocity.h"

int main() {

    // test_line();
    // test_arc_xy_plane();
    // test_arc_yz_plane();
    // test_arc_xz_plane();
    // test_clothoid_line_line();

    // test_clothoid_helix_helix();

    // test_line_line_fit();

    // test_line_arc_fit_g3_0();
    // test_line_arc_fit_g3_1();
    // test_line_arc_fit_g3_2();
    // test_line_arc_fit_g3_3();

    //test_line_arc_fit_g2_0();
    //test_line_arc_fit_g2_1();
    //test_line_arc_fit_g2_2();
    // test_line_arc_fit_g2_3();

    // test_arc_arc_fit();

    // test_fit();

    // test_clothoid_line_line_remove_extra_torsion_turn();

    // velocity_test();

    // velocity_time_test();


    double jerk=1;
    double acc=1;
    double dv, time;
    jerk_to_dv(jerk,acc,&dv);
    initialise_null_frame(acc,dv,&time);

    return 0;
}











