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
#ifndef EQ20_H
#define EQ20_H

/* eq20, page 4.
 *
 * stot = total clothoid length
 * s = global length at clothoid to retrieve sharpness c.
 *
 * Retrieve the theta at s.
 *
 */
static void eq20(const double s, const double stot,
          const double sharpnessi0, const double sharpnessi1, const double sharpnessi2, const double sharpnessi3,
          const double kappai0, const double kappai1, const double kappai2, const double kappai3,
          const double thetai0, const double thetai1, const double thetai2, const double thetai3,
          const double yi1, const double yi2, const double yi3, const double yi4,
          double *theta_at_s) {

    double s1 = stot / 4.0;
    double s2 = s1 + s1;
    double s3 = s1 + s1 + s1;
    double s4 = stot;

    if (s <= s1) {
        *theta_at_s = thetai0 + kappai0 * s + (0.5*sharpnessi0) * (s * s) + (1.0 / 6.0) * yi1 * (s * s * s);
    } else if (s > s1 && s <= s2) {
        *theta_at_s = thetai1 + kappai1 * (s - s1) + (0.5*sharpnessi1) * ((s - s1) * (s - s1)) + (1.0 / 6.0) * yi2 * ((s - s1) * (s - s1) * (s - s1));
    } else if (s > s2 && s <= s3) {
        *theta_at_s = thetai2 + kappai2 * (s - s2) + (0.5*sharpnessi2) * ((s - s2) * (s - s2)) + (1.0 / 6.0) * yi3 * ((s - s2) * (s - s2) * (s - s2));
    } else if (s > s3 && s <= s4) {
        *theta_at_s = thetai3 + kappai3 * (s - s3) + (0.5*sharpnessi3) * ((s - s3) * (s - s3)) + (1.0 / 6.0) * yi4 * ((s - s3) * (s - s3) * (s - s3));
    } else {
        // Handle the case where s is out of bounds (optional)
        *theta_at_s = 0.0; // or some other default value
        printf("error: eq20.\n");
    }
}

/* Get the theta given s (length) for a single clothoid.
 *
 */
static void eq20_single_clothoid(const double s,
                          const double sharpnessi0,
                          const double kappai0,
                          const double thetai0,
                          const double yi1,
                          double *theta_at_s) {
    *theta_at_s = thetai0 + kappai0 * s + (0.5*sharpnessi0) * (s * s) + (1.0 / 6.0) * yi1 * (s * s * s);
}

#endif // EQ20_H
