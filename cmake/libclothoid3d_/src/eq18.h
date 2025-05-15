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
#ifndef EQ18_H
#define EQ18_H

/* eq18, page 4.
 *
 * stot = total clothoid length
 * s = global length at clothoid to retrieve sharpness c.
 *
 * Retrieve the sharpness for s.
 *
 */
static void eq18(const double s, const double stot,
          const double sharpnessi0, const double sharpnessi1, const double sharpnessi2, const double sharpnessi3,
          const double yi1, const double yi2, const double yi3, const double yi4,
          double *sharpness_at_s) {

    double s1 = stot / 4.0;
    double s2 = s1 + s1;
    double s3 = s1 + s1 + s1;
    double s4 = stot;

    if (s <= s1) {
        *sharpness_at_s = sharpnessi0 + yi1 * s;
    } else if (s > s1 && s <= s2) {
        *sharpness_at_s = sharpnessi1 + yi2 * (s - s1);
    } else if (s > s2 && s <= s3) {
        *sharpness_at_s = sharpnessi2 + yi3 * (s - s2);
    } else if (s > s3 && s <= s4) {
        *sharpness_at_s = sharpnessi3 + yi4 * (s - s3);
    } else {
        // Handle the case where s is out of bounds (optional)
        *sharpness_at_s = 0.0; // or some other default value
        printf("error: eq18. \n");
    }
}

static void eq18_single_clothoid(const double s,
                          const double sharpnessi0,
                          const double yi1,
                          double *sharpness_at_s) {
    *sharpness_at_s = sharpnessi0 + yi1 * s;
}

#endif // EQ18_H
