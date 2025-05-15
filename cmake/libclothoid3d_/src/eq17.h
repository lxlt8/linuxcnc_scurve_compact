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
#ifndef EQ17_H
#define EQ17_H

/* eq17, page 4.
 *
 * stot = total clothoid length
 * s = global length at clothoid to retrieve gamma.
 * yi[x] = gamma value's. Clothoid rigrid values.
 *
 * Retrieve the gamma for s.
 *
 */
static void eq17(const double s, const double stot,
          const double yi1, const double yi2, const double yi3, const double yi4,
          double *yi_at_s) {

    double s1 = stot / 4.0;
    double s2 = s1 + s1;
    double s3 = s1 + s1 + s1;
    double s4 = stot;

    if (s <= s1) {
        *yi_at_s = yi1;
    } else if (s > s1 && s <= s2) {
        *yi_at_s = yi2;
    } else if (s > s2 && s <= s3) {
        *yi_at_s = yi3;
    } else if (s > s3 && s <= s4) {
        *yi_at_s = yi4;
    } else {
        // Handle the case where s is out of bounds (optional)
        *yi_at_s = 0.0; // or some other default value
        printf("error: eq17. \n");
    }
}

#endif // EQ17_H
