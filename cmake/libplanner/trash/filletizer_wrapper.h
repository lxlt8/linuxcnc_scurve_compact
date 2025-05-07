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
#ifndef FILLETIZER_WRAPPER_H
#define FILLETIZER_WRAPPER_H

#include "filletizer.h"  // Header where filletizer class is defined
#include "segment.h"

// Creates a new filletizer instance and returns its pointer
extern struct filletizer* filletizer_init_ptr();

// Destroys the filletizer instance
extern void filletizer_remove_ptr(struct filletizer* filletizer_ptr);

// Applies fillet transformations on segments.
// Calculates kmax for the fillet segment s1.
// Fillet type clothoid=2.
extern int filletizer_filletize(struct filletizer* filletizer_ptr,
                                struct emcmot_segment* s0,
                                struct emcmot_segment* s1,
                                struct emcmot_segment* s2,
                                int fillet_type,
                                int debug);

extern int filletizer_interpolate_segment(struct filletizer* filletizer_ptr,
                                          struct emcmot_segment* seg,
                                          double progress,
                                          struct EmcPose *pos);


#endif // FILLETIZER_WRAPPER_H








