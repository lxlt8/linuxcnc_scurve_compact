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
#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include "segment.h"

#define RINGBUFFER_SIZE 10 // Use #define for constants

struct ringbuffer {
    struct emcmot_segment seg[RINGBUFFER_SIZE];
    int push_counter;
    int motion_counter;
};

void ringbuffer_print_content(struct ringbuffer *rb) {
    printf("\nRing Buffer Contents:\n");

    // Determine the number of elements in the buffer
    int count = (rb->push_counter < RINGBUFFER_SIZE) ? rb->push_counter : RINGBUFFER_SIZE;

    // Iterate through the buffer and print each segment
    for (int i = 0; i < count; i++) {
        // Calculate the actual index in the circular buffer
        int index = (rb->push_counter - count + i) % RINGBUFFER_SIZE;

        // Print the segment data (assuming `data` is a field in `emcmot_segment`)

        printf("Push counter: %d, motion counter:P %d, Endpos x: %f, y: %f, z: %f \n",
               rb->push_counter,
               rb->motion_counter,
               rb->seg[index].end.tran.x,
               rb->seg[index].end.tran.y,
               rb->seg[index].end.tran.z);
    }

    if (count == 0) {
        printf("Buffer is empty.\n");
    }
}

// Initialize or reset the ring buffer
void ringbuffer_reset(struct ringbuffer *rb) {
    rb->push_counter = 0;
    rb->motion_counter = 0;

    for(int i=0; i<RINGBUFFER_SIZE; i++){
        zero_emcmot_segment(&rb->seg[i]);
    }
}

// Add a segment to the ring buffer
void ringbuffer_add_segment(struct ringbuffer *rb, struct emcmot_segment seg) {
    // Use modulo to wrap the push_counter within the buffer size
    int index = rb->push_counter % RINGBUFFER_SIZE;

    // Add the segment to the buffer at the calculated index
    rb->seg[index] = seg;

    // Increment the push_counter
    rb->push_counter++;

    printf("ringbuffer add's segment, push_counter: %d \n",rb->push_counter);

    ringbuffer_print_content(rb);
}

// Get the current push counter value
int ringbuffer_push_counter(struct ringbuffer *rb) {
    return rb->push_counter;
}

// Get the segment at a specific index (based on the push counter)
void ringbuffer_item_at(struct ringbuffer *rb, int index_counter, struct emcmot_segment *seg) {
    // Calculate the actual index in the buffer
    int index = index_counter % RINGBUFFER_SIZE;

    // Copy the segment data to the caller's pointer
    *seg = rb->seg[index];
}



#endif // RINGBUFFER_H
