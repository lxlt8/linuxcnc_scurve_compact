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
#ifndef VECTOR_H
#define VECTOR_H

#include "stdio.h"
#include "stdlib.h"
#include "emcmot_segment.h"

#define VECTOR_BUFFER_SIZE 40    // 10 is about the minimal size.
                                 // Using lower value will cause ringbuffer index to stay zero.
struct vector {
    struct emcmot_segment seg[VECTOR_BUFFER_SIZE];
    int push_counter;
};

static struct vector* vector_init_ptr() {
    // Allocate memory for the vector
    struct vector* vec = (struct vector*)malloc(sizeof(struct vector));
    if (vec == NULL) {
        printf("Error: Failed to allocate memory for vector.\n");
        return NULL; // Return NULL if allocation fails
    }

    // Initialize the vector
    vec->push_counter = 0;

    // Optionally, initialize the segments in the buffer
    for (int i = 0; i < VECTOR_BUFFER_SIZE; i++) {
        zero_emcmot_segment(&vec->seg[i]);
    }

    return vec;
}

// Add a segment to the vector
static void vector_push_back(struct vector *ptr, struct emcmot_segment seg) {
    // Use modulo to wrap the push_counter within the buffer size
    int index = ptr->push_counter % VECTOR_BUFFER_SIZE;

    // Add the segment to the buffer at the calculated index
    ptr->seg[index] = seg;

    // Increment the push_counter
    ptr->push_counter++;

    // printf("Vector added segment, push_counter: %d \n", ptr->push_counter);

    // Print the buffer contents (optional, for debugging)
    // vector_print_content(ptr);
}

// Get the number of elements currently in the vector
static int vector_size(struct vector *ptr) {
    return (ptr->push_counter < VECTOR_BUFFER_SIZE) ? ptr->push_counter : VECTOR_BUFFER_SIZE;
}

static int push_counter(struct vector *ptr){
    return ptr->push_counter;
}

// Clear the vector
static void vector_clear(struct vector *ptr) {
    ptr->push_counter = 0;

    // Clear the buffer contents
    for (int i = 0; i < VECTOR_BUFFER_SIZE; i++) {
        zero_emcmot_segment(&ptr->seg[i]);
    }
}

// Get a pointer to the segment at a specific index
static struct emcmot_segment* vector_at(struct vector *ptr, int index) {
    // Check if the buffer is empty
    if (vector_size(ptr) == 0) {
        printf("Error: Buffer is empty.\n");
        return NULL;
    }

    // Calculate the actual index in the buffer using modulo
    int actual_index = index % VECTOR_BUFFER_SIZE;

    // Return a pointer to the segment
    return &ptr->seg[actual_index];
}

// Print the contents of the vector
static void vector_print_content(struct vector *ptr) {
    printf("\nVector content:\n");

    // Iterate through the buffer and print each segment
    for (int i = 0; i < VECTOR_BUFFER_SIZE; i++) {
        struct emcmot_segment *seg = vector_at(ptr,i);
        print_emcmot_segment_id_length_vo_vm_ve(seg);
    }

    printf("\n");
}

#endif // VECTOR_H
