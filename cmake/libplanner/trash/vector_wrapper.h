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
#ifndef VECTOR_WRAPPER_H
#define VECTOR_WRAPPER_H

#include "vector.h"
#include "segment.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes a new vector pointer.
extern struct vector* vector_init_ptr();

// Deletes the vector pointer.
extern void vector_remove_ptr(struct vector *ptr);

// Adds an element to the end of the vector.
extern void vector_push_back(struct vector *ptr, struct emcmot_segment seg);

// Removes the last element from the vector.
extern void vector_pop_back(struct vector *ptr);

// Removes the last element from the vector.
extern void vector_pop_front(struct vector *ptr);

// Points to the last element in the vector.
extern struct emcmot_segment* vector_back(struct vector *ptr);

// Returns the number of elements in the vector.
extern int vector_size(struct vector *ptr);

// Clears all elements in the vector.
extern void vector_clear(struct vector *ptr);

// Removes the element at the specified index in the vector.
extern void vector_remove_at(struct vector *ptr, int index);

// Returns a pointer to the element at the specified index in the vector.
extern struct emcmot_segment* vector_at(struct vector *ptr, int index);

// Copy content of vec into shared vec. Using adress: 65536
extern void vector_copy_to_shared_vector(struct vector *ptr);

#ifdef __cplusplus
}
#endif

#endif // VECTOR_WRAPPER_H
