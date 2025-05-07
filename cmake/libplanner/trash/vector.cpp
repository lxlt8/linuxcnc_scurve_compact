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
#include <vector>
#include <cstdio>
#include <cstdlib>
#include "vector.h"

// Constructor initializes shared memory and the shared vector
vector::vector(){}

extern "C" {

struct vector* vector_init_ptr() {
    return new vector();
}

void vector_remove_ptr(struct vector *ptr){
    delete ptr;
}

void vector_push_back(struct vector *ptr, struct emcmot_segment seg) {
    ptr->vec.push_back(seg);

    // Calculate size in bytes
    // size_t sizeInBytes = ptr->vec.size() * sizeof(emcmot_segment);

    // Convert to megabytes
    // double sizeInMB = static_cast<double>(sizeInBytes) / (1024 * 1024);

    // Print size in MB
    // std::cout << "Size of vector in MB: " << sizeInMB << " MB" << std::endl;
}

void vector_pop_back(struct vector *ptr) {
    if (ptr->vec.size()>0) {
        ptr->vec.pop_back();
    }
}

void vector_pop_front(struct vector *ptr) {
    if (!ptr->vec.empty()) {
        ptr->vec.erase(ptr->vec.begin()); // Removes the first element
    }
}

struct emcmot_segment* vector_back(struct vector *ptr) {
    return &ptr->vec.back();
}

int vector_size(struct vector *ptr) {
    return ptr->vec.size();
}

void vector_clear(struct vector *ptr) {
    ptr->vec.clear();
}

void vector_remove_at(struct vector *ptr, int index) {
    ptr->vec.erase(ptr->vec.begin() + index);
}

struct emcmot_segment* vector_at(struct vector *ptr, int index) {
    return &ptr->vec.at(index);
}

} // extern "C"

