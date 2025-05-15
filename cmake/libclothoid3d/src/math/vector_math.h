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
#ifndef VECTOR_MATH_H
#define VECTOR_MATH_H

#include <math.h>

// Function to print a 3-element vector with a label
static void print_vector(const char* label, const double vec[3]) {
    printf("%s [%f, %f, %f]\n", label, vec[0], vec[1], vec[2]);
}

static void copy_vector(const double src[3], double dest[3]) {
    for (int i = 0; i < 3; i++) {
        dest[i] = src[i];
    }
}

/* Function to negate a vector (reverse its direction) */
static void negate_vector(double vec[3]) {
    vec[0] = -vec[0];
    vec[1] = -vec[1];
    vec[2] = -vec[2];
}

/* Function to calculate the magnitude xyz of a 3D vector.
 * Magnitude is the total length of the vector, given its components (x, y, z).
 */
static double magnitude_xyz(const double vec[3]) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}

/* Function to calculate the magnitude xy of a 3D vector.
 * Magnitude is total lenght of a line.
 */
static double magnitude_xy(const double vec[3]) {
    return sqrt(vec[0] * vec[0] + vec[1] * vec[1]);
}

// Function to normalize a 3D vector
static void normalize(double v[3]) {
    // Calculate the magnitude of the vector
    double magnitude = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    // Avoid division by zero if magnitude is extremely small
    if (magnitude > 1e-12) {
        // Normalize the vector by dividing each component by the magnitude
        v[0] /= magnitude;
        v[1] /= magnitude;
        v[2] /= magnitude;
    } else {
        // If the magnitude is very small, set the vector to zero
        v[0] = v[1] = v[2] = 0.0;
    }
}

// Function to scale a vector by a given length
static void scale_vector(const double dir[3], double length, double result[3]) {
    double norm_dir[3];
    norm_dir[0] = dir[0];
    norm_dir[1] = dir[1];
    norm_dir[2] = dir[2];

    normalize(norm_dir);  // Normalize the direction vector

    result[0] = norm_dir[0] * length;
    result[1] = norm_dir[1] * length;
    result[2] = norm_dir[2] * length;
}

// Function to calculate the unit vector between two points
static void vector_3d(double p1[3], double p2[3], double v[3]) {
    // Calculate the vector components (p2 - p1)
    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];
}

// Function to subtract two vectors
static void subtract_vectors(const double v1[3], const double v2[3], double result[3]) {
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

static void add_vectors(const double a[3], const double b[3], double result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = a[i] + b[i];
    }
}

// Function to calculate the unit vector between two points
static void unit_vector(const double p1[3], const double p2[3], double v[3]) {
    // Calculate the vector components (p2 - p1)
    v[0] = p2[0] - p1[0];
    v[1] = p2[1] - p1[1];
    v[2] = p2[2] - p1[2];

    // Normalize the vector (divide each component by the magnitude)
    normalize(v);
}

// Function to calculate the cross product of two vectors and return the result vr.
static void cross_product(const double v1[3], const double v2[3], double vr[3]) {
    vr[0] = v1[1] * v2[2] - v1[2] * v2[1];  // x-component
    vr[1] = v1[2] * v2[0] - v1[0] * v2[2];  // y-component
    vr[2] = v1[0] * v2[1] - v1[1] * v2[0];  // z-component
}

// Dot product of two vectors
static double dot_product(const double v1[3], const double v2[3]) {
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

// Function to calculate the cross product of two vectors and return the result vr.
static void cross_product_normalized(double v1[3], double v2[3], double vr[3]) {
    vr[0] = v1[1] * v2[2] - v1[2] * v2[1];  // x-component
    vr[1] = v1[2] * v2[0] - v1[0] * v2[2];  // y-component
    vr[2] = v1[0] * v2[1] - v1[1] * v2[0];  // z-component

    normalize(vr);
}

#endif // VECTOR_MATH_H
