#ifndef HAL_DATA_TYPES_H
#define HAL_DATA_TYPES_H

#include "hal.h"

typedef struct {
    bool ok;
} skynet_t;
// skynet_t *skynet;

typedef struct {
    hal_float_t *Pin;
} float_data_t;

//! Pins
typedef struct {
    hal_bit_t *Pin;
} bit_data_t;

//typedef struct { // UINT8
//    hal_u8_t *Pin;
//} u8_data_t;

//typedef struct { // UINT16
//    hal_u16_t *Pin;
//} u16_data_t;

typedef struct { // UINT32
    hal_u32_t *Pin;
} u32_data_t;

typedef struct { // UINT64
    hal_u64_t *Pin;
} u64_data_t;

//typedef struct {
//    hal_s8_t *Pin;
//} s8_data_t;

//typedef struct {
//    hal_s16_t *Pin;
//} s16_data_t;

typedef struct {
    hal_s32_t *Pin;
} s32_data_t;

typedef struct {
    hal_s64_t *Pin;
} s64_data_t;

typedef struct {
    hal_port_t *Pin;
} port_data_t;
// port_data_t *port;

//! Params
typedef struct {
    hal_float_t Pin;
} param_float_data_t;

typedef struct {
    hal_bit_t Pin;
} param_bit_data_t;

#endif // HAL_DATA_TYPES_H
