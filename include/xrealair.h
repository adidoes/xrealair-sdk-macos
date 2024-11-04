#pragma once
//
// Copyright (c) 2023-2024. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#ifndef __cplusplus
#include <stdbool.h>
#include <stdint.h>
#else
#include <cstdint>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ===== HID IDs =====
#define NUM_SUPPORTED_PRODUCTS 4

extern const uint16_t xreal_vendor_id;
extern const uint16_t xreal_product_ids [NUM_SUPPORTED_PRODUCTS];

bool is_xreal_product_id(uint16_t product_id);
int xreal_imu_interface_id(uint16_t product_id);
int xreal_mcu_interface_id(uint16_t product_id);

// ===== Device Base =====
bool device_init();
void device_exit();

// ===== CRC32 =====
uint32_t crc32_checksum(const uint8_t* buf, uint32_t len);

// ===== Device MCU =====
// [All MCU constants from device_mcu.h]
#define DEVICE_MCU_MSG_R_BRIGHTNESS 0x03
// ... [rest of the MCU constants]

enum device_mcu_error_t {
    // ... [MCU error enum content]
};

struct __attribute__((__packed__)) device_mcu_packet_t {
    // ... [MCU packet struct content]
};

enum device_mcu_event_t {
    // ... [MCU event enum content]
};

typedef enum device_mcu_error_t device_mcu_error_type;
typedef struct device_mcu_packet_t device_mcu_packet_type;
typedef enum device_mcu_event_t device_mcu_event_type;
typedef void (*device_mcu_event_callback)(
    uint64_t timestamp,
    device_mcu_event_type event,
    uint8_t brightness,
    const char* msg
);

struct device_mcu_t {
    // ... [MCU device struct content]
};

typedef struct device_mcu_t device_mcu_type;

// [MCU function declarations]
device_mcu_error_type device_mcu_open(device_mcu_type* device, device_mcu_event_callback callback);
// ... [rest of MCU functions]

// ===== Device IMU =====
// [All IMU constants from device_imu.h]
#define DEVICE_IMU_MSG_GET_CAL_DATA_LENGTH 0x14
// ... [rest of the IMU constants]

// [All IMU type declarations]
enum device_imu_error_t {
    // ... [IMU error enum content]
};

// [Rest of IMU structs and type definitions]
struct device_imu_packet_t {
    // ... [IMU packet struct content]
};

// [IMU function declarations]
device_imu_error_type device_imu_open(device_imu_type* device, device_imu_event_callback callback);
// ... [rest of IMU functions]

#ifdef __cplusplus
}
#endif
