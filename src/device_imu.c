//
// Created by thejackimonster on 30.03.23.
//
// Copyright (c) 2023-2024 thejackimonster. All rights reserved.
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

#include "device_imu.h"
#include "device.h"
#include "magnet.h"

#include <Fusion/FusionAxes.h>
#include <Fusion/FusionMath.h>
#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

#include <Fusion/Fusion.h>
#include <json-c/json.h>

#include <hidapi/hidapi.h>

#include "crc32.h"
#include "hid_ids.h"

#define GRAVITY_G (9.806f)

#ifndef NDEBUG
#define device_imu_error(msg) fprintf(stderr, "ERROR: %s\n", msg)
#else
#define device_imu_error(msg) (0)
#endif

struct device_imu_calibration_t
{
	FusionMatrix gyroscopeMisalignment;
	FusionVector gyroscopeSensitivity;
	FusionVector gyroscopeOffset;

	// continuously updated
	FusionOffset gyroscopeFusionOffset;

	FusionMatrix accelerometerMisalignment;
	FusionVector accelerometerSensitivity;
	FusionVector accelerometerOffset;

	FusionMatrix magnetometerMisalignment;
	FusionVector magnetometerSensitivity;
	FusionVector magnetometerOffset;

	bool using_magnet;
	FusionMatrix softIronMatrix;
	FusionVector hardIronOffset;

	FusionQuaternion noises;
};

#define MAX_PACKET_SIZE 64

static bool send_payload(device_imu_type *device, uint8_t size, const uint8_t *payload)
{
	int payload_size = size;
	if (payload_size > MAX_PACKET_SIZE)
	{
		payload_size = MAX_PACKET_SIZE;
	}

	int transferred = hid_write(device->handle, payload, payload_size);

	if (transferred != payload_size)
	{
		device_imu_error("Sending payload failed");
		return false;
	}

	return (transferred == size);
}

static bool recv_payload(device_imu_type *device, uint8_t size, uint8_t *payload)
{
	int payload_size = size;
	if (payload_size > MAX_PACKET_SIZE)
	{
		payload_size = MAX_PACKET_SIZE;
	}

	int transferred = hid_read(device->handle, payload, payload_size);

	if (transferred >= payload_size)
	{
		transferred = payload_size;
	}

	if (transferred == 0)
	{
		return false;
	}

	if (transferred != payload_size)
	{
		device_imu_error("Receiving payload failed");
		return false;
	}

	return (transferred == size);
}

struct __attribute__((__packed__)) device_imu_payload_packet_t
{
	uint8_t head;
	uint32_t checksum;
	uint16_t length;
	uint8_t msgid;
	uint8_t data[56];
};

typedef struct device_imu_payload_packet_t device_imu_payload_packet_type;

static bool send_payload_msg(device_imu_type *device, uint8_t msgid, uint8_t len, const uint8_t *data)
{
	static device_imu_payload_packet_type packet;

	const uint16_t packet_len = 3 + len;
	const uint16_t payload_len = 5 + packet_len;

	packet.head = 0xAA;
	packet.length = htole16(packet_len);
	packet.msgid = msgid;

	memcpy(packet.data, data, len);
	packet.checksum = htole32(
			crc32_checksum(
					(const uint8_t *)(&packet.length),
					packet.length));

	return send_payload(device, payload_len, (uint8_t *)(&packet));
}

static bool send_payload_msg_signal(device_imu_type *device, uint8_t msgid, uint8_t signal)
{
	return send_payload_msg(device, msgid, 1, &signal);
}

static bool recv_payload_msg(device_imu_type *device, uint8_t msgid, uint8_t len, uint8_t *data)
{
	static device_imu_payload_packet_type packet;

	packet.head = 0;
	packet.length = 0;
	packet.msgid = 0;

	const uint16_t packet_len = 3 + len;
	const uint16_t payload_len = 5 + packet_len;

	if (!recv_payload(device, payload_len, (uint8_t *)(&packet)))
	{
		return false;
	}

	if (packet.msgid != msgid)
	{
		return false;
	}

	memcpy(data, packet.data, len);
	return true;
}

static FusionVector json_object_get_vector(struct json_object *obj)
{
	if ((!json_object_is_type(obj, json_type_array)) ||
			(json_object_array_length(obj) != 3))
	{
		return FUSION_VECTOR_ZERO;
	}

	FusionVector vector;
	vector.axis.x = (float)json_object_get_double(json_object_array_get_idx(obj, 0));
	vector.axis.y = (float)json_object_get_double(json_object_array_get_idx(obj, 1));
	vector.axis.z = (float)json_object_get_double(json_object_array_get_idx(obj, 2));
	return vector;
}

static FusionQuaternion json_object_get_quaternion(struct json_object *obj)
{
	if ((!json_object_is_type(obj, json_type_array)) ||
			(json_object_array_length(obj) != 4))
	{
		return FUSION_IDENTITY_QUATERNION;
	}

	FusionQuaternion quaternion;
	quaternion.element.x = (float)json_object_get_double(json_object_array_get_idx(obj, 0));
	quaternion.element.y = (float)json_object_get_double(json_object_array_get_idx(obj, 1));
	quaternion.element.z = (float)json_object_get_double(json_object_array_get_idx(obj, 2));
	quaternion.element.w = (float)json_object_get_double(json_object_array_get_idx(obj, 3));
	return quaternion;
}

// gyro bias reading isn't in the same coordinate system as the raw gyro readings
static FusionVector align_gyro_bias_coordinate_system(FusionVector v)
{
	FusionVector result = v;
	result.axis.x = -v.axis.x;
	return result;
}

// some calibration readings are in JPL coordinate system, convert to Hamilton
static FusionQuaternion quaternion_jpl_to_hamilton(FusionQuaternion q)
{
	FusionQuaternion result = {
			.element = {
					.x = -q.element.x,
					.y = -q.element.y,
					.z = -q.element.z,
					.w = q.element.w}};
	return result;
}

device_imu_error_type device_imu_open(device_imu_type *device, device_imu_event_callback callback)
{
	if (!device)
	{
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	memset(device, 0, sizeof(device_imu_type));
	device->vendor_id = xreal_vendor_id;
	device->product_id = 0;
	device->callback = callback;

	if (!device_init())
	{
		device_imu_error("Not initialized");
		return DEVICE_IMU_ERROR_NOT_INITIALIZED;
	}

	struct hid_device_info *info = hid_enumerate(
			device->vendor_id,
			device->product_id);

	struct hid_device_info *it = info;
	while (it)
	{
		int interface_id = xreal_imu_interface_id(it->product_id);
		if (interface_id != -1 && it->interface_number == interface_id)
		{
			device->product_id = it->product_id;
			device->handle = hid_open_path(it->path);

			if (device->handle != NULL)
			{
				wchar_t serial_number[256];
				int result = hid_get_serial_number_string(device->handle, serial_number, 256);
				if (result == 0)
				{
					size_t needed_size = wcstombs(NULL, serial_number, 0);
					device->serial_number = malloc(needed_size + 1);
					if (device->serial_number != NULL)
					{
						wcstombs(device->serial_number, serial_number, needed_size + 1);
#ifndef NDEBUG
						printf("Found IMU device with product_id 0x%x, interface %d, serial number %s\n",
									 device->product_id, interface_id, device->serial_number);
#endif
					}
				}
				else
				{
					device_imu_error("Failed to retrieve serial number");
				}
			}
			break;
		}

		it = it->next;
	}

	hid_free_enumeration(info);

	if (!device->handle)
	{
		device_imu_error("No handle");
		return DEVICE_IMU_ERROR_NO_HANDLE;
	}

	if (!send_payload_msg_signal(device, DEVICE_IMU_MSG_START_IMU_DATA, 0x0))
	{
		device_imu_error("Failed sending payload to stop imu data stream");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	device_imu_clear(device);

	if (!send_payload_msg(device, DEVICE_IMU_MSG_GET_STATIC_ID, 0, NULL))
	{
		device_imu_error("Failed sending payload to get static id");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	uint32_t static_id = 0;
	if (recv_payload_msg(device, DEVICE_IMU_MSG_GET_STATIC_ID, 4, (uint8_t *)&static_id))
	{
		device->static_id = static_id;
	}
	else
	{
		device->static_id = 0x20220101;
	}

	device->calibration = malloc(sizeof(device_imu_calibration_type));
	device_imu_reset_calibration(device, true, true, true);

	if (!send_payload_msg(device, DEVICE_IMU_MSG_GET_CAL_DATA_LENGTH, 0, NULL))
	{
		device_imu_error("Failed sending payload to get calibration data length");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	uint32_t calibration_len = 0;
	if (recv_payload_msg(device, DEVICE_IMU_MSG_GET_CAL_DATA_LENGTH, 4, (uint8_t *)&calibration_len))
	{
		char *calibration_data = malloc(calibration_len + 1);

		uint32_t position = 0;
		while (position < calibration_len)
		{
			const uint32_t remaining = (calibration_len - position);

			if (!send_payload_msg(device, DEVICE_IMU_MSG_CAL_DATA_GET_NEXT_SEGMENT, 0, NULL))
			{
				break;
			}

			const uint8_t next = (remaining > 56 ? 56 : remaining);

			if (!recv_payload_msg(device, DEVICE_IMU_MSG_CAL_DATA_GET_NEXT_SEGMENT, next, (uint8_t *)calibration_data + position))
			{
				break;
			}

			position += next;
		}

		calibration_data[calibration_len] = '\0';

#ifndef NDEBUG
		printf("Calibration data: %s\n", calibration_data);
#endif

		struct json_tokener *tokener = json_tokener_new();
		struct json_object *root = json_tokener_parse_ex(tokener, calibration_data, calibration_len);
		struct json_object *imu = json_object_object_get(root, "IMU");
		struct json_object *dev1 = json_object_object_get(imu, "device_1");

		FusionVector accel_bias = json_object_get_vector(json_object_object_get(dev1, "accel_bias"));
		FusionQuaternion accel_q_gyro = quaternion_jpl_to_hamilton(json_object_get_quaternion(json_object_object_get(dev1, "accel_q_gyro")));
		FusionVector gyro_bias = json_object_get_vector(json_object_object_get(dev1, "gyro_bias"));
		FusionQuaternion gyro_q_mag = quaternion_jpl_to_hamilton(json_object_get_quaternion(json_object_object_get(dev1, "gyro_q_mag")));
		FusionVector mag_bias = json_object_get_vector(json_object_object_get(dev1, "mag_bias"));
		FusionQuaternion imu_noises = json_object_get_quaternion(json_object_object_get(dev1, "imu_noises"));
		FusionVector scale_accel = json_object_get_vector(json_object_object_get(dev1, "scale_accel"));
		FusionVector scale_gyro = json_object_get_vector(json_object_object_get(dev1, "scale_gyro"));
		FusionVector scale_mag = json_object_get_vector(json_object_object_get(dev1, "scale_mag"));

		const FusionQuaternion accel_q_mag = FusionQuaternionMultiply(accel_q_gyro, gyro_q_mag);

		device->calibration->gyroscopeMisalignment = FusionQuaternionToMatrix(accel_q_gyro);
		device->calibration->gyroscopeSensitivity = scale_gyro;
		device->calibration->gyroscopeOffset = align_gyro_bias_coordinate_system(gyro_bias);

		device->calibration->accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
		device->calibration->accelerometerSensitivity = scale_accel;
		device->calibration->accelerometerOffset = accel_bias;

		device->calibration->magnetometerMisalignment = FusionQuaternionToMatrix(gyro_q_mag);
		device->calibration->magnetometerSensitivity = scale_mag;
		device->calibration->magnetometerOffset = FUSION_VECTOR_ZERO;

		device->calibration->noises = imu_noises;

		json_tokener_free(tokener);
		free(calibration_data);
	}

	if (!send_payload_msg_signal(device, DEVICE_IMU_MSG_START_IMU_DATA, 0x1))
	{
		device_imu_error("Failed sending payload to start imu data stream");
		return DEVICE_IMU_ERROR_PAYLOAD_FAILED;
	}

	const uint32_t SAMPLE_RATE = 1000;
	FusionOffsetInitialise(&device->calibration->gyroscopeFusionOffset, SAMPLE_RATE);

	device->ahrs = malloc(sizeof(FusionAhrs));
	FusionAhrsInitialise((FusionAhrs *)device->ahrs);

	// Since we're only accepting very low-error accel and magnet readings, we can turn the gain up high to rely on them and
	// prevent drift. A lower threshold means we'll be incurring higher rejection rates for values outside that range, use a longer
	// recovery trigger period to avoid triggering that state.
	const FusionAhrsSettings settings = {
			.convention = FusionConventionNed,
			.gain = 0.7f,
			.accelerationRejection = 2.0f,
			.magneticRejection = 2.0f,
			.recoveryTriggerPeriod = 10 * SAMPLE_RATE};

	FusionAhrsSetSettings((FusionAhrs *)device->ahrs, &settings);
	return DEVICE_IMU_ERROR_NO_ERROR;
}

device_imu_error_type device_imu_reset_calibration(device_imu_type *device, bool gyro, bool accel, bool magnet)
{
	if (!device)
	{
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (!device->calibration)
	{
		device_imu_error("Not allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}

	if (gyro)
	{
		device->calibration->gyroscopeMisalignment = FUSION_IDENTITY_MATRIX;
		device->calibration->gyroscopeSensitivity = FUSION_VECTOR_ONES;
		device->calibration->gyroscopeOffset = FUSION_VECTOR_ZERO;
	}

	if (accel)
	{
		device->calibration->accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
		device->calibration->accelerometerSensitivity = FUSION_VECTOR_ONES;
		device->calibration->accelerometerOffset = FUSION_VECTOR_ZERO;
	}

	if (magnet)
	{
		device->calibration->magnetometerMisalignment = FUSION_IDENTITY_MATRIX;
		device->calibration->magnetometerSensitivity = FUSION_VECTOR_ONES;
		device->calibration->magnetometerOffset = FUSION_VECTOR_ZERO;

		device->calibration->using_magnet = false;
		device->calibration->softIronMatrix = FUSION_IDENTITY_MATRIX;
		device->calibration->hardIronOffset = FUSION_VECTOR_ZERO;
	}

	// unsure which sensor this applies to, but it is currently unused
	device->calibration->noises = FUSION_IDENTITY_QUATERNION;
	device->calibration->noises.element.w = 0.0f;
}

device_imu_error_type device_imu_load_calibration(device_imu_type *device, const char *path)
{
	if (!device)
	{
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (!device->calibration)
	{
		device_imu_error("Not allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}

	FILE *file = fopen(path, "rb");
	if (!file)
	{
		device_imu_error("No file opened");
		return DEVICE_IMU_ERROR_FILE_NOT_OPEN;
	}

	device_imu_error_type result = DEVICE_IMU_ERROR_NO_ERROR;

	size_t count;
	count = fread(device->calibration, 1, sizeof(device_imu_calibration_type), file);

	if (sizeof(device_imu_calibration_type) != count)
	{
		device_imu_error("Not fully loaded");
		result = DEVICE_IMU_ERROR_LOADING_FAILED;
	}

	if (0 != fclose(file))
	{
		device_imu_error("No file closed");
		return DEVICE_IMU_ERROR_FILE_NOT_CLOSED;
	}

	return result;
}

device_imu_error_type device_imu_save_calibration(device_imu_type *device, const char *path)
{
	if (!device)
	{
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (!device->calibration)
	{
		device_imu_error("Not allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}

	FILE *file = fopen(path, "wb");
	if (!file)
	{
		device_imu_error("No file opened");
		return DEVICE_IMU_ERROR_FILE_NOT_OPEN;
	}

	device_imu_error_type result = DEVICE_IMU_ERROR_NO_ERROR;
	size_t count;
	count = fwrite(device->calibration, 1, sizeof(device_imu_calibration_type), file);

	if (sizeof(device_imu_calibration_type) != count)
	{
		device_imu_error("Not fully saved");
		result = DEVICE_IMU_ERROR_SAVING_FAILED;
	}

	if (0 != fclose(file))
	{
		device_imu_error("No file closed");
		return DEVICE_IMU_ERROR_FILE_NOT_CLOSED;
	}

	return result;
}

static void device_imu_callback(device_imu_type *device,
																uint64_t timestamp,
																device_imu_event_type event)
{
	if (!device->callback)
	{
		return;
	}

	device->callback(timestamp, event, device->ahrs);
}

static int32_t pack32bit_signed(const uint8_t *data)
{
	uint32_t unsigned_value = (data[0]) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	return ((int32_t)unsigned_value);
}

static int32_t pack24bit_signed(const uint8_t *data)
{
	uint32_t unsigned_value = (data[0]) | (data[1] << 8) | (data[2] << 16);
	if ((data[2] & 0x80) != 0)
		unsigned_value |= (0xFF << 24);
	return ((int32_t)unsigned_value);
}

static int16_t pack16bit_signed(const uint8_t *data)
{
	uint16_t unsigned_value = (data[1] << 8) | (data[0]);
	return (int16_t)unsigned_value;
}

static int32_t pack32bit_signed_swap(const uint8_t *data)
{
	uint32_t unsigned_value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
	return ((int32_t)unsigned_value);
}

static int16_t pack16bit_signed_swap(const uint8_t *data)
{
	uint16_t unsigned_value = (data[0] << 8) | (data[1]);
	return (int16_t)unsigned_value;
}

static int16_t pack16bit_signed_bizarre(const uint8_t *data)
{
	uint16_t unsigned_value = (data[0]) | ((data[1] ^ 0x80) << 8);
	return (int16_t)unsigned_value;
}

static void readIMU_from_packet(const device_imu_packet_type *packet,
																FusionVector *gyroscope,
																FusionVector *accelerometer,
																FusionVector *magnetometer)
{
	int32_t vel_m = pack16bit_signed(packet->angular_multiplier);
	int32_t vel_d = pack32bit_signed(packet->angular_divisor);

	int32_t vel_x = pack24bit_signed(packet->angular_velocity_x);
	int32_t vel_y = pack24bit_signed(packet->angular_velocity_y);
	int32_t vel_z = pack24bit_signed(packet->angular_velocity_z);

	gyroscope->axis.x = (float)vel_x * (float)vel_m / (float)vel_d;
	gyroscope->axis.y = (float)vel_y * (float)vel_m / (float)vel_d;
	gyroscope->axis.z = (float)vel_z * (float)vel_m / (float)vel_d;

	int32_t accel_m = pack16bit_signed(packet->acceleration_multiplier);
	int32_t accel_d = pack32bit_signed(packet->acceleration_divisor);

	int32_t accel_x = pack24bit_signed(packet->acceleration_x);
	int32_t accel_y = pack24bit_signed(packet->acceleration_y);
	int32_t accel_z = pack24bit_signed(packet->acceleration_z);

	accelerometer->axis.x = (float)accel_x * (float)accel_m / (float)accel_d;
	accelerometer->axis.y = (float)accel_y * (float)accel_m / (float)accel_d;
	accelerometer->axis.z = (float)accel_z * (float)accel_m / (float)accel_d;

	int32_t magnet_m = pack16bit_signed_swap(packet->magnetic_multiplier);
	int32_t magnet_d = pack32bit_signed_swap(packet->magnetic_divisor);

	int16_t magnet_x = pack16bit_signed_bizarre(packet->magnetic_x);
	int16_t magnet_y = pack16bit_signed_bizarre(packet->magnetic_y);
	int16_t magnet_z = pack16bit_signed_bizarre(packet->magnetic_z);

	magnetometer->axis.x = (float)magnet_x * (float)magnet_m / (float)magnet_d;
	magnetometer->axis.y = (float)magnet_y * (float)magnet_m / (float)magnet_d;
	magnetometer->axis.z = (float)magnet_z * (float)magnet_m / (float)magnet_d;
}

static void align_to_NED(FusionVector *gyroscope, FusionVector *accelerometer, FusionVector *magnetometer)
{
	// raw gyro/accel sensors read west-south-up (NWU sensor, rotated +90 on the z-axis), switch to NED
	*gyroscope = FusionAxesSwap(*gyroscope, FusionAxesAlignmentNYNXNZ);
	*accelerometer = FusionAxesSwap(*accelerometer, FusionAxesAlignmentNYNXNZ);

	// gyro_q_mag leaves this vector in an east-down-north state, switch to NED
	*magnetometer = FusionAxesSwap(*magnetometer, FusionAxesAlignmentPZPXPY);
}

static void apply_calibration(const device_imu_type *device,
															FusionVector *gyroscope,
															FusionVector *accelerometer,
															FusionVector *magnetometer)
{
	FusionMatrix gyroscopeMisalignment;
	FusionVector gyroscopeSensitivity;
	FusionVector gyroscopeOffset;

	FusionMatrix accelerometerMisalignment;
	FusionVector accelerometerSensitivity;
	FusionVector accelerometerOffset;

	FusionMatrix magnetometerMisalignment;
	FusionVector magnetometerSensitivity;
	FusionVector magnetometerOffset;

	FusionMatrix softIronMatrix;
	FusionVector hardIronOffset;

	if (device->calibration)
	{
		gyroscopeMisalignment = device->calibration->gyroscopeMisalignment;
		gyroscopeSensitivity = device->calibration->gyroscopeSensitivity;
		gyroscopeOffset = device->calibration->gyroscopeOffset;

		accelerometerMisalignment = device->calibration->accelerometerMisalignment;
		accelerometerSensitivity = device->calibration->accelerometerSensitivity;
		accelerometerOffset = device->calibration->accelerometerOffset;

		magnetometerMisalignment = device->calibration->magnetometerMisalignment;
		magnetometerSensitivity = device->calibration->magnetometerSensitivity;
		magnetometerOffset = device->calibration->magnetometerOffset;

		softIronMatrix = device->calibration->softIronMatrix;
		hardIronOffset = device->calibration->hardIronOffset;
	}
	else
	{
		gyroscopeMisalignment = FUSION_IDENTITY_MATRIX;
		gyroscopeSensitivity = FUSION_VECTOR_ONES;
		gyroscopeOffset = FUSION_VECTOR_ZERO;

		accelerometerMisalignment = FUSION_IDENTITY_MATRIX;
		accelerometerSensitivity = FUSION_VECTOR_ONES;
		accelerometerOffset = FUSION_VECTOR_ZERO;

		magnetometerMisalignment = FUSION_IDENTITY_MATRIX;
		magnetometerSensitivity = FUSION_VECTOR_ONES;
		magnetometerOffset = FUSION_VECTOR_ZERO;

		softIronMatrix = FUSION_IDENTITY_MATRIX;
		hardIronOffset = FUSION_VECTOR_ZERO;
	}

	gyroscopeOffset = FusionVectorMultiplyScalar(
			gyroscopeOffset,
			FusionRadiansToDegrees(1.0f));

	*gyroscope = FusionCalibrationInertial(
			*gyroscope,
			gyroscopeMisalignment,
			gyroscopeSensitivity,
			gyroscopeOffset);

	*accelerometer = FusionCalibrationInertial(
			*accelerometer,
			accelerometerMisalignment,
			accelerometerSensitivity,
			accelerometerOffset);

	if (device->calibration->using_magnet)
	{
		*magnetometer = FusionCalibrationMagnetic(
				*magnetometer,
				device->calibration->softIronMatrix,
				device->calibration->hardIronOffset);

		*magnetometer = FusionMatrixMultiplyVector(
				magnetometerMisalignment,
				*magnetometer);
	}
}

device_imu_error_type device_imu_clear(device_imu_type *device)
{
	return device_imu_read(device, 10);
}

static void update_magnet_calibration(FusionMatrix *soft_iron_matrix, FusionVector *hard_iron_offsets)
{
	double soft_iron[9] = {0.0};
	double hard_iron[3] = {0.0};
	compute_magnet_calibration(&hard_iron, &soft_iron);
	soft_iron_matrix->element.xx = soft_iron[0];
	soft_iron_matrix->element.xy = soft_iron[1];
	soft_iron_matrix->element.xz = soft_iron[2];
	soft_iron_matrix->element.yx = soft_iron[3];
	soft_iron_matrix->element.yy = soft_iron[4];
	soft_iron_matrix->element.yz = soft_iron[5];
	soft_iron_matrix->element.zx = soft_iron[6];
	soft_iron_matrix->element.zy = soft_iron[7];
	soft_iron_matrix->element.zz = soft_iron[8];
	hard_iron_offsets->axis.x = hard_iron[0];
	hard_iron_offsets->axis.y = hard_iron[1];
	hard_iron_offsets->axis.z = hard_iron[2];
}

device_imu_error_type device_imu_calibrate(device_imu_type *device, uint32_t iterations, bool gyro, bool accel, bool magnet)
{
	if (!device)
	{
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (!device->handle)
	{
		device_imu_error("No handle");
		return DEVICE_IMU_ERROR_NO_HANDLE;
	}

	if (!device->calibration)
	{
		device_imu_error("No calibration allocated");
		return DEVICE_IMU_ERROR_NO_ALLOCATION;
	}

	if (MAX_PACKET_SIZE != sizeof(device_imu_packet_type))
	{
		device_imu_error("Not proper size");
		return DEVICE_IMU_ERROR_WRONG_SIZE;
	}

	device_imu_packet_type packet;
	int transferred;

	bool initialized = false;

	FusionVector cal_gyroscope;
	FusionVector cal_accelerometer;

	FusionMatrix softIronMatrix;
	FusionVector hardIronOffset;

	const float factor = iterations > 0 ? 1.0f / ((float)iterations) : 0.0f;

	FusionVector prev_accel;
	while (iterations > 0)
	{
		memset(&packet, 0, sizeof(device_imu_packet_type));

		transferred = hid_read(
				device->handle,
				(uint8_t *)&packet,
				MAX_PACKET_SIZE);

		if (transferred == -1)
		{
			device_imu_error("Device may be unplugged");
			return DEVICE_IMU_ERROR_UNPLUGGED;
		}

		if (transferred == 0)
		{
			continue;
		}

		if (MAX_PACKET_SIZE != transferred)
		{
			device_imu_error("Unexpected packet size");
			return DEVICE_IMU_ERROR_UNEXPECTED;
		}

		if ((packet.signature[0] != 0x01) || (packet.signature[1] != 0x02))
		{
			continue;
		}

		FusionVector gyroscope;
		FusionVector accelerometer;
		FusionVector magnetometer;

		readIMU_from_packet(&packet, &gyroscope, &accelerometer, &magnetometer);

		if (magnet)
		{
			// the sensor will spam these values initially
			const bool sensor_ready = magnetometer.axis.x != -16.0f || magnetometer.axis.y != -16.0f || magnetometer.axis.z != -16.0f;
			if (sensor_ready)
			{
				collect_magnet(
						magnetometer.axis.x,
						magnetometer.axis.y,
						magnetometer.axis.z,
						(uint32_t)(packet.timestamp / 1e6));
			}
			else
			{
				iterations++;
			}
		}
		iterations--;
	}

	if (magnet)
	{
		device->calibration->using_magnet = true;
		update_magnet_calibration(&device->calibration->softIronMatrix, &device->calibration->hardIronOffset);
	}

	return DEVICE_IMU_ERROR_NO_ERROR;
}

bool device_imu_using_magnet(device_imu_type *device)
{
	if (!device)
	{
		device_imu_error("No device");
		return false;
	}

	if (!device->calibration)
	{
		device_imu_error("No calibration allocated");
		return false;
	}

	return device->calibration->using_magnet;
}

device_imu_error_type device_imu_read(device_imu_type *device, int timeout)
{
	if (!device)
	{
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (!device->handle)
	{
		device_imu_error("No handle");
		return DEVICE_IMU_ERROR_NO_HANDLE;
	}

	if (MAX_PACKET_SIZE != sizeof(device_imu_packet_type))
	{
		device_imu_error("Not proper size");
		return DEVICE_IMU_ERROR_WRONG_SIZE;
	}

	device_imu_packet_type packet;
	memset(&packet, 0, sizeof(device_imu_packet_type));

	int transferred = hid_read_timeout(
			device->handle,
			(uint8_t *)&packet,
			MAX_PACKET_SIZE,
			timeout);

	if (transferred == -1)
	{
		device_imu_error("Device may be unplugged");
		return DEVICE_IMU_ERROR_UNPLUGGED;
	}

	if (transferred == 0)
	{
		return DEVICE_IMU_ERROR_NO_ERROR;
	}

	if (MAX_PACKET_SIZE != transferred)
	{
		device_imu_error("Unexpected packet size");
		return DEVICE_IMU_ERROR_UNEXPECTED;
	}

	const uint64_t timestamp = le64toh(packet.timestamp);

	if ((packet.signature[0] == 0xaa) && (packet.signature[1] == 0x53))
	{
		device_imu_callback(device, timestamp, DEVICE_IMU_EVENT_INIT);
		return DEVICE_IMU_ERROR_NO_ERROR;
	}

	if ((packet.signature[0] != 0x01) || (packet.signature[1] != 0x02))
	{
		device_imu_error("Not matching signature");
		return DEVICE_IMU_ERROR_WRONG_SIGNATURE;
	}

	const uint64_t delta = timestamp - device->last_timestamp;
	const float deltaTime = (float)((double)delta / 1e9);

	device->last_timestamp = timestamp;

	int16_t temperature = pack16bit_signed(packet.temperature);

	// According to the ICM-42688-P datasheet: (offset: 25 °C, sensitivity: 132.48 LSB/°C)
	device->temperature = ((float)temperature) / 132.48f + 25.0f;

	FusionVector gyroscope;
	FusionVector accelerometer;
	FusionVector magnetometer;

	readIMU_from_packet(&packet, &gyroscope, &accelerometer, &magnetometer);
	apply_calibration(device, &gyroscope, &accelerometer, &magnetometer);
	align_to_NED(&gyroscope, &accelerometer, &magnetometer);

	if (device->calibration)
	{
		gyroscope = FusionOffsetUpdate(&device->calibration->gyroscopeFusionOffset, gyroscope);
	}

#ifndef NDEBUG
	static int count = 0;
	if (++count % 500 == 0)
	{
		printf("G: %.3f %.3f %.3f\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);

		printf("A: %.3f %.3f %.3f\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);

		float norm = sqrt(magnetometer.axis.x * magnetometer.axis.x + magnetometer.axis.y * magnetometer.axis.y + magnetometer.axis.z * magnetometer.axis.z);
		printf("M: %.3f %.3f %.3f\n", magnetometer.axis.x / norm, magnetometer.axis.y / norm, magnetometer.axis.z / norm);
	}
#endif

	if (device->ahrs)
	{
		if (isnan(magnetometer.axis.x) || isnan(magnetometer.axis.x) || isnan(magnetometer.axis.x))
		{
			FusionAhrsUpdateNoMagnetometer((FusionAhrs *)device->ahrs, gyroscope, accelerometer, deltaTime);
		}
		else
		{
			if (device->calibration->using_magnet)
			{
				FusionAhrsUpdate((FusionAhrs *)device->ahrs, gyroscope, accelerometer, magnetometer, deltaTime);
			}
			else
			{
				FusionAhrsUpdateNoMagnetometer((FusionAhrs *)device->ahrs, gyroscope, accelerometer, deltaTime);
			}
		}

		const device_imu_quat_type orientation = device_imu_get_orientation(device->ahrs);

		// TODO: fix detection of this case; quat.x as a nan value is only a side-effect of some issue with ahrs or
		//       the gyro/accel/magnet readings
		if (isnan(orientation.x) || isnan(orientation.y) || isnan(orientation.z) || isnan(orientation.w))
		{
			device_imu_error("Invalid orientation reading");
			return DEVICE_IMU_ERROR_INVALID_VALUE;
		}
	}

	device_imu_callback(device, timestamp, DEVICE_IMU_EVENT_UPDATE);
	return DEVICE_IMU_ERROR_NO_ERROR;
}

device_imu_vec3_type device_imu_get_earth_acceleration(const device_imu_ahrs_type *ahrs)
{
	FusionVector acceleration = ahrs ? FusionAhrsGetEarthAcceleration((const FusionAhrs *)ahrs) : FUSION_VECTOR_ZERO;
	device_imu_vec3_type a;
	a.x = acceleration.axis.x;
	a.y = acceleration.axis.y;
	a.z = acceleration.axis.z;
	return a;
}

device_imu_vec3_type device_imu_get_linear_acceleration(const device_imu_ahrs_type *ahrs)
{
	FusionVector acceleration = ahrs ? FusionAhrsGetLinearAcceleration((const FusionAhrs *)ahrs) : FUSION_VECTOR_ZERO;
	device_imu_vec3_type a;
	a.x = acceleration.axis.x;
	a.y = acceleration.axis.y;
	a.z = acceleration.axis.z;
	return a;
}

device_imu_quat_type device_imu_get_orientation(const device_imu_ahrs_type *ahrs)
{
	FusionQuaternion quaternion = ahrs ? FusionAhrsGetQuaternion((const FusionAhrs *)ahrs) : FUSION_IDENTITY_QUATERNION;
	device_imu_quat_type q;
	q.x = quaternion.element.x;
	q.y = quaternion.element.y;
	q.z = quaternion.element.z;
	q.w = quaternion.element.w;
	return q;
}

device_imu_euler_type device_imu_get_euler(device_imu_quat_type quat)
{
	FusionQuaternion quaternion;
	quaternion.element.x = quat.x;
	quaternion.element.y = quat.y;
	quaternion.element.z = quat.z;
	quaternion.element.w = quat.w;
	FusionEuler euler = FusionQuaternionToEuler(quaternion);
	device_imu_euler_type e;
	e.roll = euler.angle.roll;
	e.pitch = euler.angle.pitch;
	e.yaw = euler.angle.yaw;
	return e;
}

device_imu_error_type device_imu_close(device_imu_type *device)
{
	if (!device)
	{
		device_imu_error("No device");
		return DEVICE_IMU_ERROR_NO_DEVICE;
	}

	if (device->calibration)
	{
		free(device->calibration);
	}

	if (device->ahrs)
	{
		free(device->ahrs);
	}

	if (device->serial_number)
	{
		free(device->serial_number);
	}

	if (device->handle)
	{
		hid_close(device->handle);
	}

	memset(device, 0, sizeof(device_imu_type));
	device_exit();

	return DEVICE_IMU_ERROR_NO_ERROR;
}
