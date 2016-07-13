#ifndef LIDAR_LITE_V1_H
#define LIDAR_LITE_V1_H		1

#include <lidar_lite_v1_hal.h>

#define USE_TIMEOUTS	1

#ifdef USE_TIMEOUTS
	typedef enum {
		STATE_IDLE,
		STATE_INITIATED_MEASUREMENT,
		STATE_TRIGGER_TIMEOUT,
		STATE_MEASUREMENT_TIMEOUT
	} lidar_lite_measurement_status;
#endif
#ifndef USE_TIMEOUTS
	typedef enum {
		STATE_IDLE,
		STATE_INITIATED_MEASUREMENT
	} lidar_lite_measurement_status;
#endif
typedef struct {
	lidar_lite_measurement_status sensor_measurement_state;
	float last_measured_distance;
} lidar_lite_sensor_instance;

#define LIDAR_LITE_MEASUREMENT_INTERVAL_MILLIS	35U

void lidar_lite_initialize_instance(lidar_lite_sensor_instance *inst);
void lidar_lite_propagate_state_machine(lidar_lite_sensor_instance *inst);

float lidar_lite_get_latest_measurement(lidar_lite_sensor_instance inst);

int lidar_lite_trigger_measurement(void);
uint16_t lidar_lite_get_raw_data(void);

void lidar_lite_reset_data_bus(void);

#endif