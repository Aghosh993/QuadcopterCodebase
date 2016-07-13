#ifndef NMEA_1_H
#define NMEA_1_H 1

#include <stdio.h>
#include <string.h>
#include <stdint.h>

struct GPSData
{
  float latNum;
  float longtNum;
  float groundSpeedNum;
  float headingNum;
  float heightNum;
};

enum nmea_fix_quality
{
	NMEA_FIX_INVALID,
	NMEA_FIX_GPS_SPS,
	NMEA_FIX_GPS_DGPS,
	NMEA_FIX_GPS_PPS,
	NMEA_FIX_GPS_REALTIME_KINETIC,
	NMEA_FIX_GPS_FLOAT_RTK,
	NMEA_FIX_GPS_DEADRECKON,
	NMEA_FIX_GPS_MANUAL_INPUT,
	NMEA_FIX_GPS_SIMULATION
};

#define NMEA_MESSAGE_TYPE_LEN 		5
#define NMEA_FIX_TIME_LEN 			10
#define NMEA_LATITUDE_LEN 			9
#define NMEA_LONGITUDE_LEN 			10
#define NMEA_NUM_TRACKED_SATS_LEN 	2
#define NMEA_HORIZONTAL_DOP_LEN 	3
#define NMEA_ALTITUDE_LEN 			4 // Leave it there for the time being... need to look at GPS module data dumps to confirm...
#define NMEA_GEOID_SEPARATION_LEN 	5 // Same comment as above...
#define NMEA_DGPS_CORR_AGE_LEN 		1 // Appears to not be actually used since we're not using DGPS...?
#define NMEA_DGPS_STATION_ID_LEN 	4 // Not too sure...
#define NMEA_CHECKSUM_LEN 			2 // Asterisk character + 2-digit hex

#define COURSE_LEN					6
#define SPEED_KNOTS_LEN				6
#define SPEED_KPH_LEN				6


typedef struct
{
	// Common contents:
	char message_type[NMEA_MESSAGE_TYPE_LEN];
	char checksum[NMEA_CHECKSUM_LEN];

	// GPGGA sentence-specific contents:
	char fix_time_utc[NMEA_FIX_TIME_LEN];
	char latitude[NMEA_LATITUDE_LEN];
	char latitude_north_south;
	char longitude[NMEA_LONGITUDE_LEN];
	char longitude_east_west;
	char gps_fix_quality;
	char num_tracked_satellites[NMEA_NUM_TRACKED_SATS_LEN];
	char horizontal_dilution[NMEA_HORIZONTAL_DOP_LEN];
	char altitude_mean_sea_level[NMEA_ALTITUDE_LEN];
	char altitude_units;
	char geoid_separation[NMEA_GEOID_SEPARATION_LEN];
	char geoid_separation_units;
	char dgps_correction_age[NMEA_DGPS_CORR_AGE_LEN];
	char dgps_station_id[NMEA_DGPS_STATION_ID_LEN];

	// GPVTG sentence-specific contents:
	char course_true[COURSE_LEN];
	char reference_true;
	char course_magnetic[COURSE_LEN];
	char reference_magnetic;
	char speed_knots[SPEED_KNOTS_LEN];
	char speed_knots_unit;
	char speed_kph[SPEED_KPH_LEN];
	char speed_kph_unit;
	char mode;
} nmea_packet;

typedef struct
{
	// GPGGA data:

	double latitude;
	char latitude_direction;
	double longitude;
	char longitude_direction;

	char utc_hour[2];
	char utc_min[2];
	char utc_sec[6];

	char fix_quality;
	uint8_t num_tracked_satellites;
	double altitude;
	char altitude_units;

	// GPVTG data:
	double course_magnetic;
	double course_true;

	double speed_knots;
	double speed_kph;
} gps_data;

enum nmea_message_type
{
	GPGGA_MESSAGE,
	GPVTG_MESSAGE,
	GPGSV_MESSAGE,
	GPGLL_MESSAGE,
	GPRMC_MESSAGE,
	NMEA_UNKNOWN_MESSAGE,
	PROCESSING_ERR
};

enum nmea_parse_status
{
	PARSE_SUCCESS,
	ERR_UNSUPPORTED_MESSAGE,
	ERR_BAD_PACKET
};

enum gps_read_status
{
	GPGGA_READ_SUCCESS,
	GPVTG_READ_SUCCESS,
	ERR_PARSE_ERR,
	ERR_TIMEOUT
};

#define	ERR_NO_TERMINATING_CHARACTER_FOUND -1
#define NMEA_SENTENCE_MAX_LEN				177

#define GPS_READ_MAX_CYCLES					50000000 	// GPS read function timeout parameter.
														// 50 million cycles should approximate 1 second... tune this
														// parameter based on the target processor's clock and cycle times

/* Global Variable(s): */

/* Function prototypes: */

double groundToNum(const char* groudSpeed);
double courseToNum(const char* courseOG);
double latToNum(const char* lat);
double longtToNum(const char* longt);
double veloToNum(const char* velo, int VeloSize);
double heightToNum(const char* height, int heightSize);

void nmea_packet_struct_init(nmea_packet *packet);
enum nmea_parse_status parse_nmea_message(char *nmea_input_sentence, int nmea_sentence_len, enum nmea_message_type msg_type, nmea_packet *output);
int get_nmea_sentence_length(char *nmea_sentence_buffer);
enum nmea_message_type get_nmea_sentence_type(char *nmea_buffer);

void GPS_globals_init(void);
void UART_gps_callback(void);
enum gps_read_status GPS_get_data_blocking(gps_data *dataBuffer, enum nmea_message_type desired_message);
enum gps_read_status GPS_get_data(gps_data *dataBuffer, enum nmea_message_type desired_message);
uint8_t gps_new_data_available(void);

// Macro to make the use of serial port generic:

uint8_t serialPort_read_byte(void);

#endif
