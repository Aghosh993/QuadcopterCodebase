//#include "sys_common.h"

#include "math.h"
#include "NMEA.h"

uint8_t serialPort_read_byte(void)
{
	//sciReceiveByte(scilinREG);
	return 0; // Placeholder for now..
}

/* Flags to indicate status of NMEA message reception state, and message type once a complete message has been acquired: */
volatile uint8_t new_nmea_message_incoming_flag;
volatile uint8_t new_nmea_message_received;
volatile enum nmea_message_type new_nmea_message_received_type;

/* Iterator to indicate position on global buffer, and reception/post-processing buffers: */
volatile uint8_t nmea_buffer_iterator;

volatile uint8_t nmea_message_buffer[NMEA_SENTENCE_MAX_LEN];
volatile uint8_t nmea_post_processing_buffer[NMEA_SENTENCE_MAX_LEN];

//convert ground speed from char to double, the ground speed in format of 000.0 in the unit of knot
/*
double groundToNum(const char* groundSpeed)
{
     double grdDouble;
     
     long grdInt;
     
     grdInt = groundSpeed[4] - '0';
     grdInt += (groundSpeed[2] - '0')*10;
     grdInt += (groundSpeed[1] - '0')*100;
     grdInt += (groundSpeed[0] - '0')*1000;
     
     grdDouble = (double)grdInt;
     grdDouble *= 0.000514444444;

     return grdDouble;   
}
*/

/*
	Convert ground speed from char to double, the ground speed in any format, so long it is COURSE_LEN bytes long
	(the COURSE_LEN	define may be changed in NMEA.h
*/
double groundToNum(const char* groundSpeed)
{
	double retval = 0.0f;
	double integer_retval = 0;
	int i = 0;
	int j = 0;
	for(i=0;i<COURSE_LEN;++i)
	{
		integer_retval += (int)(groundSpeed[i]-'0')*(int)pow(10, COURSE_LEN-i-1);

		if(groundSpeed[i] == '.')
		{
			for(j=i+1;j<COURSE_LEN;++j)
			{
				retval += (double)(groundSpeed[j]-'0')*(double)pow(10, i-j);
			}
			for(j=i-1;j>=0;--j)
			{
				retval += (double)(groundSpeed[j]-'0')*(double)pow(10, i-j-1);
			}
			return retval;
		}
	}
	// If we've gone through the loop above without finding a decimal point, return the parsed integral value:
	return integer_retval;
}

//convert course over ground from char to double, the course over ground in format of 000.0
double courseToNum(const char* courseOG)
{
  double courseDouble;
  
  long courseInt;
  
  courseInt = courseOG[4] - 48;
  courseInt += (courseOG[2] - 48)*10;
  courseInt += (courseOG[1] - 48)*100;
  courseInt += (courseOG[0] - 48)*1000;
  
  courseDouble = (double)courseInt;
  courseDouble /= 57000;
  
  /*courseDouble = (courseOG[0]-48)*100;
  courseDouble += (courseOG[1]-48)*10;
  courseDouble += (courseOG[2]-48)*1;
  courseDouble += (courseOG[4]-48)/10.0;*/
  
  return courseDouble;
}


//convert latitude  from char to double,from minute to degree, the latitude in format of mmmm.ssss
double latToNum(const char* lat)
{
     double latDouble;

     latDouble = (lat[2] -48)*10;
     latDouble += (lat[3] -48);
     latDouble += (lat[5] -48)/10.0;
     latDouble += (lat[6] -48)/100.0;
     latDouble += (lat[7] -48)/1000.0;
     latDouble += (lat[8] -48)/10000.0;
	
     
	// convert from minute to degree
	 latDouble/=60;
	 
	 latDouble+=((lat[0] -48)*10+(lat[1] -48));
    
	 return latDouble;
    
}

//convert longitude  from char to double,from minute to degree, the longitude in format of mmmmm.ssss
double longtToNum(const char* longt)
{
     double longtDouble;
	 
     longtDouble = (longt[3] -48)*10;
     longtDouble += (longt[4] -48);
     longtDouble += (longt[6] -48)/10.0;
     longtDouble += (longt[7] -48)/100.0;
     longtDouble += (longt[8] -48)/1000.0;
     longtDouble += (longt[9] -48)/10000.0;
	 
     // convert from minute to degree
     longtDouble/=60;
     longtDouble+=((longt[0] -48)*100+(longt[1] -48)*10+(longt[2] -48)*1);
         
     return longtDouble;   
}

//convert velocity  from char to double, the velocity in range of -999.9 to 999.9 m/s
double veloToNum(const char* velo, int VeloSize)
{
  double veloDouble;
  int commpos;
  int i,j;
  
  for(i=0;i<VeloSize;i++)
  {
    if(velo[i] == '.')
      commpos = i;
  }
  
  veloDouble = 0;
  
  if(velo[0]=='-')
  {
    for(j = 1;j<commpos;j++)
      veloDouble += (velo[commpos-j]-48)*pow(10,j-1);
    for(j = 1;j<VeloSize-commpos;j++)
      veloDouble += (velo[commpos+j]-48)*pow(10,-j);
    veloDouble = veloDouble*(-1);
  }
  else
  {
    for(j = 1;j<commpos+1;j++)
      veloDouble += (velo[commpos-j]-48)*pow(10,j-1);
    for(j = 1;j<VeloSize-commpos;j++)
      veloDouble += (velo[commpos+j]-48)*pow(10,-j);
  }

  return veloDouble;
  

}

//convert height from char to double, the height in range of -999.9 to 9999.9 m/s
double heightToNum(const char* height, int heightSize)
{

  double heightDouble;
  long heightInt;
  int power[4] = {10, 100, 1000, 10000};
  int index = 1;
  int i;
  
  heightInt = height[heightSize];
  
  if(height[0] != '-')
  {
    for(i = 0; i<heightSize-2; i++)
    {
      heightInt += height[heightSize-2-i]*power[i];
      index *= 10;
    }
  }
  else
  {
    for(i = 0; i<heightSize-3; i++)
    {
      heightInt += height[heightSize-3-i]*power[i];
      index *= 10;
    }
    heightInt = 0-heightInt;
  }
  
  heightDouble = (double)heightInt;
  heightDouble /= index;

  return heightDouble;
  
}

//convert heading from char to double, the heading in range of -999.9 to 999.9 m/s
double headingToNum(const char* heading)
{

  double headingDouble;
  
  if(heading[0]=='-')
  {
     headingDouble = (heading[1] -48)*100;
     headingDouble += (heading[2] -48)*10;
     headingDouble += (heading[3] -48);
	 
     headingDouble += (heading[5] -48)/10.0;
	 headingDouble*=(-1);
  }
  
  else
  {
     headingDouble = (heading[0] -48)*100;
     headingDouble += (heading[1] -48)*10;
     headingDouble += (heading[2] -48)*1;
		 
     headingDouble += (heading[4] -48)/10.0;
	 
  }
  
  return headingDouble; 
}

/*
 * @Description: Initializes all the characters and strings within an nmea_packet struct
 * @Argument(s): Pointer to an nmea_packet struct to be initialized
 * @Return(s):	None
 */
void nmea_packet_struct_init(nmea_packet *packet)
{
	// Initial values: All strings initialized to represent a 0 value when interpreted as numbers:
	strcpy(packet->altitude_mean_sea_level, "0000");
	packet->altitude_units = 'M';	// Default units to SI, so use meters for altitude units
	strcpy(packet->checksum, "00");
	strcpy(packet->dgps_correction_age, "0");
	strcpy(packet->dgps_station_id, "0000");
	strcpy(packet->fix_time_utc, "0000000000");
	strcpy(packet->geoid_separation, "00000");
	packet->geoid_separation_units = '0';
	packet->gps_fix_quality = '0';
	strcpy(packet->horizontal_dilution, "000");
	strcpy(packet->latitude, "000000000");
	packet->latitude_north_south = 'N';			// Default latitude to North
	strcpy(packet->longitude, "0000000000");
	packet->longitude_east_west = 'E';			// Default longitude to East
	strcpy(packet->message_type, "00000");
	strcpy(packet->num_tracked_satellites, "00");

	strcpy(packet->course_magnetic, "000.00");
	packet->reference_magnetic = ' ';
	strcpy(packet->course_true, "000.00");
	packet->reference_true = ' ';
	strcpy(packet->speed_knots, "000.00");
	packet->speed_knots_unit = ' ';
	strcpy(packet->speed_kph, "000.00");
	packet->speed_kph_unit = ' ';

}

enum nmea_parse_status parse_nmea_message(char *nmea_input_sentence, int nmea_sentence_len, enum nmea_message_type msg_type, nmea_packet *output)
{
	/* Verify that we have a valid beginning and end of message:
	 	 If the first character is not a "$", or the third from last character is not a "*",
	 	 we probably have a bad packet:
	 */
	if(nmea_input_sentence[0] != '$' || nmea_input_sentence[nmea_sentence_len-3] != '*')
	{
		return ERR_BAD_PACKET;
	}
	// Local variable declarations:
	int i = 0; // Iterator variables to get around C89 limitations regarding for loops
	int j = 0;

	// Variables to hold the array indices of the start and end of NMEA packet portions:
	int nmea_packet_segment_start = 0;
	int nmea_packet_segment_length = 0;

	/*
	 * Enumerator to hold the state of parsing when handling a GPGGA message.
	 * Each state represents a comma-delimited section of a standard NMEA sentence
	 */

	enum packet_segment
	{
		// Common:
		SEGMENT_MESSAGE_TYPE,
		SEGMENT_CHECKSUM,
		STATE_DONE,

		// For GPGGA sentences:
		SEGMENT_TIME,
		SEGMENT_LATITUDE,
		SEGMENT_LATITUDE_UNITS,
		SEGMENT_LONGITUDE,
		SEGMENT_LONGITUDE_UNITS,
		SEGMENT_POSITION_FIX,
		SEGMENT_NUMBER_SATS,
		SEGMENT_HDOP,
		SEGMENT_ALT_MSL,
		SEGMENT_ALT_MSL_UNITS,
		SEGMENT_GEOID_SEPARATION,
		SEGMENT_GEOID_SEP_UNITS,
		SEGMENT_DGPS_CORRECTION_AGE,
		SEGMENT_DGPS_STATION_ID,

		// For GPVTG sentences:
		SEGMENT_COURSE_T,
		SEGMENT_REFERENCE_T,
		SEGMENT_COURSE_M,
		SEGMENT_REFERENCE_M,
		SEGMENT_SPEED_KNOTS,
		SEGMENT_SPEED_UNITS_KNOTS,
		SEGMENT_SPEED_KPH,
		SEGMENT_SPEED_UNITS_KPH,
		SEGMENT_MODE
	};

	// Initialize the parsing state to the first NMEA packet segment
	enum packet_segment parse_state = SEGMENT_MESSAGE_TYPE;

	// Clean up the nmea_packet structure before parsing into it:
	nmea_packet_struct_init(output);

	/*
	 * Branch depending on the input to the function as to what type of message we are handling.
	 * Only supports the primarily-used navigation GPGGA statements for the time being.
	 */
	switch(msg_type)
	{
	case GPGGA_MESSAGE:
		/*
		 * Initialize nmea_packet_segment_start variable.
		 * Since the message type packet segment begins at the second character (array index 1),
		 * we initialize the nmea_packet_segment_start variable to 1:
		 */
		nmea_packet_segment_start = 1;
		// Skip the first character (the "$" sign), and iterate through the entire input nmea_input_sentence C-string:
		for(i=1;i<nmea_sentence_len;++i)
		{
			// If we get a delimiting character, process the NMEA packet segment:
			if(nmea_input_sentence[i] == ',' || nmea_input_sentence[i] == '*')
			{
				/*
				 * Compute the length of the just-delimited segment by subtracting the index of the segment start
				 * from the current array index-1 and then adding 1:
				 */
				nmea_packet_segment_length = i-nmea_packet_segment_start;
				// Branch based on the portion of the NMEA sentence we're supposed to be processing this time:
				switch(parse_state)
				{
				/*
				 * Copy the contents of the message type portion of the input sentence into
				 * the appropriate C-string in the nmea_packet data structure:
				 */
				 case SEGMENT_MESSAGE_TYPE:
					if(nmea_packet_segment_length <= NMEA_MESSAGE_TYPE_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->message_type[j+NMEA_MESSAGE_TYPE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_TIME;
					break;

				/*
				 * Copy the contents of the UTC time portion of the input sentence into
				 * the appropriate C-string in the nmea_packet data structure:
				 */
				case SEGMENT_TIME:
					if(nmea_packet_segment_length <= NMEA_FIX_TIME_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->fix_time_utc[j+NMEA_FIX_TIME_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_LATITUDE;
					break;
				/*
				 * Copy the contents of the latitude portion of the input sentence into
				 * the appropriate C-string in the nmea_packet data structure:
				 */
				case SEGMENT_LATITUDE:
					if(nmea_packet_segment_length <= NMEA_LATITUDE_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->latitude[j+NMEA_LATITUDE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_LATITUDE_UNITS;
					break;
				/*
				 * Copy the latitude units into the corresponding character within the
				 * nmea_packet data structure:
				 */
				case SEGMENT_LATITUDE_UNITS:
					output->latitude_north_south = nmea_input_sentence[i-1];
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_LONGITUDE;
					break;
				/*
				 * Copy the contents of the longitude portion of the input sentence into
				 * the appropriate C-string in the nmea_packet data structure:
				 */
				case SEGMENT_LONGITUDE:
					if(nmea_packet_segment_length <= NMEA_LONGITUDE_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->longitude[j+NMEA_LONGITUDE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_LONGITUDE_UNITS;
					break;
				/*
				 * Copy the longitude units into the corresponding character within the
				 * nmea_packet data structure:
				 */
				case SEGMENT_LONGITUDE_UNITS:
					output->longitude_east_west = nmea_input_sentence[i-1];
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_POSITION_FIX;
					break;
				/*
				 * Copy the GPS fix quality indicator character into the corresponding character within
				 * the nmea_packet data structure:
				 */
				case SEGMENT_POSITION_FIX:
					output->gps_fix_quality = nmea_input_sentence[i-1];
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_NUMBER_SATS;
					break;
				case SEGMENT_NUMBER_SATS:
					if(nmea_packet_segment_length <= NMEA_NUM_TRACKED_SATS_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->num_tracked_satellites[j+NMEA_NUM_TRACKED_SATS_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_HDOP;
					break;
				case SEGMENT_HDOP:
					if(nmea_packet_segment_length <= NMEA_HORIZONTAL_DOP_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->horizontal_dilution[j+NMEA_HORIZONTAL_DOP_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_ALT_MSL;
					break;
				case SEGMENT_ALT_MSL:
					if(nmea_packet_segment_length <= NMEA_ALTITUDE_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->altitude_mean_sea_level[j+NMEA_ALTITUDE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_ALT_MSL_UNITS;
					break;
				case SEGMENT_ALT_MSL_UNITS:
					output->altitude_units = nmea_input_sentence[i-1];
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_GEOID_SEPARATION;
					break;
				case SEGMENT_GEOID_SEPARATION:
					if(nmea_packet_segment_length <= NMEA_GEOID_SEPARATION_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->geoid_separation[j+NMEA_GEOID_SEPARATION_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_GEOID_SEP_UNITS;
					break;
				case SEGMENT_GEOID_SEP_UNITS:
					output->geoid_separation_units = nmea_input_sentence[i-1];
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_DGPS_CORRECTION_AGE;
					break;
				case SEGMENT_DGPS_CORRECTION_AGE:
					if(nmea_packet_segment_length <= NMEA_DGPS_CORR_AGE_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->dgps_correction_age[j+NMEA_DGPS_CORR_AGE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_DGPS_STATION_ID;
					break;
				case SEGMENT_DGPS_STATION_ID:
					if(nmea_packet_segment_length <= NMEA_DGPS_STATION_ID_LEN)
					{
						for(j=0;j<nmea_packet_segment_length;++j)
						{
							output->dgps_station_id[j+NMEA_DGPS_STATION_ID_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
						}
					}
					nmea_packet_segment_start = i+1;
					parse_state = SEGMENT_CHECKSUM;
					break;
				default:
					break;
				}
			}
		}
		/*
		 * If we have reached the checksum portion of the NMEA sentence, pull out the last two characters of the NMEA
		 * sentence and copy them into the appropriate buffer within the nmea_packet data structure:
		 */
		if(parse_state == SEGMENT_CHECKSUM)
		{
			for(j=0;j<2;++j)
			{
				output->checksum[j] = nmea_input_sentence[nmea_sentence_len-2+j];
			}
			parse_state = STATE_DONE;
		}
		/*
		 * Otherwise, something is wrong with the NMEA sentence provided as an input. Inform the user
		 * and exit with an error enumerator returned:
		 */
		else
		{
			return ERR_BAD_PACKET;
		}
		break;
	case GPVTG_MESSAGE:
//		sciSend(sciREG, 6, (uint8_t *)"Stuff\n");
			/*
			 * Initialize nmea_packet_segment_start variable.
			 * Since the message type packet segment begins at the second character (array index 1),
			 * we initialize the nmea_packet_segment_start variable to 1:
			 */
			nmea_packet_segment_start = 1;
			// Skip the first character (the "$" sign), and iterate through the entire input nmea_input_sentence C-string:
			for(i=1;i<nmea_sentence_len;++i)
			{
				// If we get a delimiting character, process the NMEA packet segment:
				if(nmea_input_sentence[i] == ',' || nmea_input_sentence[i] == '*')
				{
					/*
					 * Compute the length of the just-delimited segment by subtracting the index of the segment start
					 * from the current array index-1 and then adding 1:
					 */
					nmea_packet_segment_length = i-nmea_packet_segment_start;
					// Branch based on the portion of the NMEA sentence we're supposed to be processing this time:
					switch(parse_state)
					{
					case SEGMENT_MESSAGE_TYPE:
						if(nmea_packet_segment_length <= NMEA_MESSAGE_TYPE_LEN)
						{
							for(j=0;j<nmea_packet_segment_length;++j)
							{
								output->message_type[j+NMEA_MESSAGE_TYPE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
							}
						}
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_COURSE_T;
						break;
					case SEGMENT_COURSE_T:
						if(nmea_packet_segment_length <= COURSE_LEN)
						{
							for(j=0;j<nmea_packet_segment_length;++j)
							{
								output->course_true[j+COURSE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
							}
						}
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_REFERENCE_T;
						break;
					case SEGMENT_REFERENCE_T:
						output->reference_true = nmea_input_sentence[i-1];
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_COURSE_M;
						break;
					case SEGMENT_COURSE_M:
						if(nmea_packet_segment_length <= COURSE_LEN)
						{
							for(j=0;j<nmea_packet_segment_length;++j)
							{
								output->course_magnetic[j+COURSE_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
							}
						}
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_REFERENCE_M;
						break;
					case SEGMENT_REFERENCE_M:
						output->reference_magnetic = nmea_input_sentence[i-1];
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_SPEED_KNOTS;
						break;
					case SEGMENT_SPEED_KNOTS:
						if(nmea_packet_segment_length <= SPEED_KNOTS_LEN)
						{
							for(j=0;j<nmea_packet_segment_length;++j)
							{
								output->speed_knots[j+SPEED_KNOTS_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
							}
						}
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_SPEED_UNITS_KNOTS;
						break;
					case SEGMENT_SPEED_UNITS_KNOTS:
						output->speed_knots_unit = nmea_input_sentence[i-1];
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_SPEED_KPH;
						break;
					case SEGMENT_SPEED_KPH:
						if(nmea_packet_segment_length <= SPEED_KPH_LEN)
						{
							for(j=0;j<nmea_packet_segment_length;++j)
							{
								output->speed_kph[j+SPEED_KPH_LEN-nmea_packet_segment_length] = nmea_input_sentence[i-nmea_packet_segment_length+j];
							}
						}
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_SPEED_UNITS_KPH;
						break;
					case SEGMENT_SPEED_UNITS_KPH:
						output->speed_kph_unit = nmea_input_sentence[i-1];
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_MODE;
						break;
					case SEGMENT_MODE:
						output->mode = nmea_input_sentence[i-1];
						nmea_packet_segment_start = i+1;
						parse_state = SEGMENT_CHECKSUM;
						break;
					}
				}
			}
			/*
			 * If we have reached the checksum portion of the NMEA sentence, pull out the last two characters of the NMEA
			 * sentence and copy them into the appropriate buffer within the nmea_packet data structure:
			 */
			if(parse_state == SEGMENT_CHECKSUM)
			{
				for(j=0;j<2;++j)
				{
					output->checksum[j] = nmea_input_sentence[nmea_sentence_len-2+j];
				}
				parse_state = STATE_DONE;
			}
			/*
			 * Otherwise, something is wrong with the NMEA sentence provided as an input. Inform the user
			 * and exit with an error enumerator returned:
			 */
			else
			{
				return ERR_BAD_PACKET;
			}
			break;
		break;
	/*
	 * For now, the only supported message type is a GPGGA or GPVTG message, so otherwise
	 * return ERR_UNSUPPORTED_MESSAGE:
	 */
	default:
		return ERR_UNSUPPORTED_MESSAGE; // No break necessary after return statement.
	}
	/*
	 * If we have reached this point, parsing is successful. Inform the user
	 * of the same by returning the correct enumerator:
	 */
	return PARSE_SUCCESS;
}

void UART_gps_callback(void)
{
	uint8_t i = 0U;
	uint8_t recvByte = 0U;

	// Read incoming byte and clear the data register:
	recvByte = serialPort_read_byte();

	/*
	 * Check if we have a start of NMEA sentence character:
	 */
	if(recvByte == '$')
	{
		/*
		 * If whatever we have so far is considered old, look for the beginning of an NMEA sentence first:
		 */
		if(new_nmea_message_received == 0U)
		{
			/*
			 * We are now at the beginning of a new NMEA sentence,
			 * so set the appropriate flag:
			 */
			new_nmea_message_incoming_flag = 1U;
			/*
			 * Initialize the buffer index variable to the beginning of the buffer
			 * and place the initial character in the buffer. Finally, increment the
			 * NMEA message buffer array index:
			 */
			nmea_buffer_iterator = 0U;
			nmea_message_buffer[nmea_buffer_iterator] = recvByte;
			++nmea_buffer_iterator;
		}
	}
	else
	{
		/*
		 * If we know we are in the process of receiving an NMEA sentence, and what we already
		 * have in the message buffer has been marked as old:
		 */
		if(new_nmea_message_incoming_flag == 1U && new_nmea_message_received == 0U)
		{
			/*
			 * A newline character indicates end of NMEA sentence.
			 * If we see this, we start looking at the NMEA sentence recorded so far to see if
			 * it contains information useful to us:
			 */
			if(recvByte == '\n')
			{
				/* We are no longer in the process of receiving an NMEA sentence: */
				new_nmea_message_incoming_flag = 0U;

				/* Get the type of NMEA sentence we've just received: */
				new_nmea_message_received_type = get_nmea_sentence_type((char *)nmea_message_buffer);

				/* If we have a GPGGA (navigation) or GPVTG (course) message, we set the appropriate flag to initiate post-processing: */
				if(new_nmea_message_received_type == GPGGA_MESSAGE || new_nmea_message_received_type == GPVTG_MESSAGE)
				{
					new_nmea_message_received = 1U;

					/*
					 * Decrement nmea_buffer_iterator by 1 since we don't care about the carriage return on the end
					 * of the buffer:
					 */
					--nmea_buffer_iterator;
					for(i=0;i<=nmea_buffer_iterator;++i)
					{
						nmea_post_processing_buffer[i] = nmea_message_buffer[i];
					}
				}
			}
			else
			{
				/* Do some bounds checking to make sure we don't overrun the statically-allocated buffer: */
				if(nmea_buffer_iterator < NMEA_SENTENCE_MAX_LEN)
				{
					/* Put the received byte in the appropriate place within the NMEA buffer: */
					nmea_message_buffer[nmea_buffer_iterator] = recvByte;
					/* Increment the buffer position iterator: */
					++nmea_buffer_iterator;
				}
			}
		}
	}
}

uint8_t gps_new_data_available(void)
{
	return new_nmea_message_received;
}
enum gps_read_status GPS_get_data_blocking(gps_data *dataBuffer, enum nmea_message_type desired_message)
{
	uint8_t i = 0U;
	uint8_t j = 0U;

	nmea_packet parsed_data;

	for(i=0;i<GPS_READ_MAX_CYCLES;++i)
	{
		if(new_nmea_message_received == 1U)
		{
			switch(desired_message)
			{
			case GPGGA_MESSAGE:
				if(parse_nmea_message((char *)nmea_post_processing_buffer, nmea_buffer_iterator, desired_message,&parsed_data) == PARSE_SUCCESS)
				{
					dataBuffer->latitude = latToNum(parsed_data.latitude);
					dataBuffer->latitude_direction = parsed_data.latitude_north_south;

					dataBuffer->longitude = latToNum(parsed_data.longitude);
					dataBuffer->longitude_direction = parsed_data.longitude_east_west;

					dataBuffer->utc_hour[0] = parsed_data.fix_time_utc[0];
					dataBuffer->utc_hour[1] = parsed_data.fix_time_utc[1];

					dataBuffer->utc_min[0] = parsed_data.fix_time_utc[2];
					dataBuffer->utc_min[1] = parsed_data.fix_time_utc[3];

					for(j=0U;j<6U;++j)
					{
						dataBuffer->utc_sec[j] = parsed_data.fix_time_utc[j+4];
					}

					dataBuffer->fix_quality = parsed_data.gps_fix_quality;
					dataBuffer->num_tracked_satellites = ((uint8_t)parsed_data.num_tracked_satellites[0]-'0')*10
							+ ((uint8_t)parsed_data.num_tracked_satellites[1]-'0');
					dataBuffer->altitude = heightToNum(parsed_data.altitude_mean_sea_level, NMEA_ALTITUDE_LEN);
					dataBuffer->altitude_units = parsed_data.altitude_units;

					new_nmea_message_received = 0U;
					return GPGGA_READ_SUCCESS;
				}
				else
				{
					return ERR_PARSE_ERR;
				}
			case GPVTG_MESSAGE:
				if(parse_nmea_message((char *)nmea_post_processing_buffer, nmea_buffer_iterator, desired_message,&parsed_data) == PARSE_SUCCESS)
				{
					dataBuffer->course_magnetic = groundToNum(parsed_data.course_magnetic);
					dataBuffer->course_true = groundToNum(parsed_data.course_true);

					dataBuffer->speed_knots = groundToNum(parsed_data.speed_knots);
					dataBuffer->speed_kph =groundToNum(parsed_data.speed_kph);

					new_nmea_message_received = 0U;
					return GPVTG_READ_SUCCESS;
				}
				else
				{
					return ERR_PARSE_ERR;
				}
			}
			/*
			if(parse_nmea_message((char *)nmea_post_processing_buffer, nmea_buffer_iterator, GPGGA_MESSAGE,&parsed_data) == PARSE_SUCCESS)
			{
				dataBuffer->latitude = latToNum(parsed_data.latitude);
				dataBuffer->latitude_direction = parsed_data.latitude_north_south;

				dataBuffer->longitude = latToNum(parsed_data.longitude);
				dataBuffer->longitude_direction = parsed_data.longitude_east_west;

				dataBuffer->utc_hour[0] = parsed_data.fix_time_utc[0];
				dataBuffer->utc_hour[1] = parsed_data.fix_time_utc[1];

				dataBuffer->utc_min[0] = parsed_data.fix_time_utc[2];
				dataBuffer->utc_min[1] = parsed_data.fix_time_utc[3];

				for(j=0U;j<6U;++j)
				{
					dataBuffer->utc_sec[j] = parsed_data.fix_time_utc[j+4];
				}

				dataBuffer->fix_quality = parsed_data.gps_fix_quality;
				dataBuffer->num_tracked_satellites = ((uint8_t)parsed_data.num_tracked_satellites[0]-'0')*10
						+ ((uint8_t)parsed_data.num_tracked_satellites[1]-'0');
				dataBuffer->altitude = heightToNum(parsed_data.altitude_mean_sea_level, NMEA_ALTITUDE_LEN);
				dataBuffer->altitude_units = parsed_data.altitude_units;

				new_nmea_message_received = 0U;
				return GPGGA_READ_SUCCESS;
			}
			if(parse_nmea_message((char *)nmea_post_processing_buffer, nmea_buffer_iterator, GPVTG_MESSAGE,&parsed_data) == PARSE_SUCCESS)
			{

				dataBuffer->course_magnetic = groundToNum(parsed_data.course_magnetic);
				dataBuffer->course_true = groundToNum(parsed_data.course_true);

				dataBuffer->speed_knots = groundToNum(parsed_data.speed_knots);
				dataBuffer->speed_kph =groundToNum(parsed_data.speed_kph);

				new_nmea_message_received = 0U;
				return GPVTG_READ_SUCCESS;
			}

			else
			{
				return ERR_PARSE_ERR;
			}*/
		}
	}
	return ERR_TIMEOUT;
}

enum gps_read_status GPS_get_data(gps_data *dataBuffer, enum nmea_message_type desired_message)
{
	uint8_t j = 0U;

	nmea_packet parsed_data;

	switch(desired_message)
	{
	case GPGGA_MESSAGE:
		if(parse_nmea_message((char *)nmea_post_processing_buffer, nmea_buffer_iterator, desired_message,&parsed_data) == PARSE_SUCCESS)
		{
			dataBuffer->latitude = latToNum(parsed_data.latitude);
			dataBuffer->latitude_direction = parsed_data.latitude_north_south;

			dataBuffer->longitude = latToNum(parsed_data.longitude);
			dataBuffer->longitude_direction = parsed_data.longitude_east_west;

			dataBuffer->utc_hour[0] = parsed_data.fix_time_utc[0];
			dataBuffer->utc_hour[1] = parsed_data.fix_time_utc[1];

			dataBuffer->utc_min[0] = parsed_data.fix_time_utc[2];
			dataBuffer->utc_min[1] = parsed_data.fix_time_utc[3];

			for(j=0U;j<6U;++j)
			{
				dataBuffer->utc_sec[j] = parsed_data.fix_time_utc[j+4];
			}

			dataBuffer->fix_quality = parsed_data.gps_fix_quality;
			dataBuffer->num_tracked_satellites = ((uint8_t)parsed_data.num_tracked_satellites[0]-'0')*10
					+ ((uint8_t)parsed_data.num_tracked_satellites[1]-'0');
			dataBuffer->altitude = heightToNum(parsed_data.altitude_mean_sea_level, NMEA_ALTITUDE_LEN);
			dataBuffer->altitude_units = parsed_data.altitude_units;

			new_nmea_message_received = 0U;
			return GPGGA_READ_SUCCESS;
		}
	case GPVTG_MESSAGE:
		if(parse_nmea_message((char *)nmea_post_processing_buffer, nmea_buffer_iterator, desired_message,&parsed_data) == PARSE_SUCCESS)
		{
			dataBuffer->course_magnetic = groundToNum(parsed_data.course_magnetic);
			dataBuffer->course_true = groundToNum(parsed_data.course_true);

			dataBuffer->speed_knots = groundToNum(parsed_data.speed_knots);
			dataBuffer->speed_kph =groundToNum(parsed_data.speed_kph);

			new_nmea_message_received = 0U;
			return GPVTG_READ_SUCCESS;
		}
	}
	return ERR_PARSE_ERR;
}

void GPS_globals_init(void)
{
	new_nmea_message_incoming_flag = 0U;
	new_nmea_message_received = 0U;
	nmea_buffer_iterator = 0U;
}

/*
 * Function to find the length of an NMEA sentence
 * @Argument(s): An NMEA sentence that is terminated by the asterisk character and two-digit hexadecimal checksum
 * @Return: Either ERR_NO_TERMINATING_CHARACTER_FOUND if no asterisk preceding the checksum is found,
 * 				or a positive value of the length of the buffer provided as the argument, from the beginning "$"
 * 				character to the last digit of the two hexadecimal-digit checksum.
 */

int get_nmea_sentence_length(char *nmea_sentence_buffer)
{
	int i = 0;
	while(nmea_sentence_buffer[i] != '*')
	{
		if(i>NMEA_SENTENCE_MAX_LEN)
		{
			return ERR_NO_TERMINATING_CHARACTER_FOUND;
		}
		++i;
	}
	return i+2+1; // Add 2 for the checksum bytes, and 1 to index the length from 1 instead of 0
}

/*
 * Function to find the type of NMEA message received
 * @Argument(s): An NMEA sentence that is terminated by the asterisk character and two-digit hexadecimal checksum
 * @Return: Enumerator specifying the type of message contained by the buffer
 * 			Can be: GPGGA_MESSAGE, GPGSV_MESSAGE, GPGLL_MESSAGE or GPRMC_MESSAGE if successful
 * 			- or - NMEA_UNKNOWN_MESSAGE or PROCESSING_ERR if there is a failure
 */

enum nmea_message_type get_nmea_sentence_type(char *nmea_buffer)
{
	/*
	 * Check if message is valid:
	 */
	switch(nmea_buffer[1])
	{
	case 'G':
		/*
		 * Check 4th character and branch accordingly:
		 */
		switch(nmea_buffer[3])
		{
		case 'G':
			/*
			 * Check 5th character and branch accordingly:
			 */
			switch(nmea_buffer[4])
			{
			case 'G':
				/*
				 * Check the last NMEA message type character:
				 */
				switch(nmea_buffer[5])
				{
				/*
				 * If it's an A and we're here, we have a GPGGA message
				 * Otherwise, it's an unknown message type:
				 */
				case 'A':
					return GPGGA_MESSAGE;
				default:
					return NMEA_UNKNOWN_MESSAGE;
				}
				// break; // We cannot get here
			case 'S':
				switch(nmea_buffer[5])
				{
				case 'V':
					return GPGSV_MESSAGE;
				default:
					return NMEA_UNKNOWN_MESSAGE;
				}
				// break; // We cannot get here
			default:
				return NMEA_UNKNOWN_MESSAGE;
			}
			// break; // We cannot get here
		case 'R':
			switch(nmea_buffer[4])
			{
			case 'M':
				switch(nmea_buffer[5])
				{
				case 'C':
					return GPRMC_MESSAGE;
				default:
					return NMEA_UNKNOWN_MESSAGE;
				}
				// break; // We cannot get here
			default:
				return NMEA_UNKNOWN_MESSAGE;
			}
			// break; // We cannot get here
		}
		break;
	default:
		return NMEA_UNKNOWN_MESSAGE;
	}
	/*
	 * This should never be returned:
	 */
	return PROCESSING_ERR;
}
