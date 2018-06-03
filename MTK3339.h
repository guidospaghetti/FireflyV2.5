
#ifndef MTK3339_H_
#define MTK3339_H_

#include <stdint.h>

//////////////////////////////////////////////////
///
/// GPS Commands
///
//////////////////////////////////////////////////

#define PMTK_ACK						"PMTK001"
#define PMTK_SYS_MSG					"PMTK010"
#define PMTK_TXT_MSG					"PMTK011"
#define PMTK_CMD_HOT_START				"PMTK101"
#define PMTK_CMD_WARM_START				"PMTK102"
#define PMTK_CMD_COLD_START				"PMTK103"
#define PMTK_CMD_FULL_COLD_START		"PMTK104"
#define PMTK_SET_NMEA_UPDATERATE		"PMTK220"
#define PMTK_SET_NMEA_BAUDRATE			"PMTK251"
#define PMTK_API_SET_DGPS_MODE			"PMTK301"
#define PMTK_API_Q_DGPS_MODE			"PMTK401"
#define PMTK_API_DT_DGPS_MODE			"PMTK501"
#define PMTK_API_SET_SBAS_ENABLED		"PMTK313"
#define PMTK_API_Q_SBAS_ENABLED			"PMTK413"
#define PMTK_DT_SBAS_ENABLED			"PMTK513"
#define PMTK_API_SET_NMEA_OUTPUT		"PMTK314"
#define PMTK_API_Q_NMEA_OUTPUT			"PMTK414"
#define PMTK_API_DT_NMEA_OUTPUT			"PMTK514"
#define PMTK_API_SET_SBAS_MODE			"PMTK319"
#define PMTK_API_Q_SBAS_MODE			"PMTK419"
#define PMTK_API_DT_SBAS_MODE			"PMTK519"
#define PMTK_Q_RELEASE					"PMTK605"
#define PMTK_DT_RELEASE					"PMTK705"
#define PMTK_Q_EPO_INFO					"PMTK607"
#define PMTK_DT_EPO_INFO				"PMTK707"
#define PMTK_CMD_CLEAR_EPO				"PMTK127"
#define PMTK_SET_NAV					"PMTK386"
#define PMTK_Q_NAV_THRESHOLD			"PMTK447"
#define PMTK_DT_NAV_THRESHOLD			"PMTK527"
#define PMTK_CMD_STANDBY_MODE			"PMTK161"
#define PMTK_SET_AL_DEE_CFG				"PMTK223"
#define PMTK_CMD_PERIODIC_MODE			"PMTK225"
#define PMTK_CMD_AIC_MODE				"PMTK286"
#define PMTK_CMD_EASY_ENABLE			"PMTK869"
#define PMTK_LOCUS_CONFIG				"PMTK187"
#define PMTK_API_SET_DATUM 				"PMTK330"
#define PMTK_API_Q_DATUM 				"PMTK430"
#define PMTK_API_DT_DATUM				"PMTK530"
#define PMTK_API_SET_SUPPORT_QZSS_NMEA	"PMTK351"
#define PMTK_API_SET_STOP_QZSS			"PMTK352"

#define PMTK_PACKET_END					"\r\n"

#define PMTK_API_SET_NMEA_OUTPUT_RMC_GGA	PMTK_API_SET_NMEA_OUTPUT ",0,1,0,1,0,0,0,0,0,0,0,0,0,0"
#define PMTK_SET_NMEA_UPDATERATE_1000		PMTK_SET_NMEA_UPDATERATE ",1000"

#define PMTK_MAX_NUM_SATELLITES			14
#define PMTK_COMMAND_NAME_LENGTH		5
#define PMTK_CHECKSUM_LENGTH			2
#define PMTK_COMMAND_MAX_NUM_FIELDS		21
#define PMTK_COMMAND_MAX_FIELD_LENGTH	10

#define PMTK_GGA_RETURN					"GPGGA"
#define PMTK_GSA_RETURN					"GPGSA"
#define PMTK_GSV_RETURN					"GPGSV"
#define PMTK_RMC_RETURN					"GPRMC"
#define PMTK_VTG_RETURN					"GPVTG"

#define PMTK_GGA_UTC_TIME				1
#define PMTK_GGA_LATITUDE				2
#define PMTK_GGA_NS_INDICATOR			3
#define PMTK_GGA_LONGITUDE				4
#define PMTK_GGA_EW_INDICATOR			5
#define PMTK_GGA_FIX					6
#define PMTK_GGA_NUM_SATELLITES			7
#define PMTK_GGA_HDOP					8
#define PMTK_GGA_ALTITUDE				9
#define PMTK_GGA_ALTITUDE_UNITS			10
#define PMTK_GGA_GEOIDAL_SEPARATION		11
#define	PMTK_GGA_GS_UNITS				12
#define PMTK_GGA_AGE_OF_DIFF_CORR		13
#define PMTK_GGA_CHECKSUM				14

#define PMTK_GSA_MODE_1					1
#define PMTK_GSA_MODE_2					2
#define PMKT_GSA_SAT_USED_1				3
#define PMKT_GSA_SAT_USED_2				4
#define PMKT_GSA_SAT_USED_3				5
#define PMKT_GSA_SAT_USED_4				6
#define PMKT_GSA_SAT_USED_5				7
#define PMKT_GSA_SAT_USED_6				8
#define PMKT_GSA_SAT_USED_7				9
#define PMKT_GSA_SAT_USED_8				10
#define PMKT_GSA_SAT_USED_9				11
#define PMKT_GSA_SAT_USED_10			12
#define PMKT_GSA_SAT_USED_11			13
#define PMKT_GSA_SAT_USED_12			14
#define PMTK_GSA_PDOP					15
#define PMTK_GSA_HDOP					16
#define PMTK_GSA_VDOP					17
#define PMTK_GSA_CHECKSUM				18

#define PMTK_GSV_NUM_MESSAGES			1
#define PMTK_GSV_MESSAGE_NUM			2
#define PMTK_GSV_SATELLITES_IN_VIEW		3
#define PMTK_GSV_SATELLITE_ID_1			4
#define PMTK_GSV_ELEVATION_1			5
#define PMTK_GSV_AZIMUTH_1				6
#define PMTK_GSV_SNR_1					7
#define PMTK_GSV_SATELLITE_ID_2			8
#define PMTK_GSV_ELEVATION_2			9
#define PMTK_GSV_AZIMUTH_2				10
#define PMTK_GSV_SNR_2					11
#define PMTK_GSV_SATELLITE_ID_3			12
#define PMTK_GSV_ELEVATION_3			13
#define PMTK_GSV_AZIMUTH_3				14
#define PMTK_GSV_SNR_3					15
#define PMTK_GSV_SATELLITE_ID_4			16
#define PMTK_GSV_ELEVATION_4			17
#define PMTK_GSV_AZIMUTH_4				18
#define PMTK_GSV_SNR_4					19
#define PMTK_GSV_CHECKSUM				20

#define PMTK_RMC_UTC_TIME				1
#define PMTK_RMC_STATUS					2
#define PMTK_RMC_LATITUDE				3
#define PMTK_RMC_NS_INDICATOR			4
#define PMTK_RMC_LONGITUDE				5
#define PMTK_RMC_EW_INDICATOR			6
#define PMTK_RMC_SPEED_OVER_GND			7
#define PMTK_RMC_COURSE_OVER_GND		8
#define PMTK_RMC_DATE					9
#define PMTK_RMC_MAG_VAR				10
#define PMTK_RMC_MAG_VAR_DIR			11
#define PMTK_RMC_MODE					12
#define PMTK_RMC_CHECKSUM				13

#define PMTK_VTG_COURSE_1				1
#define PMTK_VTG_REFERENCE_1			2
#define PMTK_VTG_COURSE_2				3
#define PMTK_VTG_REFERENCE_2			4
#define PMTK_VTG_SPEED_1				5
#define PMTK_VTG_SPEED_UNITS_1			6
#define PMTK_VTG_SPEED_2				7
#define PMTK_VTG_SPEED_UNITS_2			8
#define PMTK_VTG_MODE					9
#define PMTK_VTG_CHECKSUM				10

#define PMTK_RMC						0x01
#define PMTK_VTG						0x02
#define PMTK_GGA						0x04
#define PMTK_GSA						0x08
#define PMTK_GSV						0x10

//////////////////////////////////////////////////
///
///	GPS Data Strucures
///
//////////////////////////////////////////////////

typedef struct satellite_t {
	uint8_t satelliteID;	/**< ID of the satellite*/
	uint8_t elevation;		/**< Degrees above horizon*/
	uint8_t azimuth;		/**< Degrees from north*/
	uint8_t SNR;			/**< Signal to noise ratio*/
} satellite_t;

typedef struct time_t {
	uint8_t day;			/**< Current day of the month*/
	uint8_t month;			/**< Current month of the year*/
	uint8_t year;			/**< Current year*/
	float utcTime;			/**< UTC time in the format of hhmmss.sss*/
} time_t;

typedef struct location_t {
	float latitude;			/**< Latitude in the format of ddmm.sss*/
	float longitude;		/**< Longitude in the format of ddmm.sss*/
	float altitude;			/**< Altitude above mean sea level in meters*/
	char NS;				/**< North or south indicator for latitude*/
	char EW;				/**< East of west indicator for longitude*/
} location_t;

typedef struct gpsData_t {
	time_t time;			/**< Current time, contains the date and time*/
	uint8_t fix;			/**< 0 - No fix, 1 - GPS fix, 2 Differential GPS fix*/
	location_t location;	/**< Latitude, longitude, altitude*/
	uint8_t status;			/**< A - Data Valid, V - Data Invalid*/
	float speed;			/**< Speed in knots*/
	float heading;			/**< Direction of movement*/
	float magVar;			/**< Magnitude of magnetic field??*/
	char magVarDir;			/**< Direction of magnetic field??*/
	uint8_t numSatellites;	/**< How many satellites in view*/
	satellite_t allSatellites[PMTK_MAX_NUM_SATELLITES]; /**< All the satellites connected*/
} gpsData_t;

typedef struct gpsParams_t {
	uint16_t updateRate;	/**< Interval of update in milliseconds*/
	uint8_t outputFrames;	/**< Which frames will be outputted from the GPS*/
} gpsParams_t;

/**
 *  @param params How the GPS should be initialized
 *  @brief Initializes the GPS to be ready to output the data requested
 */
void initGPS(gpsParams_t* params);

/**
 *  @param gps Pointer to a gpsData_t structure where the output will be stored
 *  @return 0 - No update, nothing stored 1 - Update, stored in gps
 *  @brief Stores the output of the GPS into gps if there is an update
 */
uint8_t checkForUpdate(gpsData_t* gps);

/**
 *  @param byte The byte received over UART
 *  @brief Function designed to be called by UART Rx interrupt. Simply appends characters 
 *  together if they are a part of a GPS output. Sets flag for if a full command is found
 */
void gpsRxHandler(uint8_t byte);
#endif /* MTK3339_H_ */
