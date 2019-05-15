#ifndef _GPS_HANDLER_
#define _GPS_HANDLER_

#include <Arduino.h>
#include <HardwareSerial.h>

#define GPS_BUFFER_OVERFLOW -10
#define GPS_MESSAGE_GARBAGE -11
#define GPS_MESSAGE_MAX_LENGTH 128
#define GPS_CHECKSUM_MISMATCH -15
#define GPS_VALID_FIX_NEVER_MADE -30

#define GPS_DEBUG
#define GPS_MAX_ACCEPTABLE_LATENCY 5000

/*
Simple GPS parser
Only uses GPGGA data tags
*/

class GPS_Handler{
public:

  enum GPS_FIX_QUALITY{
    FIX_INVALID = 0,
    GPS_FIX = 1,
    D_GPS_FIX = 2
  };

  enum GPS_IDENTIFIER{
    GGA, GSA
  };

  struct GPS_Time{
    uint8_t hours = 0;
    uint8_t minutes = 0;
    uint8_t seconds = 0;
    uint8_t centiseconds = 0;
    bool valid = false;
  };

  struct GPS_Data{
    char *raw_string[GPS_MESSAGE_MAX_LENGTH];
    u_int16_t length = 0;
    float latitude = 0;
    float longitude = 0;
    float altitude = 0;
    GPS_Time time_stamp;
    uint8_t satellites = 0;
    float dilution = 0;
    GPS_FIX_QUALITY fix_quality = FIX_INVALID;
  };

  /**
    Initializes the class, sets up for receiving and parsing gps messages

    @param serial object which the serial device is connected to. Should already be initialized the the gps communication rate
    and any special communication styles.
    @return an int code 0 being success
    and < 0 a corresponding error code
  */
  int init(HardwareSerial *serial);

  /**
    Updates by checking for new GPS data and parsing said data

    @return an int code 0 general success but no new messages
    < 0 a corresponding error code,
    1 being a new message or gps data available
  */
  int update();
  int getLatestData(GPS_Data *data);
  int getLastValidData(GPS_Data *data);
  long getTimeFromLastValidMessage();
  bool isConnected();

private:
  HardwareSerial *gps_communication;
  GPS_Data last_data;
  GPS_Data last_valid_data;
  char serial_buffer[GPS_MESSAGE_MAX_LENGTH];
  uint8_t buffer_index = 0;
  long lastValidMessageMillis = 0;

  /**
    Reads the available chars, populates the running message buffer

    @return an int code with 1 being a new message available,
    < 0 a corresponding error code,
    0 being no new messages
  */
  int readGps();

  /**
    Parses the new gps message, discards all non GPGGA messages for now. parses for context and updates
    new GPS_Data object

    @param newData object to populate with new data
    @return an int code with 1 being a successful message parsing
    < 0 a corresponding error code,
    0 being an unused gps message
  */
  int handleNewGpsMessage(GPS_Data *newData);

  /**
    Helper function for finding index of a char in an array

    @param buffer char buffer to look through
    @param length max length of the char buffer
    @param toLookFor char to look for
    @param startIndex optional first index (inclusive) to start looking for toLookFor char
    @return an int code corresponding to the zero index location or
    < 0 corresponding not found
  */
  int findChar(char *buffer, uint length, char toLookFor, uint startIndex = 0);

  /**
    Helper function for finding index of a char array in an array

    @param buffer char buffer to look through
    @param length max length of the char buffer
    @param toLookFor char array to look for
    @param look_length length of char array to look for
    @param startIndex optional first index (inclusive) to start looking for toLookFor char array
    @return an int code corresponding to the zero index location or
    < 0 corresponding not found
  */
  int findStr(char *buffer, uint length, char *toLookFor, uint look_length, uint startIndex = 0);

  /**
    Parse the utc time from the gps string

    @param buffer char buffer to look through
    @param length max length of the char buffer
    @param time object to fill
    @return an int code corresponding to 0 being success
    < 0 failure
  */
  int parseTime(char *buffer, uint length, GPS_Time *time);

  /**
    Parse the fix from the gps string

    @param buffer char buffer to look through
    @param length max length of the char buffer
    @param fix object to fill
    @return an int code corresponding to 0 being success
    < 0 failure
  */
  int parseFix(char *buffer, uint length, GPS_FIX_QUALITY *fix);

  int parseSatellites(char *buffer, uint length, uint8_t *number);

  int parseHorizontalDilution(char *buffer, uint length, float *dilution);

  int parseAntennaAltitude(char *buffer, uint length, float *altitude);

  int parseChecksum(char *buffer, uint length, int *altitude);

  int computeChecksum(char *buffer, uint length, int *checksum);

  int copyGpsData(GPS_Data *target, GPS_Data *source);

  int copyGpsTimeData(GPS_Time *target, GPS_Time *source);

  int parseLatitude(char *latBuffer, uint length, char dir, float *latitude);

  int parseLongitude(char *lonBuffer, uint length, char dir, float *longitude);

};


#endif
