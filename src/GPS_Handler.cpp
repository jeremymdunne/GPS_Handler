#include <GPS_Handler.h>

bool GPS_Handler::isConnected(){
  long latency = getTimeFromLastValidMessage();
  if(latency >= 0 && latency <= GPS_MAX_ACCEPTABLE_LATENCY){
    return true;
  }
  return false;
}

int GPS_Handler::update(){
  //assume we may need to process multiple messages
  bool stop = false;
  while(!stop){
    int status = readGps();
    if(status < 0) return status; //error code received!
    if(status > 0){
      //just print out the message for now
      #ifdef GPS_DEBUG
        Serial.print("\nNew Gps Message: ");
        for(uint i = 0; i < buffer_index; i ++){
          Serial.print(serial_buffer[i]);
        }
        Serial.println();
      #endif
      //parse the message
      GPS_Data newData;
      status = handleNewGpsMessage(&newData);
      if(status < 0) return status;
      if(status > 0){
        copyGpsData(&last_data, &newData);
        //validate message and then store results in the local data if applicable
        //modes: data is non-valid fix and previous data was valid: don't store
        if((newData.fix_quality != FIX_INVALID)){
          //store it
          copyGpsData(&last_valid_data, &newData);
          lastValidMessageMillis = millis();
        }
        if((newData.fix_quality == FIX_INVALID) && (last_valid_data.fix_quality == FIX_INVALID)){
          //store it
          copyGpsData(&last_valid_data, &newData);
          lastValidMessageMillis = millis();
        }
      }
      buffer_index = 0;
      return 0;
    }
  }
  return 0;
}

long GPS_Handler::getTimeFromLastValidMessage(){
  if(lastValidMessageMillis == 0) return GPS_VALID_FIX_NEVER_MADE;
  return millis()-lastValidMessageMillis;
}

int GPS_Handler::handleNewGpsMessage(GPS_Data *newData){
  //first check for message completeness if it is a GPGGA message
  //look for both a '$' and a '*'
  int start = findChar(&serial_buffer[0], buffer_index, '$');
  int end = findChar(&serial_buffer[0], buffer_index, '*');
  if(start < 0 || end < 0){
    //error!
    #ifdef GPS_DEBUG
      Serial.println("Invalid location of \'$\' and/or \'*\'");
    #endif
    return GPS_MESSAGE_GARBAGE;
  }
  //next, check if GPGGA
  char toLookFor[] = "GPGGA";
  int GPGGA_start = findStr(&serial_buffer[0], buffer_index, &toLookFor[0], 5);
  if(GPGGA_start < 0) return 0;
  //lets start parsing!
  //first is the GPPGA stamp, diregaurd
  //start an array of comma locations...
  #ifdef GPS_DEBUG
    Serial.println("New GPGGA line found, parsing!");
  #endif
  int commaLocations[14];
  commaLocations[0] = findChar(&serial_buffer[0], buffer_index, ',');
  if(commaLocations[0] < 0){
    #ifdef GPS_DEBUG
      Serial.println("0 Comma location failed!");
      #endif
    return GPS_MESSAGE_GARBAGE;
  }
  for(uint i = 1; i < 14; i ++){
    commaLocations[i] = findChar(&serial_buffer[0], buffer_index, ',',commaLocations[i-1] + 1);
    if(commaLocations[i] < 0){
      #ifdef GPS_DEBUG
        Serial.println(String(i) + " Comma location failed!");
      #endif
      return GPS_MESSAGE_GARBAGE;
    }
  }
  //check the checksums real quick
  int checksum = 0;
  int status = parseChecksum(&serial_buffer[end + 1],buffer_index - end - 1, &checksum);
  //compute what the checksum should be
  int calculatedChecksum = 0;
  status = computeChecksum(&serial_buffer[1], end-1, &calculatedChecksum);
  if(status < 0) return status;
  if(calculatedChecksum != checksum){
    #ifdef GPS_DEBUG
      Serial.println("Checksum mismatch, expected: " + String(calculatedChecksum) + " received: " + String(checksum));
    #endif
    return GPS_CHECKSUM_MISMATCH;
  }
  //go through and assign and do any extra parsing of the codes
  status = parseTime(&serial_buffer[commaLocations[0]], commaLocations[1] - commaLocations[0], &newData->time_stamp);
  if(status < 0) return status;
  //next do Latitude
  status = parseLatitude(&serial_buffer[commaLocations[1]], commaLocations[2] - commaLocations[1],serial_buffer[commaLocations[2] + 1],&newData->latitude);
  if(status < 0) return status;
  //next do longitude
  status = parseLongitude(&serial_buffer[commaLocations[3]], commaLocations[4] - commaLocations[3],serial_buffer[commaLocations[4] + 1],&newData->longitude);
  if(status < 0) return status;
  //gps fix quality
  status = parseFix(&serial_buffer[commaLocations[5]],commaLocations[6] - commaLocations[5], &newData->fix_quality);
  if(status < 0) return status;
  //now get number of satellites
  status = parseSatellites(&serial_buffer[commaLocations[6]],commaLocations[7] - commaLocations[6], &newData->satellites);
  if(status < 0) return status;
  //get horizontal dilution
  status = parseSatellites(&serial_buffer[commaLocations[6]],commaLocations[7] - commaLocations[6], &newData->satellites);
  if(status < 0) return status;
  //altitude
  status = parseAntennaAltitude(&serial_buffer[commaLocations[7]],commaLocations[8] - commaLocations[7], &newData->altitude);
  if(status < 0) return status;
  //sucessful up to this point, return with no error
  return 0;
}

int GPS_Handler::readGps(){
  //read until a delim char, return status
  while(gps_communication->available()){
    //Serial.println("Data avail!");
      char c = (char)gps_communication->read();
      //Serial.print(c);
      if(c == '\n' || c == '\r'){
        //check for minimum message length
        if(buffer_index > 2){
          //message completed, report it
          return 1;
        }
        //else, assume useless return and continue
        else{
          buffer_index = 0;
        }
      }
      else{
        //new char, add it to the buffer
        serial_buffer[buffer_index] = c;
        buffer_index ++;
        if(buffer_index >= GPS_MESSAGE_MAX_LENGTH){
          //ruh ro..
          //wipe it
          buffer_index = 0;
          //report an error
          return GPS_BUFFER_OVERFLOW;
        }
      }
  }
  //no new message completed, return blank status
  return 0;
}


int GPS_Handler::copyGpsData(GPS_Data *target, GPS_Data *source){
  for(uint i = 0; i < source->length; i ++){
    target->raw_string[i] = source->raw_string[i];
  }
  target->length = source->length;
  target->latitude = source->latitude;
  target->longitude = source->longitude;
  target->altitude = source->altitude;
  copyGpsTimeData(&target->time_stamp, &source->time_stamp);
  target->satellites = source->satellites;
  target->dilution = source->dilution;
  target->fix_quality = source->fix_quality;
  return 0;
}

int GPS_Handler::copyGpsTimeData(GPS_Time *target, GPS_Time *source){
  target->hours = source->hours;
  target->minutes = source->minutes;
  target->seconds = source->seconds;
  target->centiseconds = source->centiseconds;
  target->valid = source->valid;
  return 0;
}

int GPS_Handler::init(HardwareSerial *serial){
  serial->println("Hello!");
  gps_communication = serial;
  return 0;
}

int GPS_Handler::computeChecksum(char *buffer, uint length, int *checksum){
  //go up until the last part
  int calculatedChecksum = 0;
  for(uint i = 0; i < length; i ++){
    calculatedChecksum ^= buffer[i];
  }
  *checksum = calculatedChecksum;
  return 0;
}

int GPS_Handler::parseLatitude(char *latBuffer, uint length, char dir, float *latitude){
    //go check the direction char for validity
    if(dir != 'N' && dir != 'S'){
      #ifdef GPS_DEBUG
        Serial.println("Latitude Parsing Dir Unkown: " + String(dir));
      #endif
      return -1;
    }
    float parsedLatitude = 0;
    char num[length];
    for(uint i = 0; i < length; i ++){
      num[i] = latBuffer[i];
    }
    //so now we parse to decimal format
    //first two 'digits' are correct (degrees)
    parsedLatitude += 10*(latBuffer[0] - '0');
    parsedLatitude += (latBuffer[1] - '0');
    //the rest is in minutes format
    float temp = atof(&num[2]);
    parsedLatitude += 1.0/60.0*temp;
    if(dir == 'S') parsedLatitude *=-1;
    *latitude = parsedLatitude;
    return 0;
}

int GPS_Handler::parseLongitude(char *lonBuffer, uint length, char dir, float *longitude){
  //go check the direction char for validity
  if(dir != 'W' && dir != 'E'){
    #ifdef GPS_DEBUG
      Serial.println("Longitude Parsing Dir Unkown: " + String(dir));
    #endif
    return -1;
  }
  float parsedLongitude = 0;
  char num[length];
  for(uint i = 0; i < length; i ++){
    num[i] = lonBuffer[i];
  }
  //so now we parse to decimal format
  //first two 'digits' are correct (degrees)
  parsedLongitude += 100*(lonBuffer[0] - '0');
  parsedLongitude += 10(lonBuffer[1] - '0');
  parsedLongitude += (lonBuffer[2] - '0');
  //the rest is in minutes format
  float temp = atof(&num[3]);
  parsedLongitude += 1.0/60.0*temp;
  if(dir == 'W') parsedLongitude *=-1;
  *longitude = parsedLongitude;
  return 0;
}

int GPS_Handler::parseSatellites(char *buffer, uint length, uint8_t *number){
  char num[length];
  for(uint i = 0; i < length; i ++){
    num[i] = buffer[i];
  }
  *number = atoi(num);
  return 0;
}

int GPS_Handler::parseHorizontalDilution(char *buffer, uint length, float *dilution){
  char num[length];
  for(uint i = 0; i < length; i ++){
    num[i] = buffer[i];
  }
  *dilution = atof(num);
  return 0;
}

int GPS_Handler::parseAntennaAltitude(char *buffer, uint length, float *altitude){
  char num[length];
  for(uint i = 0; i < length; i ++){
    num[i] = buffer[i];
  }
  *altitude = atof(num);
  return 0;
}

int GPS_Handler::parseChecksum(char *buffer, uint length, int *checksum){
  char num[length];
  for(uint i = 0; i < length; i ++){
    num[i] = buffer[i];
  }
  *checksum = strtol(num,NULL,16);
  return 0;
}

int GPS_Handler::parseFix(char *buffer, uint length, GPS_FIX_QUALITY *fix){
  if(length > 1){
    //something's wrong...
    #ifdef GPS_DEBUG
      Serial.println("GPS fix too large!");
    #endif
    return -1;
  }
  char fixFill[1] = {buffer[0]};
  int fixStatus = atoi(fixFill);
  if(fixStatus > 2){
    //uhhhh
    #ifdef GPS_DEBUG
      Serial.println("Unvalid GPS fix quantity: " + String(fixStatus));
    #endif
    return -1;
  }
  if(fixStatus == 0) *fix = FIX_INVALID;
  else if(fixStatus == 1) *fix = GPS_FIX;
  else if(fixStatus == 2) *fix = D_GPS_FIX;
  else return -1;
  return 0;
}

int GPS_Handler::parseTime(char *buffer, uint length, GPS_Time *time){
  int secondPlace = findChar(&buffer[0], length, '.');
  if(secondPlace < 0){
    #ifdef GPS_DEBUG
      Serial.println("No /'./' found on time code parsing");
    #endif
    return -1;
  }
  //otherwise, the following chars are the centi seconds
  if(length - secondPlace < 2){
    //idk if this is a hard failure
    #ifdef GPS_DEBUG
      Serial.println("Not enough decimal places after /'./' in time code parsing. Places avail: " + String(length - secondPlace));
    #endif
  }
  else{
    char centisecondsPlace[2] = {buffer[secondPlace +1], buffer[secondPlace +2]};
    time->centiseconds = atoi(centisecondsPlace);
  }
  //otherwise, work our ways back up
  if(secondPlace < 6){
    #ifdef GPS_DEBUG
      Serial.println("Not enough decimal places before /'./' for hhmmss in time code parsing. Places avail: " + String(secondPlace));
    #endif
    return -1;
  }
  else{
    char seconds[2] = {buffer[secondPlace - 2], buffer[secondPlace - 1]};
    time->seconds = atoi(seconds);
    char minutes[2] = {buffer[secondPlace - 4], buffer[secondPlace - 3]};
    time->minutes = atoi(minutes);
    char hours[2] = {buffer[secondPlace - 6], buffer[secondPlace - 5]};
    time->hours = atoi(hours);
  }
  return 0;
}

int GPS_Handler::findChar(char *buffer, uint length, char toLookFor, uint startIndex){
  for(uint i = startIndex; i < length; i ++){
    if(buffer[i] == toLookFor){
      return i;
    }
  }
  return -1;
}

int GPS_Handler::findStr(char *buffer, uint length, char *toLookFor, uint look_length, uint startIndex){
  for(uint i = startIndex; i < length - look_length; i ++){
    if(buffer[i] == toLookFor[0]){
      //now go look ahead
      bool success = true;
      for(uint p = 1; p < look_length; p ++){
        if(buffer[i+p] != toLookFor[p]){
          success = false;
          break;
        }
      }
      if(success) return i;
    }
  }
  return -1;
}
