#ifndef timing_h
#define timing_h

#include <Arduino.h>

// Holds time information
struct TimeStruct {
  int hours;
  int minutes;
  float seconds;
};

// Creates a fresh time structure without logical garbage
TimeStruct createBlankTime();

// Updates a time structure with a time delta
void updateTime(TimeStruct &toUpdate, unsigned long timeDeltaMillis);

// Creates a string from a time structure
String formatTime(TimeStruct tStruct);

#endif