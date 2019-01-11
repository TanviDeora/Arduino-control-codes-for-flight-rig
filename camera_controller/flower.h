#ifndef FLOWER_H
#define FLOWER_H
#include "Arduino.h"
class Flower {
/* *
 *  An abstraction of an artificial flower.
 */
  public:
  Flower(int irChannel, int solnChannel);
  bool isEmpty(void);
  float readIRSensor(void);
  float readSolutionSensor(void);
  void proboscisDetectShift(float);
  bool proboscisDetected(void);
  bool proboscisUndetected(void);
  void checkSolnState(float);
  void checkProboscisState(float);
  void clearProboscisDetectFlags(void);
  
  int irChannel; // Pin connected to this flower's IR sensor
  int solnChannel; // Pin connected to this flower's solution sensor

  // Thresholds used for proboscis detection
  float detectThreshold = 1.;
  float undetectThreshold = 0.3;

  // Thresholds used in solution detection / undetection
  float fullThreshold = 3.;
  float emptyThreshold = 0.3;

  // Variables used for tracking proboscis state
  bool proboscisWasDetected = false;
  bool proboscisWasRemoved = true;
  
  // This enumeration is added for code clarity
  typedef enum {
    FULL,
    EMPTY
  }solnState_t;
  solnState_t solnState; // State variable used to indiciate if flower is full or empty

  // Timing parameters used to control when injections occur
  static constexpr uint32_t tDelay = 6e6; // six seconds delay time
  uint32_t tRemoved;
  uint32_t tDetect;

  // Enumeration added for code clarity
  typedef enum {
    DETECTED,
    UNDETECTED,
    INDETERMINATE
  }proboscisDetect_t;
  
  // Structs used in proboscis detection / undetection
  struct {
    static const int len = 10;
    proboscisDetect_t window[len];
  } proboscisDetectWindow;

};

/* The flower class constructor */
Flower::Flower(int irChannel, int solnChannel) {
  this->irChannel = irChannel;
  this->solnChannel = solnChannel;
  this->solnState = EMPTY;
  this->proboscisWasRemoved = true;
}

/* Returns true is the flower is empty */
bool Flower::isEmpty(void) {
  return solnState == EMPTY;
}

/* Reads the IR sensor, checks the proboscis state, returns the voltage measured */
float Flower::readIRSensor(void) {
  float value = 3.3 / 1024 * analogRead(this->irChannel);
  this->checkProboscisState(value);
  return value;
}

/* */
void Flower::checkProboscisState(float value) {
  proboscisDetectShift(value);
  if (proboscisDetected() && (proboscisWasRemoved)) {
    proboscisWasDetected = true;
    proboscisWasRemoved = false;
    tDetect = micros();
  }
  else if (proboscisUndetected() && (!proboscisWasRemoved)) {
    proboscisWasRemoved = true;
    tRemoved = micros();
  }
}

/* Clears the proboscis detection flags
   This function is called by the main loop whenever
   a data packet is transmitted to the PC.
*/
void Flower::clearProboscisDetectFlags(void) {
  this->proboscisWasDetected = false;
}


/* Reads the solution sensor, checks the solution state, returns the voltage measured*/
float Flower::readSolutionSensor(void) {
  float value = 3.3 / 1024 * analogRead(this->solnChannel);
  this->checkSolnState(value);
  return value;
}

/* Checks to see if the solution state has changed, and updates it accordingly. */
void Flower::checkSolnState(float value) {
  if ((solnState == FULL) && (value < emptyThreshold)) {
    solnState = EMPTY;
  }
  else if ((solnState == EMPTY) && (value > fullThreshold)) {
    solnState = FULL;
  }
}

/* Used to shift new data into the probiscis detection window */
void Flower::proboscisDetectShift(float ir_sensor_value) {
  proboscisDetect_t * window = proboscisDetectWindow.window;
  for (int i = 0; i < proboscisDetectWindow.len - 1; i ++) {
    window[i+1] = window[i];
  }
  if (ir_sensor_value > detectThreshold) {
    window[0] = DETECTED;
  }
  else if (ir_sensor_value < undetectThreshold) {
    window[0] = UNDETECTED;
  }
  else {
    window[0] = INDETERMINATE;
  }
}

/* Returns the state of the proboscis detection window */
bool Flower::proboscisDetected() {
  proboscisDetect_t * window = proboscisDetectWindow.window;
  for (int i = 0; i < proboscisDetectWindow.len; i++) {
    if (window[i] == UNDETECTED || window[i] == INDETERMINATE) {
      return false;
    }
  }
  return true;
}

/* Returns the state of the proboscis UNdetection window */
bool Flower::proboscisUndetected() {
  proboscisDetect_t * window = proboscisDetectWindow.window;
  for (int i = 0; i < proboscisDetectWindow.len; i++) {
    if (window[i] == DETECTED || window[i] == INDETERMINATE) {
      return false;
    }
  }
  return true;
}
#endif
