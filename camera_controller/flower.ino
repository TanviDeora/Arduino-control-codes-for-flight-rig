// FOO
class Flower {
/* *
 *  An abstraction of an artificial flower.
 */
  public:
  Flower(int irChannel, int solnChannel);
  bool isEmpty(void);
  float readIRSensor(void);
  float readSolutionSensor(void);
  void proboscisDetectShift(bool);
  void proboscisUndetectShift(bool);
  bool proboscisDetected(void);
  bool proboscisUndetected(void);
  void checkSolnState(float);
  void checkProboscisState(float);
  
  int irChannel; // Pin connected to this flower's IR sensor
  int solnChanel; // Pin connected to this flower's solution sensor
    
  // Thresholds used in proboscis detection / undetection
  static constexpr float detectThreshold = 0.6;
  static constexpr float undetectThreshold = 0.5;

  // Thresholds used in solution detection / undetection
  static constexpr float fullThreshold = 3.;
  static constexpr float emptyThreshld = 0.3;

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
    UNDETECTED
  }proboscisDetect_t;
  
  // Structs used in proboscis detection / undetection
  struct {
    static const int len = 10;
    proboscisDetect_t window[len];
  } proboscisDetectWindow;

  struct {
    static const int len = 5000;
    proboscisDetect_t window[len];
  } proboscisUndetectWindow;

};

/* The flower class constructor */
Flower::Flower(int irChanel, int solnChannel) {
  this->irChannel = irChannel;
  this->solnChannel = solnChannel;
  this->solutionState = false;
}

/* Returns true is the flower is empty */
bool Flower::isEmpty(void) {
  this->checkSolnState();
  return solnState == EMPTY;
}

/* Reads the IR sensor, checks the proboscis state, returns the voltage measured */
float Flower::readIRSensor(void) {
  float value = 3.3 / 4096 * analogRead(adcChannel);
  this->checkProboscisState(value);
  return value;
}

/* */
void Flower::checkProboscisState(float value) {
  
  proboscisDetectShift(value > onThreshold);
  proboscisUndetectShift(value < offThreshold);
  
  bool proboscisWasDetected = tDetect > 0;
  if (proboscisDetected() && (!proboscisWasDetected)) {
    tDetect = micros();
  }
  else if (proboscisUndetected() && (proboscisWasDetected)) {
    tRemoved = micros();
  }
}


/* Reads the solution sensor, checks the solution state, returns the voltage measured*/
float Flower::readSolutionSensor(void) {
  float value = 3.3 / 4096 * analogRead(solnChannel);
  this->checkSolnState(value);
  return value;
}

/* Checks to see if the solution state has changed, and updates it accordingly. */
void Flower::checkSolnState(float value) {
  if ((solnState == FULL) && (val > fullThreshold)) {
    solnState = EMPTY;
  }
  else if ((solnState == EMPTY) && (val < emptyThreshold)) {
    solnState = FULL;
  }
}

/* Used to shift new data into the probiscis detection window */
void Flower::proboscisDetectShift(proboscisDetect_t value) {
  proboscisDetect_t * window = proboscisDetectWindow.window;
  for (int i = 0; i < proboscisDetectWindow.len - 1; i ++) {
    window[i+1] = window[i];
  }
  window[0] = value;
}

/* Used to shift new data into the probiscis UNdetection window */
void Flower::proboscisUndetectShift(proboscisDetect_t value) {
  proboscisDetect_t * window = proboscisUndetectWindow.window;
  for (int i = 0; i < proboscisUnDetectWindow.len - 1; i ++) {
    window[i+1] = window[i];
  }
  window[0] = value;
}

/* Returns the state of the proboscis detection window */
bool Flower::proboscisDetected() {
  proboscisDetect_t * window = proboscisDetectWindow.window;
  for (int i = 0; i < proboscisDetectWindow.len; i++) {
    if (window[i] == UNDETECTED) {
      return true;
    }
  }
  return false;
}

/* Returns the state of the proboscis UNdetection window */
bool Flower::proboscisUndetected() {
  proboscisDetect_t * window = proboscisUndetectWindow.window;
  for (int i = 0; i < proboscisUndetectWindow.len; i++) {
    if (window[i] == DETECTED) {
      return false;
    }
  }
  return false;
}
