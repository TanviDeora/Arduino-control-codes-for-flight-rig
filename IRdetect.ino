// written by Tanvi Deora but actually mostly by Joseph Sullivan
// to 1) detect proboscis as a break in IR path
// 2) trigger cameras

const int inPin = A0;   // IR sensor wire is plugged into pin 7
const unsigned int sampleRate = 2000;     // sample Rate in Hz
const float dT = 1000000.0 / float(sampleRate);
const float threshold = 0.6;
const uint32_t tDelay = 10e6;
uint32_t startTime;


union {
  float asFloat;
  byte asBytes[4];
} val;

union {
  uint32_t asUint32;
  byte asBytes[4];
} tDetect;
  
uint32_t tRemoved;

bool ProboscisDetect;

struct {
  static const int len = 1000;
  bool window[len];
} DetectWindow;

void window_shift(bool value) {
  for (int i = 0; i < DetectWindow.len - 1; i ++) {
    DetectWindow.window[i+1]=DetectWindow.window[i];
  }
  DetectWindow.window[0] = value;
}

bool window_state() {
  for (int i = 0; i < DetectWindow.len; i++) {
    if (DetectWindow.window[i]) {
      return true;
    }
  }
  return false;
}

bool inject;

void setup()
{
  // pinMode(ledPin, OUTPUT);      // sets the digital pin 13 as output
  pinMode(inPin, INPUT);      // sets the digital pin 7 as input
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  // wait for 'start\n'
  char buff[6];
 while(strcmp(buff, "start")!=0){/*wait until the string "start" is read*/
    int count = Serial.readBytesUntil('\n',buff,6);
    buff[count] = '\0';
 }
  val.asFloat = 0.0; // variable to store the read value (0=beam intact,1=beam interrupted)
  ProboscisDetect = false;
  inject = false;
  startTime=micros();
  
}

void loop()
{
  //Serial.println("My Sketch has started");
  val.asFloat = 5. / 1024 * analogRead(A0);   // read the input pin
  //Serial.println(val.asFloat);

  window_shift(val.asFloat > threshold);
  if (!ProboscisDetect && (window_state() == true))
  {
    //Serial.println("Proboscis detect");
    ProboscisDetect = true;
    tDetect.asUint32 = micros() - startTime;///1000000.0; // logging t of detection in seconds 
  } 

  if (ProboscisDetect && !window_state())
  { 
    //Serial.println("Proboscis removed");
    tRemoved = micros() - startTime;
    ProboscisDetect = false;
    inject = false;
  }
  else if ((tRemoved > 0) && (micros() - startTime >= tRemoved + tDelay)) 
  {
    //Serial.println("Inject");
    tRemoved =0;
    inject = true; 
  }
  else 
  {
    inject = false;
  }
  
  //Serial.println(sizeof("start\n"));
  //Serial.write(sizeof("start\n"));
  Serial.write(0xFF);
  Serial.write(val.asBytes,4); // print out the value of val
  Serial.write(tDetect.asBytes,4);
  Serial.write(inject);
  Serial.write(0xAA);
  Serial.write(0x00);
 //Serial.write(ProboscisDetect.asBytes, 4); // print out Probocis detected or not
 //Serial.write(tDetect.asBytes, 4); // print out Probocis detected time
 //Serial.write(inject.asBytes, 4); // send inject signal after 1 min of proboscis out

 delay(1);
}
 
