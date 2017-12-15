
// written by Tanvi Deora but actually mostly by Joseph Sullivan
// to 1) detect proboscis as a break in IR path
// 2) trigger cameras

const int inPin = A0;   // IR sensor wire is plugged into pin 7
const float OnThreshold = 1.2;
const float OffThreshold = 1.0;
const uint32_t tDelay = 6e6;
uint32_t startTime;
uint32_t tRemoved;
bool ProboscisDetect;
bool inject;
bool tDetect;

union {
  float asFloat;
  byte asBytes[4];
} val;

//union {
//  uint32_t asUint32;
//  byte asBytes[4];
//} tDetect;

typedef struct{
  int len;
  bool * window;
} window_t;
//
//window_t new_window(int len) { // 'Factory'
//  bool * window = (bool *)malloc(len);
//  window_t my_window;
//  my_window.len = len;
//  my_window.window = window;
//  return my_window;
//}

bool bar[10];
window_t DetectOnWindow = {.len=sizeof(bar), .window = bar};
bool foo[3000];
window_t DetectOffWindow = {.len=sizeof(foo), .window = foo};

void window_shift(window_t * window_ptr, bool value) {
  for (int i = 0; i < window_ptr->len -1; i++) {
    window_ptr->window[i+1] = window_ptr->window[i];
  }
  window_ptr->window[0] = value;
}

bool window_state(window_t * window_ptr) {
  for (int i = 0; i < window_ptr->len; i++) {
    if (!window_ptr->window[i]) {
      return false;
    }
  }
  return true;
}

//typedef struct {
//  static const int len = 10;
//  bool window[len];
//} DetectOnWindow_t;
//
//typedef struct {
//  static const int len = 5000;
//  bool window[len];
//} DetectOffWindow_t;



// PWM Configuration related constants
const PinDescription pwm_pin_description = g_APinDescription[9];
const uint32_t periph_id = ID_PWM;
const uint32_t pwm_channel = pwm_pin_description.ulPWMChannel;
const uint32_t clocka = 1000; // frequency of pwm clock source
const uint32_t master_clk = 48000000;
const uint32_t prescaler = 0x0B; // identifies CLOCKA
const uint32_t frequency = 100; // frequency
const uint32_t period_register_data = clocka / frequency;
const uint32_t duty_register_data = period_register_data / 2;

bool interrupt_flag = false;
void PWM_Handler(void) {
  interrupt_flag = true;
  volatile long dummy = PWM_INTERFACE->PWM_ISR1; // clear interrupt interrupt_flag
  dummy = PWM_INTERFACE->PWM_ISR2; // clear interrupt interrupt_flag
}

void setup_pwm() {
  // Enable PWM clock in PMC
  pmc_enable_periph_clk(periph_id);
  PWMC_ConfigureClocks(clocka, 0, VARIANT_MCK);
  // Do PIO configuration
  PIO_Configure(
    pwm_pin_description.pPort,
    pwm_pin_description.ulPinType,
    pwm_pin_description.ulPin,
    pwm_pin_description.ulPinConfiguration);
  PWMC_ConfigureChannel(PWM_INTERFACE, pwm_channel, prescaler, 0, 0);
  PWMC_SetPeriod(PWM_INTERFACE, pwm_channel, period_register_data);
  PWMC_SetDutyCycle(PWM_INTERFACE, pwm_channel, duty_register_data);
  PWMC_EnableChannel(PWM_INTERFACE, pwm_channel);
  PWMC_EnableIt(PWM_INTERFACE, 1<<pwm_channel, 1<<pwm_channel);
  // Enable PWM interrupt in NVIC, ID = 36
  NVIC_DisableIRQ(PWM_IRQn);
  NVIC_ClearPendingIRQ(PWM_IRQn);
  NVIC_SetPriority(PWM_IRQn, 0);
  NVIC_EnableIRQ((IRQn_Type)36);
}

void setup() {
  pinMode(inPin, INPUT);      // sets the digital pin 7 as input WHY?
  Serial.begin(115200);
  // Wait for start command before we begin
  char buff[6];
  while(strcmp(buff, "start")!=0){/*wait until the string "start" is read*/
    int count = Serial.readBytesUntil('\n',buff,6);
    buff[count] = '\0';
  }
  val.asFloat = 0.0; // variable to store the read value (0=beam intact,1=beam interrupted)
  ProboscisDetect = false;
  inject = false;
  tDetect = false;
  startTime=micros();
  setup_pwm();
  Serial.write(startTime);
}

void loop() {

//Serial.println("started");
 bool stopcmd;
 char buf[5];
 int count = Serial.readBytesUntil('\n',buf,5);
 buf[count] = '\0'; 
if (strcmp(buf,"stop")==0)
  //wait until the string "stop" is read
 {
  //Serial.println("stopped");
  exit(0); 
 }
else 
 { 
  
  static struct {
    union {
      uint32_t asUint32;
      byte asBytes[4];
    };
  } camera_time;

  // Task 1, records time of camera trigger, transmits packets to PC
  if(interrupt_flag) {
    camera_time.asUint32 = micros() - startTime;
    Serial.write(0xFF);
    Serial.write(camera_time.asBytes, 4);
    Serial.write(val.asBytes, 4);
    Serial.write(tDetect);
    tDetect = false;
    Serial.write(inject);
    inject = false;
    Serial.write(0xAA);
    interrupt_flag = false;
    //Serial.println("running the loop");
  }

  uint32_t now = micros();
  static uint32_t last_time;
  static const uint32_t dt = 1 * 1000; // sample IR sensor data at 1kHz
  // Task 2, measure IR sensor, sends injection commands
  if (now - last_time > dt) {
    last_time = now;
    // Measure IR sensor, filter events through sliding window
    val.asFloat = 3.3 / 1024 * analogRead(A0);
    window_shift(&DetectOnWindow, val.asFloat > OnThreshold);
    window_shift(&DetectOffWindow, val.asFloat < OffThreshold);


    if (!ProboscisDetect && window_state(&DetectOnWindow))
    {
      ProboscisDetect = true;
      tDetect = true;
    }

    else if (ProboscisDetect && window_state(&DetectOffWindow))
    {
      //Serial.println("Proboscis removed");
      tRemoved = micros();
      ProboscisDetect = false;
    }
    else if ((tRemoved > 0) && !ProboscisDetect && (micros() >= tRemoved + tDelay))
    {
      //Serial.println("Inject");
      tRemoved =0;
      inject = true;
    }
   }

 }
}
