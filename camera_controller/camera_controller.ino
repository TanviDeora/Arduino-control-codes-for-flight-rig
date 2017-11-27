
// written by Tanvi Deora but actually mostly by Joseph Sullivan
// to 1) detect proboscis as a break in IR path
// 2) trigger cameras

const int inPin = A0;   // IR sensor wire is plugged into pin 7
const float OnThreshold = 0.6;
const float OffThreshold = 0.5;
const uint32_t tDelay = 10e6;
uint32_t startTime;
uint32_t tRemoved;
bool ProboscisDetect;
bool inject;

union {
  float asFloat;
  byte asBytes[4];
} val;

union {
  uint32_t asUint32;
  byte asBytes[4];
} tDetect;

struct {
  static const int len = 10;
  bool window[len];
} DetectOnWindow;

struct {
  static const int len = 100;
  bool window[len];
} DetectOffWindow;

void windowOn_shift(bool value) {
  for (int i = 0; i < DetectOnWindow.len - 1; i ++) {
    DetectOnWindow.window[i+1]=DetectOnWindow.window[i];
  }
  DetectOnWindow.window[0] = value;
}

void windowOff_shift(bool value) {
  for (int i = 0; i < DetectOffWindow.len - 1; i ++) {
    DetectOffWindow.window[i+1]=DetectOffWindow.window[i];
  }
  DetectOffWindow.window[0] = value;
}

bool windowOn_state() {
  for (int i = 0; i < DetectOnWindow.len; i++) {
    if (DetectOnWindow.window[i]) {
      return true;
    }
  }
  return false;
}

bool windowOff_state() {
  for (int i = 0; i < DetectOffWindow.len; i++) {
    if (DetectOffWindow.window[i]) {
      return true;
    }
  }
  return false;
}

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
  startTime=micros();
  setup_pwm();
  Serial.write(startTime);
}

void loop() {

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
    Serial.write(tDetect.asBytes, 4);
    Serial.write(inject);
    inject = false;
    Serial.write(0xAA);
    interrupt_flag = false;
  }

  uint32_t now = micros();
  static uint32_t last_time;
  static const uint32_t dt = 1 * 1000; // sample IR sensor data at 1kHz
  // Task 2, measure IR sensor, sends injection commands
  if (now - last_time > dt) {
    last_time = now;
    // Measure IR sensor, filter events through sliding window
    val.asFloat = 3.3 / 1024 * analogRead(A0);
    windowOn_shift(val.asFloat > OnThreshold);
    windowOff_shift(val.asFloat < OffThreshold);

    if (!ProboscisDetect && (windowOn_state() == true))
    {
      ProboscisDetect = true;
      if (inject == false)
      {
        tDetect.asUint32 = micros() - startTime;
      }
      else
      {
        tDetect.asUint32 = 0;     
      }
    }

    if (ProboscisDetect && windowOff_state() == true)
    {
      //Serial.println("Proboscis removed");
      tRemoved = micros() - startTime;
      ProboscisDetect = false;
    }
    else if ((tRemoved > 0) && (micros() - startTime >= tRemoved + tDelay))
    {
      //Serial.println("Inject");
      tRemoved =0;
      inject = true;
    }
   }
}
