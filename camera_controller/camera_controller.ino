#include <Wire.h>
#include "flower.h"
#include "packet.h"
/*
 * Code by Joseph Sullivan
 */

#define S1_ENERGIZE 30
#define S2_ENERGIZE 32
#define S3_ENERGIZE 34
#define INJECT  36
#define CAMERA_TRIGGER  9

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

// timing data
uint32_t startTime;
const uint32_t tDelay = 6e6; // Injection time delay in microseconds

// Set up flowers
Flower f1(A0, A7);
Flower f2(A1, A6);
Flower f3(A2, A5);
Flower f4(A3, A4);
Flower * flowers[4] = {&f1, &f2, &f3, &f4};

// PWM Interrupt configuration
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

void refillFlowers(void);

void setup() {

  pinMode(S1_ENERGIZE, OUTPUT);
  pinMode(S2_ENERGIZE, OUTPUT);
  pinMode(S3_ENERGIZE, OUTPUT);
  pinMode(INJECT, OUTPUT);
  pinMode(CAMERA_TRIGGER, OUTPUT);

  // Initialize output states
  digitalWrite(S1_ENERGIZE, LOW);
  digitalWrite(S2_ENERGIZE, LOW);
  digitalWrite(S3_ENERGIZE, LOW);
  digitalWrite(INJECT, LOW);
  digitalWrite(CAMERA_TRIGGER, HIGH);

  /* Set sensor thresholds for the flowers
  By expanding this loop, you can tune the
  thresholds for individual flowers
  */
  for (int i = 0; i < 4; i++) {
    flowers[i]->detectThreshold = 0.6;
    flowers[i]->undetectThreshold = 0.5;
    flowers[i]->fullThreshold = 3.;
    flowers[i]->emptyThreshold = 0.3;
  }
  
  // Start the serial port
  Serial.begin(115200);
  
  // Wait for start command before we begin
  char buff[6];
  while(strcmp(buff, "start")!=0){/*wait until the string "start" is read*/
    size_t count = Serial.readBytesUntil('\n',buff,6);
    buff[count] = '\0';
  }

  startTime=micros();
  setup_pwm();
  Serial.write(startTime);
}

void loop() {

  // static variables
  static bool stop_command = false;
  static uint32_t last_time;
  static const uint32_t dt = 1 * 1000; // sample IR sensor data at 1kHz
  float irSensorVals[4];
  float solnSensorVals[4];
  
  uint32_t this_time = micros();
  

  // Records time of camera trigger, transmits packets to PC
  if(interrupt_flag && !stop_command) {
    uint32_t cameraTime = micros() - startTime;
    packet_t packet;
    init_packet(&packet);
    for (int i = 0; i < 4; i++) {
      packet.irSensorVals[i] = irSensorVals[i];
      packet.proboscisDetect[i] = flowers[i]->proboscisWasDetected;
      flowers[i]->clearProboscisDetectFlags();
    }
    Serial.write(packet.data, sizeof(packet.data));
    interrupt_flag = false;
  }

  // Read bytes from the serial buffer so that we intercept the "start" and "stop" commands
  else {
    static char buff[10];
    while (Serial.available()) {
      size_t count = Serial.readBytesUntil('\n', buff, sizeof(buff)-1);
      buff[count] = '\0';
      if (!strcmp(buff, "stop")) {
        // If the stop command is received, disable PWM system
        stop_command = true;
        pmc_disable_periph_clk(periph_id);
      }
      else if (!strcmp(buff, "start")) {
        // If the start command is received, record time and enable PWM
        stop_command = false;
        startTime = micros();
        Serial.write(startTime);
        setup_pwm();
        interrupt_flag = false;
      }
    }
  }

  // Collect new data from flowers and refill them as necessary
  if ((this_time - last_time) > dt) {
    last_time = this_time;
    for (int i = 0; i < 4; i++) {
      irSensorVals[i] = flowers[i]->readIRSensor();
      solnSensorVals[i] = flowers[i]->readSolutionSensor();
    }
    refillFlowers();
  }
}

/* This tasks handles the refilling of any flowers that are empty
   It is noblocking, and must be called continuously.
*/
void refillFlowers(void) {
  bool allFlowersFull = true;
  for (int i = 0; i < 4; i++) {
    Flower * flower = flowers[i];
    if (flower->isEmpty()) {
      allFlowersFull = false;
      bool mothNotFeeding = true;
      for (int j = 0; j < 4; j++) mothNotFeeding &= flowers[j]->proboscisUndetected();
      if (mothNotFeeding) {
        if (flower->proboscisWasRemoved && (micros() > (flower->tRemoved + tDelay))) {
          // This switch activates the solenoid according to which flower is being filled
          switch (i) {
            case 0:
            digitalWrite(S1_ENERGIZE, HIGH);
            case 1:
            digitalWrite(S1_ENERGIZE, LOW);
            digitalWrite(S2_ENERGIZE, HIGH);
            case 2:
            digitalWrite(S2_ENERGIZE, LOW);
            digitalWrite(S3_ENERGIZE, HIGH);
            case 3:
            digitalWrite(S3_ENERGIZE, LOW);
            default:
            break;
          }
          // trigger the microinjector
          digitalWrite(INJECT, LOW);
        }    
      }
      break;
    }
  }
  // Stop injecting if flowers are all full
  if (allFlowersFull) {
    digitalWrite(S1_ENERGIZE, LOW);
    digitalWrite(S2_ENERGIZE, LOW);
    digitalWrite(S3_ENERGIZE, LOW);
    digitalWrite(INJECT, HIGH);
  }
}
