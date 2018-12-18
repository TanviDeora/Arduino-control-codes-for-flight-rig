// FOO
#include <Wire.h>
/*
 * Code by Joseph Sullivan
 */

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

  // Set up digital pins which indicate which flowers have nectar
  pinMode(inPin, INPUT);      // sets the digital pin 7 as input WHY?

  // Construct flower objects

  // Start the serial port
  Serial.begin(115200);
  // Wait for start command before we begin
  char buff[6];
  while(strcmp(buff, "start")!=0){/*wait until the string "start" is read*/
    size_t count = Serial.readBytesUntil('\n',buff,6);
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

  static bool stop_command = false;
  static uint32_t last_time;
  static const uint32_t dt = 1 * 1000; // sample IR sensor data at 1kHz
  static float[4] irSensorVals;
  static float[4] solnSensorvals;
  static int inject = 0;
  uint32_t now = micros();

  // Task 1, records time of camera trigger, transmits packets to PC
  if(interrupt_flag && !stop_command) {
    uint32_t cameraTime = micros() - startTime;
    packet_t packet;
    init_packet(&packet);
    for (int i = 0; i < 4; i++) {
      packet.irSensorVals[i] = irSensorVals[i];
      packet.proboscisDetect[i] = (flowers[i]->tDetect != 0);
      flowers[i]->tDetect = 0;
    }
    Serial.write((uint8_t *)packet, sizeof(packet))
    interrupt_flag = false;
  }

  // Read bytes from the serial buffer so that we intercept the "start" and "stop" commands
  else {
    static char buff[10];
    static uint8_t head = 0;
    while (Serial.available()) {
      size_t count = Serial.readBytesUntil('\n', buff, sizeof(buff)-1);
      buff[count] = '\0';
      if (!strcmp(buff, "stop")) {
        stop_command = true;
        pmc_disable_periph_clk(periph_id);
      }
      else if (!strcmp(buff, "start")) {
        stop_command = false;
        startTime = micros();
        Serial.write(startTime);
        setup_pwm();
        interrupt_flag = false;
      }
    }
  }

  /* Collect new data from flowers and refill them as necessary*/
  if ((now - last_time) > dt) {
    last_time = now;
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
  static int pinMap[4] = {4, 5, 6, 7};
  bool allFlowersFull = true;
  for (int i = 0; i < 4; i++) {
    Flower * flower = flowers[i];
    if (flower->isEmpty()) {
      allFlowersFull = false;
      bool mothNotFeeding = true;
      for (int j = 0; j < 4; j++) {
        mothNotFeeding &= flowers[j]->proboscisUndetected();
      }
      if (mothNotFeeding) {
        uint32_t tRemoved = flower->tRemoved;
        bool proboscisWasRemoved = tRemoved > 0;
        if (proboscisWasRemoved && (micros() > (tRemoved + tDelay))) {
          digitalWrite(pinMap[i], HIGH);
          digitalWrite(22, HIGH);
          inject = i+1;
        }      
      }
      break;
    }
  }

  // Stop injecting if flowers are all full
  if (allFlowersFull) {
    digitalWrite(pinMap[inject-1], LOW);
    digitalWrite(22, LOW);
  }
}
