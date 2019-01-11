#include<Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

/* The amount of time, in seconds, that the injector will wait
   after an injection before retracting the liquid. */
#define TIMEOUT    10

/* I2C bus speed, is modified in setup() */
#define I2C_FREQ 400000L

/* Buffer length used for storing / sending messages over UART
  and for doing general string comparisons*/
#define BUFFER_LEN 256

/* Defines for IO port names*/
#define S1 9 // switch near motor
#define S2 10// switch near idler
#define SOLENOID_ENERGIZE 11
#define BOUNCE_DELAY 100
#define TRIGGER_PIN 8
#define USER_TRIGGER_PIN 13

// Motor constants
static const int motor_speed = 600;
static const int steps_per_rev = 200;
// ADC threshold values for detecting liquid by electrical conductivity
/* Enumeration of possible states */
enum STATE{
  wait,
  inject,
  withdraw
};

/* Enumeration of possible positions */
enum POSITION{
  zero_stroke,
  mid_stroke,
  full_stroke
};

Adafruit_MotorShield AFMS = Adafruit_MotorShield();             // Create the motor shield object with the default I2C address.

Adafruit_StepperMotor *motor = AFMS.getStepper(steps_per_rev, 1);// Connect a stepper motor with 200 steps per revolution (1.8 degree)
                                                                // to motor port #2 (M3 and M4).

bool stopflag = false;                                          // Emertency stop flag that is set by the forward limit switch. Prevents damage to injector.
bool reset = true;                                              // Reset flag used to initialize the position of the injector.
bool is_empty;                                                  // Flag used for detecting


int steps = 0;                                                  // Number of steps left for the actuator to complete before its current command is finished
int volume = 0;                                                 // Volume of fluid requested for injection by serial communication interface.
enum POSITION injector_pos;                                     // General position of the injector at power-on reset. Can be zero-stroke, mid-stroke, or full-stroke.
enum STATE injector_state;                                      // Control variable. Can be halt, reset_pos, wait, or inject
char input_buffer[BUFFER_LEN];                                  // Serial input buffer, used to store serial data and to parse commands.
char output_buffer[BUFFER_LEN];                                 // Serial output buffer, used to store messages which will be sent out to the serial port.

void setup() {

                                                                // initialize message buffer
  int i;
  for(i = 0; i < BUFFER_LEN; i++){
    output_buffer[i] = 0;
  }
                                                                // setup serial port
  Serial.begin(115200);

  /* Set up pin modes for safety / limit switches */
  pinMode(S1, INPUT_PULLUP);                                    // pole 1 of safety switch 1
  delay(10);                                                    // For some reason this delay is necessary
  pinMode(S2, INPUT_PULLUP);                                    // Configure the S2 with an internal pullup.
  delay(10);

  pinMode(SOLENOID_ENERGIZE, OUTPUT);                           // Set solenoid mosfet driver to output
  digitalWrite(SOLENOID_ENERGIZE, LOW);

  /* Determine the initial state of the micoinjector */
  if (!digitalRead(S1) && digitalRead(S2)){                     // Actuator is initally in zero_stroke position
    injector_pos = zero_stroke;
    injector_state = wait;
    Serial.println("Acuator initial position is zero-stroke.");
  }
  else if (digitalRead(S1) && digitalRead(S2)){                // Actuator is initially in full_stroke position
    injector_pos = mid_stroke;
    injector_state = withdraw;
    // Switch the solenoid to source liquid
    digitalWrite(SOLENOID_ENERGIZE, HIGH);
    Serial.println("Actuator initial position is mid-stroke");
  }
  else if (digitalRead(S1) && !digitalRead(S2)){                 // Actuator is initially in mid_stroke
    injector_pos = full_stroke;
    injector_state = withdraw;
    // Switch the solenoid to source liquid
    digitalWrite(SOLENOID_ENERGIZE, HIGH);
    Serial.println("Actuator initial position is full-stroke");
  }
  else{
    Serial.println("Actuator initial position unknown...");
    sprintf(output_buffer, "S1 %d S2 %d", digitalRead(S1), digitalRead(S2));
    Serial.println(output_buffer);
    Serial.println("There is a problem with the limit switches.");
    Serial.println("Fix the switches and then reset or reprogram.");
    while(1);
  }

  /* Setup of the TTL injection trigger*/
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(USER_TRIGGER_PIN, INPUT_PULLUP);

  AFMS.begin();                                                 // Initialize motor
  motor->setSpeed(motor_speed);

  stopflag = false;

  TWBR = ((F_CPU / I2C_FREQ) - 16) / 2;                         // Change the I2C frequency
}

void loop() {
  if (wait == injector_state){                             // wait state. The injector is waiting for an injection commmand form the user.
    if(!digitalRead(TRIGGER_PIN) && !digitalRead(USER_TRIGGER_PIN)){
      injector_state = inject;
      digitalWrite(SOLENOID_ENERGIZE, LOW);
      Serial.println("Injecting.");
    }
  }

  else if (inject == injector_state){                           // inject state. The injector is currently stepping forward.
    /* If S2 is depressed, then the syringe is empty. */
    if (!digitalRead(S2)){
      motor->release();
      injector_pos = full_stroke;
      injector_state = withdraw;
      digitalWrite(SOLENOID_ENERGIZE, HIGH);
      Serial.println("End of actuator reached. Refilling syringe.");
    }

    /* If the liquid is detected, cease injecting and wait */
    else if (digitalRead(TRIGGER_PIN) || digitalRead(USER_TRIGGER_PIN)){
      motor->release();
      injector_state = wait;
      digitalWrite(SOLENOID_ENERGIZE, HIGH);
      Serial.println("Liquid detected, injection complete.");
    }

    else {
      motor->step(1, FORWARD, SINGLE);
      if(digitalRead(S1)){
        injector_pos = mid_stroke;                              // S1 high -> position is mid-stroke.
      }
    }
  }

  else if (withdraw == injector_state){                         // withdraw state. The injecgtor is currently stepping backwards
    /* if S1 is depressed, then the syringe is full */
    if(!digitalRead(S1)){
      motor->release();
      digitalWrite(SOLENOID_ENERGIZE, LOW);
      injector_pos = zero_stroke;
      injector_state = wait;
    }
    /* Step backward to fill the syringe */
    else {
        motor->step(10, BACKWARD, SINGLE);
    }
  }
//motor->step(1,FORWARD,SINGLE);
}
