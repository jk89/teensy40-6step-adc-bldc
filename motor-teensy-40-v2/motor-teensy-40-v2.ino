// Copyright Jonathan Kelsey 2021

// goal: 3 phase brushless motor commutation based on bemf adc measurements synced to pwm cycle off state
#define DEBUG_MODE true

#include <Arduino.h>
#include "imxrt.h"
#include <ADC.h>

#define PI 3.1415926535897932384626433832795

#define A_IN 2  // A High/low switch
#define B_IN 7  // B High/low switch
#define C_IN 8  // C High/low switch
#define A_SD 1  // PWM A
#define B_SD 0  // PWM B
#define C_SD 22 // PWM C
#define LED_EN 13

/*
  ADC channel pins:
  14: Phase A
  15: Phase B
  16: Phase C
  17: Virtual Neutral
*/

#define PWM_FREQUENCY 1950// should be 20khz+ 1950 seems to atleast do something.

#define MIN_DUTY 40
#define MAX_DUTY 240

#define MOTOR_POLES 7
#define ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE (MOTOR_POLES / 2) // per physical revolution
#define ELECTRICAL_CYCLE_MOD 6

const float FULL_CYCLE = 2 * PI;
byte ELECTRICAL_STEP_CTR = 0;
int MECHANICAL_STEP_CTR = 0;
int SPEED_STEP_CTR = 0;
float RPM = 0;
elapsedMicros TIME_SINCE_LAST_MEASUREMENT;

const byte MECHANICAL_CYCLE_MOD = ELECTRICAL_CYCLES_PER_MECHANICAL_CYCLE * ELECTRICAL_CYCLE_MOD;
const float COMMUTATION_STEP_ANGLE = FULL_CYCLE / MECHANICAL_CYCLE_MOD;

bool anticlockwise = false;

float computeSpeed() {
  float elapsedMicrosPerStep = (float) TIME_SINCE_LAST_MEASUREMENT / (float) SPEED_STEP_CTR;

  float microsPerRevolution = elapsedMicrosPerStep * (float) MECHANICAL_CYCLE_MOD;

  // 1 microsseconds = 1 / 6e+7 minutes
  // float minutesPerRevolution = microsPerRevolution / 60000000; // 6e+7
  float rpm = 60000000 / microsPerRevolution;
  RPM = rpm;

  TIME_SINCE_LAST_MEASUREMENT = 0;
  SPEED_STEP_CTR = 0;

  return rpm;
}

bool LED_EN_ON = false;
int LED_STEP_CTR = 0;
int LED_CYCLE_MOD = 66;
boolean STARTUP_MODE = true;

void incrementCommutationState()
{
  ELECTRICAL_STEP_CTR++;
  MECHANICAL_STEP_CTR++;
  if (MECHANICAL_STEP_CTR % MECHANICAL_CYCLE_MOD == 0) {
    computeSpeed();
  }
  ELECTRICAL_STEP_CTR %= ELECTRICAL_CYCLE_MOD;
  MECHANICAL_STEP_CTR %= MECHANICAL_CYCLE_MOD;
  if (STARTUP_MODE == true) { // fast blink mode every electrical cycle
    if (ELECTRICAL_STEP_CTR == 0) {
      if (LED_EN_ON == true) {
        LED_EN_ON = false;
        digitalWriteFast(LED_EN, LOW);
      }
      else if (LED_EN_ON == false) {
        LED_EN_ON = true;
        digitalWriteFast(LED_EN, HIGH);
      }
    }
  }
  else { // slow blink mode every mechanical cycle
    if (MECHANICAL_STEP_CTR == 0) {
      if (LED_EN_ON == true) {
        LED_EN_ON = false;
        digitalWriteFast(LED_EN, LOW);
      }
      else if (LED_EN_ON == false) {
        LED_EN_ON = true;
        digitalWriteFast(LED_EN, HIGH);
      }
    }
  }

  SPEED_STEP_CTR++;
}

float angularDisplacement()
{
  if (anticlockwise == true)
  {
    // subtract our progress from a full rotation
    return FULL_CYCLE - (MECHANICAL_STEP_CTR * COMMUTATION_STEP_ANGLE);
  }
  else
  {
    // positive progression of ctr adds to our angle
    return MECHANICAL_STEP_CTR * COMMUTATION_STEP_ANGLE;
  }
}

float PITCH = 0.0;
float ROLL = 0.0;
byte DUTY_TARGET = MIN_DUTY;
byte NEXT_DUTY = MIN_DUTY;

// ADC MASK CONSTANTS----------------------------------------------------------------------------------------------

// trigger enable/disable masks:

// example
// 00000000|0|1|0|00000|00000000|000|0|000|0|00000111
// ADC_ETC_CTRL = 0x40000007;  // TSC_BYPASS: TSC will control ADC2 directly // trigger 0, 1 and 2 enabled

// enable trigger 0 when phase A is pwming
// |= 00000000|0|0|0|00000|00000000|000|0|000|0|00000001
volatile uint32_t ADC1_ENABLE_TRIG_ON_PHASEA_PWM_MASK = 0x01;
// disable trigger 0
// &= 11111111|1|1|1|11111|11111111|111|1|111|1|11111110
// 0xfffffffffe
volatile uint32_t ADC1_DISABLE_TRIG_ON_PHASEA_PWM_MASK = 0xfffffffffe;

// enable trigger 1 when phase B is pwming
// |= 00000000|0|0|0|00000|00000000|000|0|000|0|00000010
volatile uint32_t ADC1_ENABLE_TRIG_ON_PHASEB_PWM_MASK = 0x02;
// disable trigger 1
// &= 11111111|1|1|1|11111|11111111|111|1|111|1|11111101
volatile uint32_t ADC1_DISABLE_TRIG_ON_PHASEB_PWM_MASK = 0xfffffffffd;

// enable trigger 2 when phase C is pwming
// |= 00000000|0|0|0|00000|00000000|000|0|000|0|00000100
volatile uint32_t ADC1_ENABLE_TRIG_ON_PHASEC_PWM_MASK = 0x04;
// disable trigger 2
// &= 11111111|1|1|1|11111|11111111|111|1|111|1|11111011
volatile uint32_t ADC1_DISABLE_TRIG_ON_PHASEC_PWM_MASK = 0xfffffffffb;

// END ADC MASK CONSTANTS----------------------------------------------------------------------------------------------

// ADC GLOBALS --------------------------------------------------------------------------------------------------------

// adc value holders
volatile uint32_t ADC1_SIGNAL_A, ADC1_SIGNAL_B, ADC1_SIGNAL_C, ADC1_SIGNAL_VN;

// adc chain interrupt handlers
int ADC1_ITER_CTR = 0;
int ADC1_TRIG_CTR = 0;

#define ADC_RINGBUFFER_MOD 3 // this is the number of samples which are averaged per channel
int ADC1_RINGBUFFER_DELTA_A[ADC_RINGBUFFER_MOD];
int ADC1_RINGBUFFER_DELTA_B[ADC_RINGBUFFER_MOD];
int ADC1_RINGBUFFER_DELTA_C[ADC_RINGBUFFER_MOD];
int ADC1_RINGBUFFER_VN[ADC_RINGBUFFER_MOD];

#define ADC_RESOLUTION 8

// END ADC GLOBALS --------------------------------------------------------------------------------------------------------

// COMMUNICATION --------------------------------------------------------------------------------------------------

const int SIZE_OF_FLOAT = sizeof(float); // 4
struct DATA_W
{
  float roll;
  float pitch;
} AXES_STRUCT;

// we read 9 bytes in total
const int SIZE_OF_PROFILE = (SIZE_OF_FLOAT * 2) + 1;

char HOST_PROFILE_BUFFER[SIZE_OF_PROFILE];
byte HOST_PROFILE_BUFFER_CTR = 0;

bool readHostControlProfile()
{
  bool proccessedAFullProfile = false;
  cli();
  while (Serial.available()) {
    HOST_PROFILE_BUFFER[HOST_PROFILE_BUFFER_CTR] = Serial.read();
    HOST_PROFILE_BUFFER_CTR++;

    if (HOST_PROFILE_BUFFER_CTR % SIZE_OF_PROFILE == 0) {
      // unpack profile
      char rollBuffer[SIZE_OF_FLOAT];
      char pitchBuffer[SIZE_OF_FLOAT];
      float roll;
      float pitch;
      char thrust;

      thrust = HOST_PROFILE_BUFFER[0];

      *((unsigned char *)&roll + 0) = HOST_PROFILE_BUFFER[1];
      *((unsigned char *)&roll + 1) = HOST_PROFILE_BUFFER[2];
      *((unsigned char *)&roll + 2) = HOST_PROFILE_BUFFER[3];
      *((unsigned char *)&roll + 3) = HOST_PROFILE_BUFFER[4];

      *((unsigned char *)&pitch + 0) = HOST_PROFILE_BUFFER[5];
      *((unsigned char *)&pitch + 1) = HOST_PROFILE_BUFFER[6];
      *((unsigned char *)&pitch + 2) = HOST_PROFILE_BUFFER[7];
      *((unsigned char *)&pitch + 3) = HOST_PROFILE_BUFFER[8];

      if (thrust == 0) {
        // this is the reset signal
        setup();
      }
      else {
        PITCH = pitch;
        ROLL = roll;
        DUTY_TARGET = max(thrust, MIN_DUTY);
        NEXT_DUTY = DUTY_TARGET;
      }
      proccessedAFullProfile = true;

    }
    HOST_PROFILE_BUFFER_CTR %= SIZE_OF_PROFILE;
  }
  sei();
  return proccessedAFullProfile;
}

void logInfo()
{
  cli(); // halt interrupts
  Serial.print(ADC1_RINGBUFFER_DELTA_A[1]);
  Serial.print("\t"); //  + ADC1_RINGBUFFER_VN[1]
  Serial.print(ADC1_RINGBUFFER_DELTA_B[1]);
  Serial.print("\t"); //  + ADC1_RINGBUFFER_VN[1]
  Serial.print(ADC1_RINGBUFFER_DELTA_C[1]);
  Serial.print("\t"); // + ADC1_RINGBUFFER_VN[1]
  Serial.print(ADC1_RINGBUFFER_VN[1]);
  Serial.print("\t");
  Serial.print(LOW);
  Serial.print("\t");
  Serial.print(LOW);
  Serial.print("\t");
  Serial.print(LOW);
  Serial.print("\t");
  Serial.print(MECHANICAL_STEP_CTR);
  Serial.print("\t");
  Serial.print(ROLL);
  Serial.print("\t");
  Serial.print(PITCH);
  Serial.print("\t");
  Serial.print(DUTY_TARGET);
  Serial.print("\t");
  Serial.print(RPM);
  Serial.print("\n");
  sei();
}

// END COMMUNICATION  ---------------------------------------------------------------------------------------------

// XBAR  ----------------------------------------------------------------------------------------------------------

void xbarConnect(unsigned int input, unsigned int output)
{
  if (input >= 88)
    return;
  if (output >= 132)
    return;
  volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
  uint16_t val = *xbar;
  if (!(output & 1))
  {
    val = (val & 0xFF00) | input;
  }
  else
  {
    val = (val & 0x00FF) | (input << 8);
  }
  *xbar = val;
}

void xbarInit()
{
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON); //turn clock on for xbara1
  // connect xbar pwm trigger output for the three pwm signals and pipe to adc etc trigger input
  xbarConnect(XBARA1_IN_FLEXPWM1_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG00); // pwm module 1.0 pin 1 phase A PWMING
  xbarConnect(XBARA1_IN_FLEXPWM1_PWM2_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG01); // pwm module 1.1 pin 0 phase B PWMING
  xbarConnect(XBARA1_IN_FLEXPWM4_PWM1_OUT_TRIG0, XBARA1_OUT_ADC_ETC_TRIG02); // pwm module 4.0 pin 22 phase C PWMING
}

// END XBAR  ------------------------------------------------------------------------------------------------------

// MOTION    ------------------------------------------------------------------------------------------------------

// ADC1_TRIG_CTR 0 A
// ADC1_TRIG_CTR 1 B
// ADC1_TRIG_CTR 2 C


void AH_BL()
{
  // disable adc trigger on phaseA pwm, phaseC pwm
  ADC_ETC_CTRL &= ADC1_DISABLE_TRIG_ON_PHASEA_PWM_MASK | ADC1_DISABLE_TRIG_ON_PHASEC_PWM_MASK;

  digitalWriteFast(C_IN, LOW);
  digitalWriteFast(B_IN, LOW);
  // turn on A_IN
  digitalWriteFast(A_IN, HIGH);

  digitalWriteFast(A_SD, LOW);
  digitalWriteFast(C_SD, LOW);
  // turn on / keep on PWM for B
  analogWrite(B_SD, NEXT_DUTY);

  // enable adc trigger on phaseB pwm
  ADC1_TRIG_CTR = 1;
  ADC_ETC_CTRL |= ADC1_ENABLE_TRIG_ON_PHASEB_PWM_MASK;
} // A_IN HIGH B_SB PWM

void AH_CL()
{
  // disable adc trigger on phaseA pwm, phaseB pwm
  ADC_ETC_CTRL &= ADC1_DISABLE_TRIG_ON_PHASEA_PWM_MASK | ADC1_DISABLE_TRIG_ON_PHASEB_PWM_MASK;

  digitalWriteFast(B_IN, LOW);
  digitalWriteFast(C_IN, LOW);
  // keep A_IN on
  digitalWriteFast(A_IN, HIGH);


  digitalWriteFast(A_SD, LOW);
  digitalWriteFast(B_SD, LOW);
  // turn on PWM for C
  analogWrite(C_SD, NEXT_DUTY);

  // enable adc trigger on phaseC pwm
  ADC1_TRIG_CTR = 2;
  ADC_ETC_CTRL |= ADC1_ENABLE_TRIG_ON_PHASEC_PWM_MASK;
} // A_IN high C_SD PWM

void BH_CL()
{
  // disable adc trigger on phaseA pwm, phaseB pwm
  ADC_ETC_CTRL &= ADC1_DISABLE_TRIG_ON_PHASEA_PWM_MASK | ADC1_DISABLE_TRIG_ON_PHASEB_PWM_MASK;

  digitalWriteFast(C_IN, LOW);
  // turn B_IN on
  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(B_IN, HIGH);

  digitalWriteFast(A_SD, LOW);
  digitalWriteFast(B_SD, LOW);
  // keep PWM on for C
  analogWrite(C_SD, NEXT_DUTY);

  // enable adc trigger on phaseC pwm
  ADC1_TRIG_CTR = 2;
  ADC_ETC_CTRL |= ADC1_ENABLE_TRIG_ON_PHASEC_PWM_MASK;
} // B_IN high C_SD PWM

void BH_AL()
{
  // disable adc trigger on phaseB pwm, phaseC pwm
  ADC_ETC_CTRL &= ADC1_ENABLE_TRIG_ON_PHASEB_PWM_MASK | ADC1_ENABLE_TRIG_ON_PHASEC_PWM_MASK;

  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(C_IN, LOW);
  // keep B_IN on
  digitalWriteFast(B_IN, HIGH);

  digitalWriteFast(B_SD, LOW);
  digitalWriteFast(C_SD, LOW);
  // turn pwn for A
  analogWrite(A_SD, NEXT_DUTY);

  // enable adc trigger on phaseA pwm
  ADC1_TRIG_CTR = 0;
  ADC_ETC_CTRL |= ADC1_ENABLE_TRIG_ON_PHASEA_PWM_MASK;
} // B_IN high A_SD PWM

void CH_AL()
{
  // disable adc trigger on phaseB pwm, phaseC pwm
  ADC_ETC_CTRL &= ADC1_ENABLE_TRIG_ON_PHASEB_PWM_MASK | ADC1_ENABLE_TRIG_ON_PHASEC_PWM_MASK;

  digitalWriteFast(A_IN, LOW);
  // turn C_IN on
  digitalWriteFast(B_IN, LOW);
  digitalWriteFast(C_IN, HIGH);

  digitalWriteFast(B_SD, LOW);
  digitalWriteFast(C_SD, LOW);
  // keep PWM on for A
  analogWrite(A_SD, NEXT_DUTY);

  // enable adc trigger on phaseA pwm
  ADC1_TRIG_CTR = 0;
  ADC_ETC_CTRL |= ADC1_ENABLE_TRIG_ON_PHASEA_PWM_MASK;
} // C_IN high A_SD PWM

void CH_BL()
{
  // disable adc trigger on phaseA pwm, phaseC pwm
  ADC_ETC_CTRL &= ADC1_DISABLE_TRIG_ON_PHASEA_PWM_MASK | ADC1_ENABLE_TRIG_ON_PHASEC_PWM_MASK;

  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(B_IN, LOW);
  // keep c_in high
  digitalWriteFast(C_IN, HIGH);

  digitalWriteFast(C_SD, LOW);
  // turn pwm on for B
  digitalWriteFast(A_SD, LOW);
  analogWrite(B_SD, NEXT_DUTY);

  // enable adc trigger on phaseB pwm
  ADC1_TRIG_CTR = 2;
  ADC_ETC_CTRL |= ADC1_ENABLE_TRIG_ON_PHASEB_PWM_MASK;
  // C_IN HIGH B_SD PWM
}

void enforceCommutationState()
{
  switch (ELECTRICAL_STEP_CTR)
  {
    case 0:
      if (anticlockwise == false)
      {
        AH_BL(); // C RISING
      }
      else
      {
        CH_BL(); // A RISING
      }
      break;
    case 1:
      if (anticlockwise == false)
      {
        AH_CL(); // B FALLING
      }
      else
      {
        CH_AL(); // B FALLING
      }
      break;
    case 2:
      if (anticlockwise == false)
      {
        BH_CL(); // A RISING
      }
      else
      {
        BH_AL(); // C RISING
      }
      break;
    case 3:
      if (anticlockwise == false)
      {
        BH_AL(); // C FALLING
      }
      else
      {
        BH_CL(); // A FALLING
      }
      break;
    case 4:
      if (anticlockwise == false)
      {
        CH_AL(); // B RISING
      }
      else
      {
        AH_CL(); // B RISING
      }
      break;
    case 5:
      if (anticlockwise == false)
      {
        CH_BL(); // A FALLING
      }
      else
      {
        AH_BL(); // C FALLING
      }
      break;
  }
}

bool LAST_LOOP_WAS_STARTUP = false;
void startup()
{
  if (STARTUP_MODE == false)
  {
    return;
  }
  // do a linear chirp to force initial motor commutation as we lack a bemf trigger
  int i, x = 0;
  while (x < 1)
  {
    i = 5000;
    while (i > 20)
    {
      if (STARTUP_MODE == false)
      { // allow for early bemf detection exit
        return;
      }
      delayMicroseconds(i);
      incrementCommutationState();
      enforceCommutationState();
      i = i - 20;
    }
    x++;
    /*if (DEBUG_MODE == true)
    { // FIXME probably disable this if it messes with the timings
      logInfo();
    }*/
  }
  STARTUP_MODE = false;
  LAST_LOOP_WAS_STARTUP = true;
}

// END MOTION -----------------------------------------------------------------------------------------------------

// BEMF -----------------------------------------------------------------------------------------------------------

// ADC1_DELTA_A
// ADC1_DELTA_B
// ADC1_DELTA_C

int ADC1_RINGBUFFER_CTR = 0;

void eliminateDropoutFromBuffer()
{
  // we have a dropouts eliminate values at index 1 and replace with values from index 2 and poll for another value
  ADC1_RINGBUFFER_DELTA_A[1] = ADC1_RINGBUFFER_DELTA_A[2];
  ADC1_RINGBUFFER_DELTA_B[1] = ADC1_RINGBUFFER_DELTA_B[2];
  ADC1_RINGBUFFER_DELTA_C[1] = ADC1_RINGBUFFER_DELTA_C[2];
  ADC1_RINGBUFFER_VN[1] = ADC1_RINGBUFFER_VN[2];
  ADC1_RINGBUFFER_CTR = 2;
}

void advanceBuffer()
{
  // no detection of zero crossing .... now ADC1_RINGBUFFER_DELTA_X[2] could have the vital cross over datapoint so make sure we preserve it and measure it. So we need to move it to ADC1_RINGBUFFER_DELTA_X[1]
  ADC1_RINGBUFFER_DELTA_A[0] = ADC1_RINGBUFFER_DELTA_A[1];
  ADC1_RINGBUFFER_DELTA_B[0] = ADC1_RINGBUFFER_DELTA_B[1];
  ADC1_RINGBUFFER_DELTA_C[0] = ADC1_RINGBUFFER_DELTA_C[1];
  ADC1_RINGBUFFER_VN[0] = ADC1_RINGBUFFER_VN[1];

  ADC1_RINGBUFFER_DELTA_A[1] = ADC1_RINGBUFFER_DELTA_A[2];
  ADC1_RINGBUFFER_DELTA_B[1] = ADC1_RINGBUFFER_DELTA_B[2];
  ADC1_RINGBUFFER_DELTA_C[1] = ADC1_RINGBUFFER_DELTA_C[2];
  ADC1_RINGBUFFER_VN[1] = ADC1_RINGBUFFER_VN[2];

  ADC1_RINGBUFFER_CTR = 2;
}

bool processBEMF()
{
  bool zeroCrossingDetected = false;

  ADC1_RINGBUFFER_DELTA_A[ADC1_RINGBUFFER_CTR] = ADC1_SIGNAL_A - ADC1_SIGNAL_VN;
  ADC1_RINGBUFFER_DELTA_B[ADC1_RINGBUFFER_CTR] = ADC1_SIGNAL_B - ADC1_SIGNAL_VN;
  ADC1_RINGBUFFER_DELTA_C[ADC1_RINGBUFFER_CTR] = ADC1_SIGNAL_C - ADC1_SIGNAL_VN;
  ADC1_RINGBUFFER_VN[ADC1_RINGBUFFER_CTR] = ADC1_SIGNAL_VN;

  ADC1_RINGBUFFER_CTR++;
  ADC1_RINGBUFFER_CTR %= ADC_RINGBUFFER_MOD;

  // if we are at the end of the ring buffer attempt zero crossing detection
  if (ADC1_RINGBUFFER_CTR == 0)
  {
    /*
      So the ADC values can randomly drop out to zero, we need to detect these and ignore these measurements.
      we will check if a measurement has value zero and its adjacent measurements > 0 then we can assume this is an outlier as the bemf is unlikely to stall and restore within 3 measurements
    */

    if (anticlockwise == true)
    {
      switch (ELECTRICAL_STEP_CTR)
      {
        case 0: // C RISING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_C[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_C[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_C[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_C[0] <= 0) && (ADC1_RINGBUFFER_DELTA_C[1] > 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 1: // B FALLING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_B[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_B[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_B[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_B[0] > 0) && (ADC1_RINGBUFFER_DELTA_B[1] <= 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 2: // A RISING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_A[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_A[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_A[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_A[0] <= 0) && (ADC1_RINGBUFFER_DELTA_A[1] > 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 3: // C FALLING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_C[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_C[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_C[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_C[0] > 0) && (ADC1_RINGBUFFER_DELTA_C[1] <= 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 4: // B RISING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_B[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_B[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_B[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_B[0] <= 0) && (ADC1_RINGBUFFER_DELTA_B[1] > 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 5: // A FALLING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_A[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_A[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_A[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_A[0] > 0) && (ADC1_RINGBUFFER_DELTA_A[1] <= 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
      }
    }
    else
    { // clockwise
      switch (ELECTRICAL_STEP_CTR)
      {
        case 0: // A RISING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_A[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_A[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_A[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_A[0] <= 0) && (ADC1_RINGBUFFER_DELTA_A[1] > 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 1: // B FALLING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_B[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_B[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_B[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_B[0] > 0) && (ADC1_RINGBUFFER_DELTA_B[1] <= 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 2: // C RISING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_C[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_C[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_C[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_C[0] <= 0) && (ADC1_RINGBUFFER_DELTA_C[1] > 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 3: // A FALLING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_A[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_A[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_A[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_A[0] > 0) && (ADC1_RINGBUFFER_DELTA_A[1] <= 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 4: // B RISING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_B[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_B[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_B[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_B[0] <= 0) && (ADC1_RINGBUFFER_DELTA_B[1] > 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
        case 5: // C FALLING
          // if middle sample was measure at exactly zero and first and second samples are +ve.
          if ((ADC1_RINGBUFFER_DELTA_C[1] == -ADC1_RINGBUFFER_VN[1]) && ADC1_RINGBUFFER_DELTA_C[0] > ADC1_RINGBUFFER_VN[0] && ADC1_RINGBUFFER_DELTA_C[2] > ADC1_RINGBUFFER_VN[2])
          {
            eliminateDropoutFromBuffer();
          }
          else
          {
            if ((ADC1_RINGBUFFER_DELTA_C[0] > 0) && (ADC1_RINGBUFFER_DELTA_C[1] <= 0))
            {
              zeroCrossingDetected = true; // leave ADC1_RINGBUFFER_CTR 0 so it resets
            }
            else
            {
              advanceBuffer();
            }
          }
          break;
      }
    }
  }
  return zeroCrossingDetected;
}

void handleZeroCrossing()
{

  // commute motor step
  incrementCommutationState();
  enforceCommutationState();

  // we have some time before the next change of state...

  if (DEBUG_MODE == true)
  {
    cli(); // halt interrupts
    Serial.print(ADC1_RINGBUFFER_DELTA_A[1]);
    Serial.print("\t"); //  + ADC1_RINGBUFFER_VN[1]
    Serial.print(ADC1_RINGBUFFER_DELTA_B[1]);
    Serial.print("\t"); //  + ADC1_RINGBUFFER_VN[1]
    Serial.print(ADC1_RINGBUFFER_DELTA_C[1]);
    Serial.print("\t"); //  + ADC1_RINGBUFFER_VN[1]
    Serial.print(ADC1_RINGBUFFER_VN[1]);
    Serial.print("\t");

    // we are past the incrementor so we have advanced one step beyond what we are printing.
    auto stepnow = ELECTRICAL_STEP_CTR - 1;
    if (stepnow == -1)
    {
      stepnow = ELECTRICAL_CYCLE_MOD - 1; //ELECTRICAL_CYCLE_MOD
    }

    switch (stepnow)
    {
      case 0:
        if (anticlockwise == true)
        {
          // AH_BL() C RISING
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(10);
          Serial.print("\t");
        }
        else
        {
          // CH_BL() A RISING
          Serial.print(10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        break;
      case 1:
        // AH_CL()  B FALLING
        if (anticlockwise == true)
        {
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(-10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        else
        {
          // CH_AL() B FALLING
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(-10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        break;
      case 2:
        if (anticlockwise == true)
        {
          // BH_CL()  A RISING
          Serial.print(10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        else
        {
          //BH_AL() C RISING
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(10);
          Serial.print("\t");
        }
        break;
      case 3:
        if (anticlockwise == true)
        {
          // BH_AL() C FALLING
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(-10);
          Serial.print("\t");
        }
        else
        {
          // BH_CL() A FALLING
          Serial.print(-10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        break;

      case 4:
        if (anticlockwise == true)
        {
          // CH_AL() B RISING
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        else
        {
          // AH_CL() B RISING
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        break;
      case 5:
        if (anticlockwise == true)
        {
          // CH_BL() A FALLING
          Serial.print(-10);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
        }
        else
        {
          // AH_BL() C FALLING
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(LOW);
          Serial.print("\t");
          Serial.print(-10);
          Serial.print("\t");
        }
        break;
    }
    Serial.print(MECHANICAL_STEP_CTR);
    Serial.print("\t");
    Serial.print(ROLL);
    Serial.print("\t");
    Serial.print(PITCH);
    Serial.print("\t");
    Serial.print(DUTY_TARGET);
    Serial.print("\t");
    Serial.print(RPM);
    Serial.print("\n");
    sei();
  }
}

// END BEMF -----------------------------------------------------------------------------------------------------------

// ADC  -----------------------------------------------------------------------------------------------------------

void adcetc0_isr()
{
  ADC_ETC_DONE0_1_IRQ |= 1; // clear
  if (ADC1_TRIG_CTR == 0)
  {
    if (ADC1_ITER_CTR == 0)
    {
      ADC1_SIGNAL_A = ADC_ETC_TRIG0_RESULT_1_0 & 4095;
    }
    else
    {
      ADC1_SIGNAL_VN = ADC_ETC_TRIG0_RESULT_3_2 & 4095;
    }
  }
  else if (ADC1_TRIG_CTR == 1)
  {
    if (ADC1_ITER_CTR == 0)
    {
      ADC1_SIGNAL_A = ADC_ETC_TRIG1_RESULT_1_0 & 4095;
    }
    else
    {
      ADC1_SIGNAL_VN = ADC_ETC_TRIG1_RESULT_3_2 & 4095;
    }
  }
  else if (ADC1_TRIG_CTR == 2)
  {
    if (ADC1_ITER_CTR == 0)
    {
      ADC1_SIGNAL_A = ADC_ETC_TRIG2_RESULT_1_0 & 4095;
    }
    else
    {
      ADC1_SIGNAL_VN = ADC_ETC_TRIG2_RESULT_3_2 & 4095;
    }
  }
  asm("dsb");
  ADC1_ITER_CTR++;
}
void adcetc1_isr()
{
  ADC_ETC_DONE0_1_IRQ |= 1 << 16; // clear
  if (ADC1_TRIG_CTR == 0)
  {
    if (ADC1_ITER_CTR == 1)
    {
      ADC1_SIGNAL_B = (ADC_ETC_TRIG0_RESULT_1_0 >> 16) & 4095;
    }
    else
    {
      ADC1_SIGNAL_C = (ADC_ETC_TRIG0_RESULT_3_2 >> 16) & 4095;
    }
  }
  else if (ADC1_TRIG_CTR == 1)
  {
    if (ADC1_ITER_CTR == 1)
    {
      ADC1_SIGNAL_B = (ADC_ETC_TRIG1_RESULT_1_0 >> 16) & 4095;
    }
    else
    {
      ADC1_SIGNAL_C = (ADC_ETC_TRIG1_RESULT_3_2 >> 16) & 4095;
    }
  }
  else if (ADC1_TRIG_CTR == 2)
  {
    if (ADC1_ITER_CTR == 1)
    {
      ADC1_SIGNAL_B = (ADC_ETC_TRIG2_RESULT_1_0 >> 16) & 4095;
    }
    else
    {
      ADC1_SIGNAL_C = (ADC_ETC_TRIG2_RESULT_3_2 >> 16) & 4095;
    }
  }
  asm("dsb");
  ADC1_ITER_CTR++;
  // handle possible zero crossing now that we have the results of our 4 adc channels A,B,C,VN
  if (ADC1_ITER_CTR > 3)
  {
    ADC1_ITER_CTR = 0;
    bool zeroCrossing = processBEMF();
    if (STARTUP_MODE == false && zeroCrossing == true)
    {
      handleZeroCrossing();
    }
  }
}

ADC *adc = new ADC();
void adcPreConfigure()
{
  adc->adc0->setAveraging(1);
  adc->adc0->setResolution(ADC_RESOLUTION);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
}

void adcInit()
{
  // init and calibrate
  analogReadResolution(ADC_RESOLUTION);
  analogRead(0);
  analogRead(1);
  // hardware trigger
  ADC1_CFG |= ADC_CFG_ADTRG;
  // External channel selection from ADC_ETC we need four channels per motor
  // motor 1 ADC 1 channel 0 -> 3
  ADC1_HC0 = 16;
  ADC1_HC1 = 16;
  ADC1_HC2 = 16;
  ADC1_HC3 = 16;
}

void adcEtcInit()
{
  ADC_ETC_CTRL &= ~(1 << 31); // SOFTRST

  // TSC_BYPASS 1
  ADC_ETC_CTRL = 0x40000000; // TSC_BYPASS 1: TSC will control ADC2 directly // trigger 0, 1 and 2 disabled

  // ADC CHANNELS WE NEED
  // 14/A0 AD_B1_02  ADC1_IN7  Analog channel 1 input 7
  // 15/A1 AD_B1_03  ADC1_IN8  Analog channel 1 input 8
  // 16/A2 AD_B1_07  ADC1_IN12 Analog channel 1 input 12
  // 17/A3 AD_B1_06  ADC1_IN11 Analog channel 1 input 11

  // setup adc trigger chain.

  ADC_ETC_TRIG0_CHAIN_1_0 = 0x50283017; // ADC1 7 8 adc channels, HWTS, IE, B2B;
  ADC_ETC_TRIG1_CHAIN_1_0 = 0x50283017; // ADC1 7 8 adc channels, HWTS, IE, B2B;
  ADC_ETC_TRIG2_CHAIN_1_0 = 0x50283017; // ADC1 7 8 adc channels, HWTS, IE, B2B;
  // TRIG(0/1/2)_CHAIN 1: Finished Interrupt on Done1, Enable B2B, the next ADC trigger will be sent as soon as possible. ADC hardware trigger selection:2, ADC channel selection 8
  // TRIG(0/1/2)_CHAIN 0: Finished Interrupt on Done0, Enable B2B, the next ADC trigger will be sent as soon as possible. ADC hardware trigger selection:1, ADC channel selection 7

  ADC_ETC_TRIG0_CHAIN_3_2 = 0x504c303b; // ADC1 11 12, chain channel, HWTS, IE, B2B;
  ADC_ETC_TRIG1_CHAIN_3_2 = 0x504c303b; // ADC1 11 12, chain channel, HWTS, IE, B2B;
  ADC_ETC_TRIG2_CHAIN_3_2 = 0x504c303b; // ADC1 11 12, chain channel, HWTS, IE, B2B;
  // TRIG(0/1/2)_CHAIN 3: Finished Interrupt on Done1, Enable B2B, the next ADC trigger will be sent as soon as possible. ADC hardware trigger selection:4, ADC channel selection 12
  // TRIG(0/1/2) CHAIN 2: Finished Interrupt on Done0, Enable B2B, the next ADC trigger will be sent as soon as possible. ADC hardware trigger selection:3, ADC channel selection 11

  // enable the triggers
  /*
    000000000000000000000|001|000|0|000|0 // 1) chain of 2 x100
    000000000000000000000|010|000|0|000|0 // 2) chain of 3 x200
    000000000000000000000|011|000|0|000|0 // 3) chain of 4 x300
  */
  ADC_ETC_TRIG0_CTRL = 0x300; // TRIG 0 chain length to the ADC Chain = 4
  ADC_ETC_TRIG1_CTRL = 0x300; // TRIG 1 chain length to the ADC Chain = 4
  ADC_ETC_TRIG2_CTRL = 0x300; // TRIG 2 chain length to the ADC Chain = 4

  // enable adc interrupt callbacks
  attachInterruptVector(IRQ_ADC_ETC0, adcetc0_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC0);
  attachInterruptVector(IRQ_ADC_ETC1, adcetc1_isr);
  NVIC_ENABLE_IRQ(IRQ_ADC_ETC1);
}

// END ADC  -----------------------------------------------------------------------------------------------------------

// PWM CTRL -----------------------------------------------------------------------------------------------------------

void pwmInit_1_0()
{ // this works ok pin 1
  analogWriteFrequency(1, PWM_FREQUENCY);
  FLEXPWM1_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 3);
}

// sync clocks of flexpwm1_SM0 and flexpwm1_SM1
/*
  CLK_SEL
  Clock Source Select
  These read/write bits determine the source of the clock signal for this submodule.
  00b - The IPBus clock is used as the clock for the local prescaler and counter.
  01b - EXT_CLK is used as the clock for the local prescaler and counter.
  10b - Submodule 0’s clock (AUX_CLK) is used as the source clock for the local prescaler and
  counter. This setting should not be used in submodule 0 as it will force the clock to logic 0.
  11b - reserved
*/
void pwmInit_1_1()
{ // this works ok pin 0
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_CLDOK(0x0F);                            //  Clear Load Okay LDOK(SM) -> no reload of PWM settings
  FLEXPWM1_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_CLK_SEL(2); //A & B independant | sm0 chosen as clock
  FLEXPWM1_MCTRL |= FLEXPWM_MCTRL_LDOK(0x0F);                             // Load Okay LDOK(SM) -> reload setting again
  FLEXPWM1_SM1TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 3);
}

void pwmInit_4_0()
{ //set PWM pin 22
  analogWriteFrequency(22, PWM_FREQUENCY);
  FLEXPWM4_SM0TCTRL = FLEXPWM_SMTCTRL_OUT_TRIG_EN(1 << 3);
}

void pwmInit()
{
  analogWriteRes(8);
  pwmInit_1_0();
  pwmInit_1_1();
  pwmInit_4_0();
}

// END PWM CTRL -----------------------------------------------------------------------------------------------------------

void setup()
{
  pinMode(A_IN, OUTPUT);
  pinMode(B_IN, OUTPUT);
  pinMode(C_IN, OUTPUT);
  pinMode(A_SD, OUTPUT);
  pinMode(B_SD, OUTPUT);
  pinMode(C_SD, OUTPUT);
  pinMode(LED_EN, OUTPUT);
  DUTY_TARGET = MIN_DUTY;
  NEXT_DUTY = MIN_DUTY;
  adcPreConfigure();
  xbarInit();
  adcInit();
  adcEtcInit();
  pwmInit();
  // force off everything
  digitalWriteFast(A_IN, LOW);
  digitalWriteFast(B_IN, LOW);
  digitalWriteFast(C_IN, LOW);
  analogWrite(A_SD, LOW);
  analogWrite(B_SD, LOW);
  analogWrite(C_SD, LOW);
  STARTUP_MODE = true;
  LAST_LOOP_WAS_STARTUP = false;
}


float CACHE_PITCH = 0.0;
float CACHE_ROLL = 0.0;
byte CACHE_DUTY_TARGET = 0;

void loop()
{
  // delay until we a signal
  if (STARTUP_MODE == true) {
    digitalWriteFast(LED_EN, LOW);
    LED_EN_ON = false;
    while (!readHostControlProfile()) {
      delay(100);
    }
    // cache read profile
    CACHE_PITCH = PITCH;
    CACHE_ROLL = ROLL;
    CACHE_DUTY_TARGET = DUTY_TARGET;
    // override host settings, sorry host but unless u know what ur doing, everythings gonna explode.
    PITCH = 0.0;
    ROLL = 0.0;
    DUTY_TARGET = MIN_DUTY;
    NEXT_DUTY = MIN_DUTY;
    digitalWriteFast(LED_EN, HIGH);
    LED_EN_ON = true;
  }

  startup();


  if (LAST_LOOP_WAS_STARTUP == true) {
    // restore host profile if it is safe to do so
    if (CACHE_DUTY_TARGET < 30) {
      PITCH = CACHE_PITCH;
      ROLL = CACHE_ROLL;
      DUTY_TARGET = CACHE_DUTY_TARGET;
      NEXT_DUTY = DUTY_TARGET;
    }
    LAST_LOOP_WAS_STARTUP = false;
    digitalWriteFast(LED_EN, LOW);
    LED_EN_ON = false;
  }


  delayMicroseconds(200);

  if (DEBUG_MODE == true)
  {
    logInfo();
  }

  readHostControlProfile();
}
