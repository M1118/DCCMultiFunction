
#include <NmraDcc.h>
#include <Servo.h>
#include <DCCServo.h>
#include <DCCStepper.h>
#include <DCCLight.h>


/*
 *  DCC Mixed Device Decoder
 *
 *  A DCC decoder designed to drive a variety of devices, a mix of RC Servos, Stepper Motor and lights
 */
#define DCC_VERSION_ID  22

#define DEBUG           0    // Enable debug output on the serial interface
#define DEBUGCV         0    // Enable output of CV relates debug
#define DEBUGACK        0    // Enable debug for sending DCC Acknowledge
#define DEBUGNOTIFY     0    // DCC Funcution and Speed notifications
#define DEBUGSETP       0    // Debug Set piece execution
#define BAUDRATE     9600

#define STAGGER_SERVOS  0    // Enables staggering of the attach of servos
#define SERVO_ACK       0    // Use servo0 for the ack pulse
#define STEPPER_ACK     0

/* Board types - define only one of the to be 1, others should be 0 */
#define V1_PINS                0    // First David Hills Board
#define NANO_PIGGYBACK_PINS    0    // Nano piggy back thru-hole board
#define V2_PROTOTYPE_PINS      1    // Nano Stripboard prototype
#define ALL_IN_ONE_PINS        0    // D Hills first all in one board


/*
 *  Arduino pin assignments
 */
#if V1_PINS
const int DccAckPin     = 3;
const int DccInPin      = 2;
const int ControlPin    = 9;
const int DccIntNum     = 0;
const int LedPin        = 13;
const int servoPins[]   = { 4, 5, 6 };
const int stepperPins[] = { A2, A3, A4, A5 };
const int lightPins[]   = { 7, 8, 10, 11 };
#define LEDFEEDBACK           0
#elif NANO_PIGGYBACK_PINS
const int DccAckPin     = 4;
const int DccInPin      = 3;
const int ControlPin    = 2;
const int DccIntNum     = 1;
const int LedPin        = 13;
const int servoPins[]   = { 5, 6, 7 };
const int stepperPins[] = { A2, A3, A4, A5 };
const int lightPins[]   = { 8, 10, 11, 12 };
#define LEDFEEDBACK           0
#elif ALL_IN_ONE_PINS
const int DccAckPin     = 2;
const int DccInPin      = 3;
const int ControlPin    = 4;
const int DccIntNum     = 1;
const int LedPin        = 13;
const int servoPins[]   = { 5, 6, 7 };
const int stepperPins[] = { A2, A3, A4, A5 };
const int lightPins[]   = { 8, 10, 11, 12 };
#define LEDFEEDBACK           0
#elif V2_PROTOTYPE_PINS
const int DccAckPin     = 3;
const int DccInPin      = 2;
const int ControlPin    = 13;
const int DccIntNum     = 0;
const int LedPin        = 12;
const int servoPins[]   = { 4, 5, 6 };
const int stepperPins[] = { A0, A1, A2, A3 };
const int lightPins[]   = { 7, 8, 9, 10 };
#define LEDFEEDBACK           0
#else
#error Must define the board type
#endif

const unsigned long AckLength       = 6;  // Nominal ack lenth in mS
const unsigned int  AckAdditionaluS = 100; // Addition time in uS to strech ack pulse

/*
   Set the below to 0 if an ack is sent by taking the DccAckPin high
   Set the below to 1 if an ack is sent by taking the DccAckPin low
*/
#define INVERTED_ACK  0
/*
   Set the below to 0 if servo power is on by taking CONTROL high
   Set the below to 1 if servo power is on by taking CONTROL low
*/
#define INVERTED_CTL  0

unsigned long startms = 0;
boolean poweron = false;
boolean powerWasOn = false;
boolean inServiceMode = false;
boolean dcc_watchdog = false;
int ledState = LOW;
int powerNotify = 0;
unsigned long skip_millis = 0;

NmraDcc     Dcc;
DCC_MSG     Packet;

DCCServo    *servo1 = NULL, *servo2 = NULL, *servo3 = NULL;
DCCStepper  *stepper = NULL;
DCCLight    *light0 = NULL, *light1 = NULL, *light2 = NULL, *light3 = NULL;
boolean      motorAttached = false;

extern void setPieceAdvance();
extern void setSpeed(int percentage, boolean forward);
/*
   Items related to the CV's that control the behavior of the
   decoder
*/
// CV Storage structure
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

int MyAddress;

/*
   The CV's that are used for the decoder
*/
#define CV_S0LIMIT0    30
#define CV_S0LIMIT1    31
#define CV_S0TRAVEL    32
#define CV_S0FLAGS     33
#define CV_S0FUNC      34
#define CV_S0BOUNCE    35
#define CV_S0POSITION  36
#define CV_S1LIMIT0    40
#define CV_S1LIMIT1    41
#define CV_S1TRAVEL    42
#define CV_S1FLAGS     43
#define CV_S1FUNC      44
#define CV_S1BOUNCE    45
#define CV_S1POSITION  36
#define CV_S2LIMIT0    50
#define CV_S2LIMIT1    51
#define CV_S2TRAVEL    52
#define CV_S2FLAGS     53
#define CV_S2FUNC      54
#define CV_S2BOUNCE    55
#define CV_S2POSITION  36
#define CV_STEPS       60
#define CV_RATIO       61
#define CV_MAXRPM      62
#define CV_STEPFUNC    63
#define CV_STEPMODE    64
#define CV_MAXLSB      65
#define CV_MAXMSB      66
#define CV_STEPDELAY   67
#define CV_CURLSB      68
#define CV_CURMSB      69
#define CV_MAXPWM      70
#define CV_PWMFUNC     71
#define CV_PWRDELAY    72
#define CV_L0EFFECT    75
#define CV_L0PERIOD    76
#define CV_L0FUNC      77
#define CV_L1EFFECT    80
#define CV_L1PERIOD    81
#define CV_L1FUNC      82
#define CV_L2EFFECT    85
#define CV_L2PERIOD    86
#define CV_L2FUNC      87
#define CV_L3EFFECT    90
#define CV_L3PERIOD    91
#define CV_L3FUNC      92

/* Set piece CV's */
#define CV_SP1STEPS   100
#define CV_SP1FN      101 // Function numebr to trigger set piece
#define CV_SP1S1F     105 // Function values in step 1
#define CV_SP1S1T     106 // Throttle setting in step 1
#define CV_SP1S1DH    107 // High byte of duration of step 1
#define CV_SP1S1DL    108 // Low byte of duration of step 1
#define CV_SP1S2F     109 // Function values in step 2
#define CV_SP1S2T     110 // Throttle setting in step 2
#define CV_SP1S2DH    111 // High byte of duration of step 2
#define CV_SP1S2DL    112 // Low byte of duration of step 2
#define CV_SP1S3F     113 // Function values in step 3
#define CV_SP1S3T     114 // Throttle setting in step 3
#define CV_SP1S3DH    115 // High byte of duration of step 3
#define CV_SP1S3DL    116 // Low byte of duration of step 3
#define CV_SP1S4F     117 // Function values in step 4
#define CV_SP1S4T     118 // Throttle setting in step 4
#define CV_SP1S4DH    119 // High byte of duration of step 4
#define CV_SP1S4DL    120 // Low byte of duration of step4
#define CV_SP1S5F     121 // Function values in step 5
#define CV_SP1S5T     122 // Throttle setting in step 5
#define CV_SP1S5DH    123 // High byte of duration of step 5
#define CV_SP1S5DL    124 // Low byte of duration of step 5
#define CV_SP1S6F     125 // Function values in step 6
#define CV_SP1S6T     126 // Throttle setting in step 6
#define CV_SP1S6DH    127 // High byte of duration of step 6
#define CV_SP1S6DL    128 // Low byte of duration of step 6
#define CV_SP1S7F     129 // Function values in step 7
#define CV_SP1S7T     130 // Throttle setting in step 7
#define CV_SP1S7DH    131 // High byte of duration of step 7
#define CV_SP1S7DL    132 // Low byte of duration of step 7
#define CV_SP1S8F     133 // Function values in step 8
#define CV_SP1S8T     134 // Throttle setting in step 8
#define CV_SP1S8DH    135 // High byte of duration of step 8
#define CV_SP1S8DL    136 // Low byte of duration of step 8
#define CV_SP1S9F     137 // Function values in step 9
#define CV_SP1S9T     138 // Throttle setting in step 9
#define CV_SP1S9DH    139 // High byte of duration of step 9
#define CV_SP1S9DL    140 // Low byte of duration of step 9
#define CV_SP1S10F    141 // Function values in step 10
#define CV_SP1S10T    142 // Throttle setting in step 10
#define CV_SP1S10DH   143 // High byte of duration of step 10
#define CV_SP1S10DL   144 // Low byte of duration of step 10

/*
   The factory default CV values
*/
CVPair FactoryDefaultCVs [] =
{
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 3},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, 3},
  {CV_ACCESSORY_DECODER_ADDRESS_MSB, 0},
  {CV_ACCESSORY_DECODER_ADDRESS_LSB, 6},
  {CV_VERSION_ID, DCC_VERSION_ID},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_29_CONFIG, 0},
  {CV_S0LIMIT0, 10},
  {CV_S0LIMIT1,  120},
  {CV_S0TRAVEL, 10},
  {CV_S0FLAGS, SERVO_INITMID},
  {CV_S0FUNC, 0},
  {CV_S0BOUNCE, 8},
  {CV_S1LIMIT0, 10},
  {CV_S1LIMIT1, 90},
  {CV_S1TRAVEL, 10},
  {CV_S1FLAGS, SERVO_INITMID},
  {CV_S1FUNC, 1},
  {CV_S1BOUNCE, 8},
  {CV_S2LIMIT0, 0},
  {CV_S2LIMIT1, 180},
  {CV_S2TRAVEL, 10},
  {CV_S2FLAGS, SERVO_INITMID},
  {CV_S2FUNC, 2},
  {CV_S2BOUNCE, 8},
  {CV_STEPS, 8},
  {CV_RATIO, 64},
  {CV_MAXRPM, 60},
  {CV_STEPFUNC, 3},
  {CV_STEPMODE, 0},
  {CV_MAXLSB, 232},
  {CV_MAXMSB, 3},
  {CV_STEPDELAY, 10},
  {CV_MAXPWM, 255},
  {CV_PWMFUNC, 4},
  {CV_PWRDELAY, 50},
  {CV_L0EFFECT, LIGHT_EFFECT_DIMABLE},
  {CV_L0PERIOD, 40},
  {CV_L0FUNC, 5},
  {CV_L1EFFECT, LIGHT_EFFECT_CONSTANT},
  {CV_L1PERIOD, 40},
  {CV_L1FUNC, 6},
  {CV_L2EFFECT, LIGHT_EFFECT_FLASH_50},
  {CV_L2PERIOD, 40},
  {CV_L2FUNC, 7},
  {CV_L3EFFECT, LIGHT_EFFECT_FLASH_25},
  {CV_L3PERIOD, 40},
  {CV_L3FUNC, 8},
  {CV_29_CONFIG, 32},
  {CV_SP1STEPS, 4},
  {CV_SP1FN, 9},
  {CV_SP1S1F, 8},
  {CV_SP1S1T, 50},
  {CV_SP1S1DH, 0},
  {CV_SP1S1DL, 20},
  {CV_SP1S2F, 41},
  {CV_SP1S2T, 50},
  {CV_SP1S2DH, 0},
  {CV_SP1S2DL, 5},
  {CV_SP1S3F, 40},
  {CV_SP1S3T, 178},
  {CV_SP1S3DH, 0},
  {CV_SP1S3DL, 5},
  {CV_SP1S3F, 1},
  {CV_SP1S3T, 178},
  {CV_SP1S3DH, 0},
  {CV_SP1S3DL, 5}
};

#define FACTORY_RESET_AT_STARTUP  0

#if FACTORY_RESET_AT_STARTUP
static uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
#else
static uint8_t FactoryDefaultCVIndex = 0;
#endif

#if INVERTED_ACK
#define ACK_OFF   HIGH
#define ACK_ON    LOW
#else
#define ACK_OFF   LOW
#define ACK_ON    HIGH
#endif

#if INVERTED_CTL
#define PWR_OFF   HIGH
#define PWR_ON    LOW
#else
#define PWR_OFF   LOW
#define PWR_ON    HIGH
#endif
/*
   The function map.
   This is an array of bitmasks used to determine the function
   numbers that activate each device. The array is indexed by
   the function number and the value is a bitmask of the items
   under the control of that function
*/
unsigned int functions[28];
#define FNSERVO0      0x0001
#define FNSERVO1      0x0002
#define FNSERVO2      0x0004
#define FNSTEPPER     0x0008
#define FNPWM         0x0010
#define FNLIGHT0      0x0020
#define FNLIGHT1      0x0040
#define FNLIGHT2      0x0080
#define FNLIGHT3      0x0100
#define FNSP1         0x0200

/*
 * Set piece control
 */
int setPieceActive = 0;                // Number of set piece in action or 0 if none
int setPieceStep = 0;                  // Step number of set piece in operation
unsigned long setPieceNextEvent = 0UL; // Millis() at which point we move to the next set piece step
int setPieceBlock = 0;                 // Used to block repeated initiation of set pieces

void servoAck()
{
  int pos = servo1->getAngle();
  servo1->setPosition(pos+10);
  servo1->setActive(true);
  unsigned long stopAt = millis() + AckLength + 1;
  while (millis() < stopAt)
  {
    servo1->loop();
  }
  
  servo1->setAngle(pos);
  Serial.println("ServoAck");
}

int ackpin = 0;
void stepperAck()
{
  digitalWrite(stepperPins[ackpin], HIGH);
  digitalWrite(stepperPins[ackpin+1], HIGH);
  delay(AckLength);
  delayMicroseconds(AckAdditionaluS);
  digitalWrite(stepperPins[ackpin], LOW);
  digitalWrite(stepperPins[ackpin+1], LOW);
  ackpin++;
  if (ackpin > 2)
  ackpin = 0;
}
// This function is called by the NmraDcc library
// when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain
// on the power supply for 6ms to ACK a CV Read
void notifyCVAck(void)
{
#if SERVO_ACK
  servoAck();
  #elif STEPPER_ACK
  stepperAck();
#else
  digitalWrite(DccAckPin, ACK_ON);
  delay(AckLength);
  delayMicroseconds(AckAdditionaluS);
  digitalWrite(DccAckPin, ACK_OFF);
#endif
#if DEBUGACK
  Serial.println("CVAck");
#endif
}

/*
 * Called to notify a CV value has been changed
 */
void notifyCVChange( uint16_t CV, uint8_t value)
{
  int i;

#if DEBUGCV
  Serial.print("CV Write: ");
  Serial.print(CV);
  Serial.print(" = ");
  Serial.println(value);
#endif

  dcc_watchdog = true;
  switch (CV)
  {
    case CV_S0LIMIT0:
      if (servo1)
        servo1->setStart(value);
      break;
    case CV_S0LIMIT1:
      if (servo1)
        servo1->setEnd(value);
      break;
    case CV_S0TRAVEL:
      if (servo1)
        servo1->setTravelTime(value);
      break;
    case CV_S0FLAGS:
      servo1->setFlags(value);
      break;
    case CV_S0FUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNSERVO0);
      functions[value % 13] |= FNSERVO0;
      break;
    case CV_S0BOUNCE:
      if (servo1)
        servo1->setBounceAngle(value);
      break;
    case CV_S1LIMIT0:
      if (servo2)
        servo2->setStart(value);
      break;
    case CV_S1LIMIT1:
      if (servo2)
        servo2->setEnd(value);
      break;
    case CV_S1TRAVEL:
      if (servo2)
        servo2->setTravelTime(value);
      break;
    case CV_S1FLAGS:
      servo2->setFlags(value);
      break;
    case CV_S1FUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNSERVO1);
      functions[value % 13] |= FNSERVO1;
      break;
    case CV_S1BOUNCE:
      if (servo2)
        servo2->setBounceAngle(value);
      break;
    case CV_S2LIMIT0:
      servo3->setStart(value);
      break;
    case CV_S2LIMIT1:
      if (servo3)
        servo3->setEnd(value);
      break;
    case CV_S2TRAVEL:
      if (servo3)
        servo3->setTravelTime(value);
      break;
    case CV_S2FLAGS:
      servo3->setFlags(value);
      break;
    case CV_S2FUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNSERVO2);
      functions[value % 13] |= FNSERVO2;
      break;
    case CV_S2BOUNCE:
      if (servo3)
        servo3->setBounceAngle(value);
      break;
    case CV_STEPS:
      {
        unsigned int maxSteps = ((unsigned int)Dcc.getCV(CV_MAXMSB) * 256) + Dcc.getCV(CV_MAXLSB);
        DCCStepper *newStepper = new DCCStepper(Dcc.getCV(CV_STEPMODE), maxSteps,
                                                Dcc.getCV(CV_RATIO) * Dcc.getCV(CV_STEPS),
                                                Dcc.getCV(CV_MAXRPM), stepperPins[0], stepperPins[1],
                                                stepperPins[2], stepperPins[3]);
        unsigned int currentStep = ((unsigned int)Dcc.getCV(CV_CURMSB) * 256) + Dcc.getCV(CV_CURLSB);
        newStepper->setCurrentPosition(currentStep);
        delete stepper;
        stepper = newStepper;
      }
      break;
    case CV_RATIO:
      {
        unsigned int maxSteps = ((unsigned int)Dcc.getCV(CV_MAXMSB) * 256) + Dcc.getCV(CV_MAXLSB);
        DCCStepper *newStepper = new DCCStepper(Dcc.getCV(CV_STEPMODE), maxSteps,
                                                Dcc.getCV(CV_RATIO) * Dcc.getCV(CV_STEPS),
                                                Dcc.getCV(CV_MAXRPM), stepperPins[0], stepperPins[1],
                                                stepperPins[2], stepperPins[3]);
        unsigned int currentStep = ((unsigned int)Dcc.getCV(CV_CURMSB) * 256) + Dcc.getCV(CV_CURLSB);
        newStepper->setCurrentPosition(currentStep);
        delete stepper;
        stepper = newStepper;
      }
      break;
    case CV_STEPFUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNSTEPPER);
      functions[value] |= FNSTEPPER;
      break;
    case CV_STEPMODE:
      stepper->setMode(value);
      break;
    case CV_MAXLSB:
      stepper->setMaxStepsLSB(value);
      break;
    case CV_MAXMSB:
      stepper->setMaxStepsMSB(value);
      break;
    case CV_STEPDELAY:
      stepper->setReverseDelay(value);
      break;
    case CV_MAXRPM:
      if (stepper)
        stepper->setRPM(value);
      break;
    case CV_L0EFFECT:
      delete light0;
      light0 = new DCCLight(lightPins[0], value);
      break;
    case CV_L0PERIOD:
      light0->setPeriod(value);
      break;
    case CV_L0FUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNLIGHT0);
      functions[value] |= FNLIGHT0;
      break;
    case CV_L1EFFECT:
      delete light1;
      light1 = new DCCLight(lightPins[1], value);
      break;
    case CV_L1PERIOD:
      light1->setPeriod(value);
      break;
    case CV_L1FUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNLIGHT1);
      functions[value] |= FNLIGHT1;
      break;
    case CV_L2EFFECT:
      delete light2;
      light2 = new DCCLight(lightPins[2], value);
      break;
    case CV_L2PERIOD:
      light2->setPeriod(value);
      break;
    case CV_L2FUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNLIGHT2);
      functions[value] |= FNLIGHT2;
      break;
    case CV_L3EFFECT:
      delete light3;
      light3 = new DCCLight(lightPins[3], value);
      break;
    case CV_L3PERIOD:
      light3->setPeriod(value);
      break;
    case CV_L3FUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNLIGHT3);
      functions[value] |= FNLIGHT3;
      break;
    case CV_MULTIFUNCTION_PRIMARY_ADDRESS:
      MyAddress = value;
      break;
    case CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB:
    case CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB:
    case CV_29_CONFIG:
      break;
    case CV_SP1FN:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNSP1);
      functions[value] |= FNSP1;
      break;
  }
}

/*
 * A fectory reset has been requested
 */
void notifyCVResetFactoryDefault()
{
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
}

/*
 *  Periodically called with the speed, direction and number of speed steps
 *
 *  Work out the current speed percentage and direction and update each of the
 *  servos with this data
 */
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION ForwardDir, DCC_SPEED_STEPS SpeedSteps )
{
#if DEBUGNOTIFY
  Serial.print("Notify DCC Speed: ");
  Serial.print(Speed);
  Serial.print(" Direction: ");
  Serial.print(ForwardDir);
  Serial.print(" Steps ");
  Serial.println(SpeedSteps);
#endif
  int percentage = ((Speed - 1) * 100) / SpeedSteps;
  dcc_watchdog = true;
  if (setPieceActive > 0)
  {
    return;
  }
  setSpeed(percentage, ForwardDir != 0);
}

/*
 * Set the current speed as a percentage
 */
void setSpeed(int percentage, boolean forward)
{
#if DEBUG
  Serial.print("Speed: ");
  Serial.print(percentage);
  Serial.println("%");
#endif
  if (servo1 && servo1->isAbsolute())
    servo1->setPosition(percentage);
  else if (servo1)
    servo1->setSpeed(percentage, forward);
  if (servo2 && servo2->isAbsolute())
    servo2->setPosition(percentage);
  else if (servo2)
    servo2->setSpeed(percentage, forward);
  if (servo3 && servo3->isAbsolute())
    servo3->setPosition(percentage);
  else if (servo3)
    servo3->setSpeed(percentage, forward);
  if (stepper)
    stepper->setSpeed(percentage, forward);
  if (light0)
  {
    light0->setBrightness(percentage);
    light0->setDirection(forward);
  }
  if (light1)
  {
    light1->setBrightness(percentage);
    light1->setDirection(forward);
  }
  if (light2)
  {
    light2->setBrightness(percentage);
    light2->setDirection(forward);
  }
  if (light3)
  {
    light3->setBrightness(percentage);
    light3->setDirection(forward);
  }
}

/*
 * Called for the state of each of the functions, use the
 * the current function mapping to set of the state for each
 * of the devices we control.
 */
static void SetFuncState(int function, boolean state, boolean sp)
{
  if (setPieceActive == false || sp)
  {
    if ((functions[function] & FNSERVO0) && servo1)
    {
#if DEBUG
      Serial.println((state ? "Servo1 active" : "Servo1 inactive"));
#endif
#if STAGGER_SERVOS
      if (millis() < startms + (Dcc.getCV(CV_PWRDELAY) * 200))
      {
        state = false;
      }
#endif
      servo1->setActive(state);
    }
    if ((functions[function] & FNSERVO1) && servo2)
    {
#if DEBUG
      Serial.println((state ? "Servo2 active" : "Servo2 inactive"));
#endif
#if STAGGER_SERVOS
      if (millis() < startms + (Dcc.getCV(CV_PWRDELAY) * 300))
      {
        state = false;
      }
#endif
      servo2->setActive(state);
    }
    if ((functions[function] & FNSERVO2) && servo3)
    {
#if DEBUG
      Serial.println((state ? "Servo3 active" : "Servo3 inactive"));
#endif
#if STAGGER_SERVOS
      if (millis() < startms + (Dcc.getCV(CV_PWRDELAY) * 400))
      {
        state = false;
      }
#endif
      servo3->setActive(state);
    }
    if ((functions[function] & FNSTEPPER) && stepper)
    {
#if DEBUG
      Serial.println((state ? "Stepper active" : "Stepper inactive"));
#endif
      stepper->setActive(state);
    }
    if ((functions[function] & FNLIGHT0) && light0)
    {
#if DEBUG
      Serial.println((state ? "Light0 active" : "Light0 inactive"));
#endif
      light0->setActive(state);
    }
    if ((functions[function] & FNLIGHT1) && light1)
    {
#if DEBUG
      Serial.println((state ? "Light1 active" : "Light1 inactive"));
#endif
      light1->setActive(state);
    }
    if ((functions[function] & FNLIGHT2) && light2)
    {
#if DEBUG
      Serial.println((state ? "Light2 active" : "Light2 inactive"));
#endif
      light2->setActive(state);
    }
    if ((functions[function] & FNLIGHT3) && light3)
    {
#if DEBUG
      Serial.println((state ? "Light3 active" : "Light3 inactive"));
#endif
      light3->setActive(state);
    }
  }
  if ((functions[function] & FNSP1) && state && (setPieceBlock & 1) == 0)
  {
#if DEBUG || DEBUGSETP
    Serial.println("Start set piece");
#endif
    setPieceActive = 1;
    setPieceAdvance();
    setPieceBlock |= 1;
  }
  if ((functions[function] & FNSP1) && state == false && (setPieceBlock & 1) == 1)
  {
    setPieceBlock &= ~1; // Allow the sequence to be started again
    if (setPieceActive == 1)
    { // If the set piece is running terminate it
      setPieceActive = 0;
      setPieceNextEvent = 0UL;
      setPieceStep = 0;
#if DEBUGSETP
      Serial.println("Terminate set piece");
#endif
    }
  }
}

/*
 *  Called regularly to report the state of the function keys
 */
void notifyDccFunc( uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
#if DEBUGNOTIFY
  Serial.print("Notify DCC Func: Grp: ");
  Serial.print(FuncGrp);
  Serial.print(" State: ");
  Serial.println(FuncState, HEX);
#endif
  dcc_watchdog = true;
  if (FuncGrp == FN_0_4)
  {
    /* Function group 1 */
    if (FuncState & 0x10)
      SetFuncState(0, true, false);
    else
      SetFuncState(0, false, false);
    if (FuncState & 0x01)
      SetFuncState(1, true, false);
    else
      SetFuncState(1, false, false);
    if (FuncState & 0x02)
      SetFuncState(2, true, false);
    else
      SetFuncState(2, false, false);
    if (FuncState & 0x04)
      SetFuncState(3, true, false);
    else
      SetFuncState(3, false, false);
    if (FuncState & 0x08)
      SetFuncState(4, true, false);
    else
      SetFuncState(4, false, false);
  }
  else if (FuncGrp == FN_5_8)
  {
    /* Function group 2 */
    if (FuncState & 0x01)
      SetFuncState(5, true, false);
    else
      SetFuncState(5, false, false);
    if (FuncState & 0x02)
      SetFuncState(6, true, false);
    else
      SetFuncState(6, false, false);
    if (FuncState & 0x04)
      SetFuncState(7, true, false);
    else
      SetFuncState(7, false, false);
    if (FuncState & 0x08)
      SetFuncState(8, true, false);
    else
      SetFuncState(8, false, false);

  }
  else if (FuncGrp == FN_9_12)
  {
    /* Function group 3 */
    if (FuncState & 0x01)
      SetFuncState(9, true, false);
    else
      SetFuncState(9, false, false);
    if (FuncState & 0x02)
      SetFuncState(10, true, false);
    else
      SetFuncState(10, false, false);
    if (FuncState & 0x04)
      SetFuncState(11, true, false);
    else
      SetFuncState(11, false, false);
    if (FuncState & 0x08)
      SetFuncState(12, true, false);
    else
      SetFuncState(12, false, false);
  }
  else if (FuncGrp == FN_13_20)
  {
    /* Function group 3 */
    if (FuncState & 0x01)
      SetFuncState(13, true, false);
    else
      SetFuncState(13, false, false);
    if (FuncState & 0x02)
      SetFuncState(14, true, false);
    else
      SetFuncState(14, false, false);
    if (FuncState & 0x04)
      SetFuncState(15, true, false);
    else
      SetFuncState(15, false, false);
    if (FuncState & 0x08)
      SetFuncState(16, true, false);
    else
      SetFuncState(16, false, false);
    if (FuncState & 0x10)
      SetFuncState(17, true, false);
    else
      SetFuncState(17, false, false);
    if (FuncState & 0x20)
      SetFuncState(18, true, false);
    else
      SetFuncState(18, false, false);
    if (FuncState & 0x40)
      SetFuncState(19, true, false);
    else
      SetFuncState(19, false, false);
    if (FuncState & 0x80)
      SetFuncState(20, true, false);
    else
      SetFuncState(20, false, false);
  }
  else if (FuncGrp == FN_21_28)
  {
    /* Function group 4 */
    if (FuncState & 0x01)
      SetFuncState(21, true, false);
    else
      SetFuncState(21, false, false);
    if (FuncState & 0x02)
      SetFuncState(22, true, false);
    else
      SetFuncState(22, false, false);
    if (FuncState & 0x04)
      SetFuncState(23, true, false);
    else
      SetFuncState(23, false, false);
    if (FuncState & 0x08)
      SetFuncState(24, true, false);
    else
      SetFuncState(24, false, false);
    if (FuncState & 0x10)
      SetFuncState(25, true, false);
    else
      SetFuncState(25, false, false);
    if (FuncState & 0x20)
      SetFuncState(26, true, false);
    else
      SetFuncState(26, false, false);
    if (FuncState & 0x40)
      SetFuncState(27, true, false);
    else
      SetFuncState(27, false, false);
    if (FuncState & 0x80)
      SetFuncState(28, true, false);
    else
      SetFuncState(28, false, false);
  }
}

/*
 * Callback from the stepper library used to record the stepper position
 */
void notifyStepperPosition(DCCStepper *motor, unsigned int position)
{
  if (motor == stepper)
  {
    Dcc.setCV(CV_CURLSB, position & 0xff);
    Dcc.setCV(CV_CURMSB, (position >> 8) & 0xff);
  }
}

/*
 * Callback from the servo library used to record the servo position
 */
void notifyServoPosition(DCCServo *servo, int position)
{
  if (servo == servo1)
  {
    Dcc.setCV(CV_S0POSITION, position);
  }
  else if (servo == servo2)
  {
    Dcc.setCV(CV_S1POSITION, position);
  }
  else if (servo == servo1)
  {
    Dcc.setCV(CV_S2POSITION, position);
  }
}

/*
 *  Advance to the next step in the set piece
 */
void setPieceAdvance()
{
  int offset = 0;

  setPieceStep++;

  switch (setPieceActive) {
    case 1:
      if (Dcc.getCV(CV_SP1STEPS) < setPieceStep)
      {
#if DEBUG
        Serial.println("Reached end of set piece");
#endif
        setPieceStep = 0;
        setPieceActive = 0;
        setPieceNextEvent = 0UL;
        break;
      }
      offset = CV_SP1S1F + ((setPieceStep - 1) * 4);
      break;
  }

  if (setPieceActive == 0)
    return;

#if DEBUGSETP
  Serial.print("SetPiece Step now ");
  Serial.println(setPieceStep);
#endif
  int funcs = Dcc.getCV(offset);
  int throttle = Dcc.getCV(offset + 1);
  unsigned int duration = (Dcc.getCV(offset + 2) << 8) + Dcc.getCV(offset + 3);
  setPieceNextEvent = millis() + (duration * 1000);
#if DEBUGSETP
  Serial.print("SetPiece Step: FN ");
  Serial.print(funcs, HEX);
  Serial.print(" Speed: ");
  Serial.print(throttle);
  Serial.print(" Duration: ");
  Serial.println(duration);
#endif
  for (int i = 0; i < 8; i++)
  {
    if (funcs & ( 1 << i))
    {
#if DEBUGSETP
      Serial.print("F");
      Serial.print(i);
      Serial.println(" On");
#endif
      SetFuncState(i, true, true);
    }
    else
    {
#if DEBUGSETP
      Serial.print("F");
      Serial.print(i);
      Serial.println(" Off");
#endif
      SetFuncState(i, false, true);
    }
  }

  setSpeed(throttle & 0x7f, (throttle & 0x80) == 0);
}

/*
 * When in service mode we should turn off high current loads
 */
void notifyServiceMode(boolean serviceMode)
{
  if (serviceMode && poweron)
  {
    digitalWrite(ControlPin, PWR_OFF);
    powerWasOn = poweron;
    poweron = false;
    inServiceMode = true;
  }
  else if (powerWasOn && serviceMode == false)
  {
    digitalWrite(ControlPin, PWR_ON);
    poweron = true;
    inServiceMode = false;


  }
  inServiceMode = serviceMode;
#if DEBUGACK
  Serial.print("Service Mode ");
  Serial.println(serviceMode ? "On" : "Off");

#endif
}

/*
The setup function is called once afer a reset of powerup event
*/
void setup()
{
  int i;

#if DEBUG || DEBUGCV || DEBUGNOTIFY || DEBUGSETP || DEBUGACK
  Serial.begin(BAUDRATE);
#endif

  // Turn off the power to the servos by taking the control line high
  pinMode(ControlPin, OUTPUT);
  digitalWrite(ControlPin, PWR_OFF);

  // Configure the DCC CV Programing ACK pin for an output
  pinMode(DccAckPin, OUTPUT);
  digitalWrite(DccAckPin, ACK_OFF);

  pinMode(LedPin, OUTPUT);

  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
  Dcc.pin(DccIntNum, DccInPin, 1);

  // If the saved CV value for the decoder does not match this version of
  // the code force a factry reset of the CV values
  if (Dcc.getCV(CV_VERSION_ID) != DCC_VERSION_ID)
  {
    FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
#if DEBUGCV
    Serial.print("Decoder version in CV's is ");
    Serial.println(Dcc.getCV(CV_VERSION_ID));
    Serial.print("Set factory default CV's ");
    Serial.println(FactoryDefaultCVIndex);
#endif
  }

#if DEBUGCV
  for (i = 0; i < sizeof(FactoryDefaultCVs) / sizeof(CVPair); i++)
  {
    Serial.print("CV ");
    Serial.print(FactoryDefaultCVs[i].CV);
    Serial.print(" = ");
    Serial.println(Dcc.getCV(FactoryDefaultCVs[i].CV));
  }
#endif

  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, DCC_VERSION_ID, FLAGS_MY_ADDRESS_ONLY | FLAGS_OUTPUT_ADDRESS_MODE, 0);

  // Create the instances of the servos, intiialise the limits and travel
  // times from the CV values
  servo1 = new DCCServo(servoPins[0], Dcc.getCV(CV_S0LIMIT0),
                        Dcc.getCV(CV_S0LIMIT1), Dcc.getCV(CV_S0TRAVEL),
                        Dcc.getCV(CV_S0FLAGS));
  servo1->setBounceAngle(Dcc.getCV(CV_S0BOUNCE));
  servo2 = new DCCServo(servoPins[1], Dcc.getCV(CV_S1LIMIT0),
                        Dcc.getCV(CV_S1LIMIT1), Dcc.getCV(CV_S1TRAVEL),
                        Dcc.getCV(CV_S1FLAGS));
  servo2->setBounceAngle(Dcc.getCV(CV_S1BOUNCE));
  servo3 = new DCCServo(servoPins[2], Dcc.getCV(CV_S2LIMIT0),
                        Dcc.getCV(CV_S2LIMIT1), Dcc.getCV(CV_S2TRAVEL),
                        Dcc.getCV(CV_S2FLAGS));
  servo3->setBounceAngle(Dcc.getCV(CV_S2BOUNCE));

  unsigned int maxSteps = ((unsigned int)Dcc.getCV(CV_MAXMSB) * 256) + Dcc.getCV(CV_MAXLSB);
  stepper = new DCCStepper(Dcc.getCV(CV_STEPMODE), maxSteps,
                           Dcc.getCV(CV_RATIO) * Dcc.getCV(CV_STEPS),
                           Dcc.getCV(CV_MAXRPM), stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);
  unsigned int currentStep = ((unsigned int)Dcc.getCV(CV_CURMSB) * 256) + Dcc.getCV(CV_CURLSB);
  stepper->setCurrentPosition(currentStep);
  stepper->setReverseDelay(Dcc.getCV(CV_STEPDELAY));

  light0 = new DCCLight(lightPins[0], Dcc.getCV(CV_L0EFFECT));
  light1 = new DCCLight(lightPins[1], Dcc.getCV(CV_L1EFFECT));
  light2 = new DCCLight(lightPins[2], Dcc.getCV(CV_L2EFFECT));
  light3 = new DCCLight(lightPins[3], Dcc.getCV(CV_L3EFFECT));

  for (i = 0; i < 28; i++)
    functions[i] = 0;
  functions[Dcc.getCV(CV_S0FUNC)] |= FNSERVO0;
  functions[Dcc.getCV(CV_S1FUNC)] |= FNSERVO1;
  functions[Dcc.getCV(CV_S2FUNC)] |= FNSERVO2;
  functions[Dcc.getCV(CV_STEPFUNC)] |= FNSTEPPER;
  functions[Dcc.getCV(CV_PWMFUNC)] |= FNPWM;
  functions[Dcc.getCV(CV_L0FUNC)] |= FNLIGHT0;
  functions[Dcc.getCV(CV_L1FUNC)] |= FNLIGHT1;
  functions[Dcc.getCV(CV_L2FUNC)] |= FNLIGHT2;
  functions[Dcc.getCV(CV_L3FUNC)] |= FNLIGHT3;
  functions[Dcc.getCV(CV_SP1FN)] |= FNSP1;

  if (servo1 && (Dcc.getCV(CV_S0FLAGS) & SERVO_INITMID) == 0)
  {
    servo1->setAngle(Dcc.getCV(CV_S0POSITION));
  }
  if (servo2 && (Dcc.getCV(CV_S1FLAGS) & SERVO_INITMID) == 0)
  {
    servo2->setAngle(Dcc.getCV(CV_S1POSITION));
  }
  if (servo3 && (Dcc.getCV(CV_S2FLAGS) & SERVO_INITMID) == 0)
  {
    servo3->setAngle(Dcc.getCV(CV_S2POSITION));
  }

  servo1->setActive(false);
  servo2->setActive(false);
  servo3->setActive(false);

}

/*
The loop function is called repeatedly after the setup has been completed.
We must call the various routines in the DCC, DDCServo and DCCStepper libraries
so that they may perform whatever actions they require.
*/
void loop()
{

  if (startms == 0) {
    startms = millis();
  }
  // After WAIT_MS turn the power on
  if (poweron == false && millis() > startms + (Dcc.getCV(CV_PWRDELAY) * 100))
  {
#if DEBUG
    Serial.println("Turn Power on");
#endif
    if (inServiceMode == true)
    {
      powerWasOn = true;
    }
    else
    {
      digitalWrite(ControlPin, PWR_ON);
      poweron = true;
      powerNotify = 8;
    }
  }
  // Execute the DCC process frequently in order to ensure
  // the DCC signal processing occurs
  Dcc.process();

  /* Check to see if the default CV values are required */
  if (FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV,
               FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }

  if (setPieceActive > 0 && millis() > setPieceNextEvent)
  {
#if DEBUGSETP
    Serial.println("Advance Set Piece");
#endif
    setPieceAdvance();
  }

  // Now call the loop method for every DCCServo instance
  if (servo1)
    servo1->loop();
  if (servo2)
    servo2->loop();
  if (servo3)
    servo3->loop();
  if (stepper)
    stepper->loop();
  if (light0)
    light0->loop();
  if (light1)
    light1->loop();
  if (light2)
    light2->loop();
  if (light3)
    light3->loop();

#if LEDFEEDBACK
  // Do not flash LED when in service mode
  if (inServiceMode == false && skip_millis != millis())
  {
    skip_millis = millis();
    if (powerNotify > 0 && (millis() & 0x7f) == 0)
    {
      if (ledState != LOW)
      {
        ledState = LOW;
      }
      else
      {
        ledState = HIGH;
      }
      powerNotify--;
      digitalWrite(LedPin, ledState);
    }
    else if ((millis() & 0x1ff) == 0)
    {
      if (ledState != LOW)
      {
        ledState = LOW;
      }
      else if (dcc_watchdog)
      {
        ledState = HIGH;
        dcc_watchdog = false;
      }
      digitalWrite(LedPin, ledState);
    }
  }
#endif
}
