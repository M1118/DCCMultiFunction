
#include <NmraDcc.h>
#include <Servo.h>
#include <DCCServo.h>
#include <DCCStepper.h>
#include <DCCLight.h>

/*
 * DCC Mixed Device Decoder
 *
 * A DCC decoder designed to drive a variety of devices, a mix of RC Servos, Stepper Motor and DC motor
 */
#define DCC_VERSION_ID  11

#define DEBUG           0    // Enable debug output on the serial interface
#define DEBUG_CV        0    // Enable output of CV relates debug
#define BAUDRATE     9600
/*
 * Arduino pin assignments
 */
const int DccAckPin = 3;
const int DccInPin = 2;
const int servoPins[] = { 4, 5, 6 };
const int stepperPins[] = { A0, A1, A2, A3 };
const int pwmPins[] = { 11, 12 };
const int lightPins[] = { 7, 8, 9, 10 };

/*
 * Power up control
 */
#define CONTROL  13      // Pin to take low after power-up delay

unsigned long startms = 0;
boolean poweron = false;

NmraDcc     Dcc;
DCC_MSG     Packet;

DCCServo    *servo1 = NULL, *servo2 = NULL, *servo3 = NULL;
DCCStepper  *stepper = NULL;
DCCLight    *light0 = NULL, *light1 = NULL, *light2 = NULL, *light3 = NULL;
boolean      motorAttached = false;

/*
 * Items related to the CV's that control the behavior of the
 * decoder
 */
// CV Storage structure
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};
 
int MyAddress;

/*
 * The CV's that are used for the decoder
 */
#define CV_S0LIMIT0    30
#define CV_S0LIMIT1    31
#define CV_S0TRAVEL    32
#define CV_S0FLAGS     33
#define CV_S0FUNC      34
#define CV_S1LIMIT0    35
#define CV_S1LIMIT1    36
#define CV_S1TRAVEL    37
#define CV_S1FLAGS     38
#define CV_S1FUNC      39
#define CV_S2LIMIT0    40
#define CV_S2LIMIT1    41
#define CV_S2TRAVEL    42
#define CV_S2FLAGS     43
#define CV_S2FUNC      44
#define CV_STEPS       50
#define CV_RATIO       51
#define CV_MAXRPM      52
#define CV_STEPFUNC    53
#define CV_STEPMODE    54
#define CV_MAXLSB      55
#define CV_MAXMSB      56
#define CV_CURLSB      57
#define CV_CURMSB      58
#define CV_MAXPWM      60
#define CV_PWMFUNC     61
#define CV_PWRDELAY    62
#define CV_L0EFFECT    65
#define CV_L0PERIOD    66
#define CV_L0FUNC      67
#define CV_L1EFFECT    70
#define CV_L1PERIOD    71
#define CV_L1FUNC      72
#define CV_L2EFFECT    75
#define CV_L2PERIOD    76
#define CV_L2FUNC      77
#define CV_L3EFFECT    80
#define CV_L3PERIOD    81
#define CV_L3FUNC      82

/*
 * The factory default CV values
 */
CVPair FactoryDefaultCVs [] =
{
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, 3},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, 3},
  {CV_VERSION_ID, DCC_VERSION_ID},
  {CV_MANUFACTURER_ID, MAN_ID_DIY},
  {CV_S0LIMIT0, 10},
  {CV_S0LIMIT1,  90},
  {CV_S0TRAVEL, 10},
  {CV_S0FLAGS, SERVO_INITMID},
  {CV_S0FUNC, 0},
  {CV_S1LIMIT0, 10},
  {CV_S1LIMIT1, 90},
  {CV_S1TRAVEL, 10},
  {CV_S1FLAGS, SERVO_INITMID},
  {CV_S1FUNC, 1},
  {CV_S2LIMIT0, 0},
  {CV_S2LIMIT1, 180},
  {CV_S2TRAVEL, 10},
  {CV_S2FLAGS, SERVO_INITMID},
  {CV_S2FUNC, 2},
  {CV_STEPS, 8},
  {CV_RATIO, 64},
  {CV_MAXRPM, 60},
  {CV_STEPFUNC, 3},
  {CV_STEPMODE, 0},
  {CV_MAXLSB, 232},
  {CV_MAXMSB, 3},
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
};

#define FACTORY_RESET_AT_STARTUP  0

#if FACTORY_RESET_AT_STARTUP
static uint8_t FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
#else
static uint8_t FactoryDefaultCVIndex = 0;
#endif

/*
 * The function map.
 * This is an array of bitmasks used to determine the function
 * numbers that activate each device. The array is indexed by
 * the function number and the value is a bitmask of the items
 * under the control of that function
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

// This function is called by the NmraDcc library 
// when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain
// on the power supply for 6ms to ACK a CV Read 
void notifyCVAck(void)
{
  digitalWrite( DccAckPin, HIGH );
  delay( 6 );  
  digitalWrite( DccAckPin, LOW );
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
    case CV_STEPS:
      {
      DCCStepper *newStepper = new DCCStepper(Dcc.getCV(CV_RATIO) * value,
                Dcc.getCV(CV_MAXRPM), stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);
      stepper = newStepper;
      }
      break;
    case CV_RATIO:
      {
      DCCStepper *newStepper = new DCCStepper(value * Dcc.getCV(CV_STEPS),
                Dcc.getCV(CV_MAXRPM), stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);
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
    case CV_MAXRPM:
      if (stepper)
        stepper->setRPM(value);
      break;
    case CV_PWMFUNC:
      for (i = 0; i < 28; i++)
        functions[i] &= (~FNPWM);
      functions[value] |= FNPWM;
      break;
    case CV_L0EFFECT:
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
  }
}

void notifyCVResetFactoryDefault()
{
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
}

/*
 * Periodically called with the speed, direction and number of speed steps
 *
 * Work out the current speed percentage and direction and update each of the
 * servos with this data
 */
void notifyDccSpeed( uint16_t Addr, uint8_t Speed, uint8_t ForwardDir, uint8_t SpeedSteps )
{
  int percentage = ((Speed - 1) * 100) / SpeedSteps;
  
#if DEBUG
  Serial.print(percentage);
  Serial.println("%");
#endif
  if (servo1 && servo1->isAbsolute())
    servo1->setPosition(percentage);
  else if (servo1)
    servo1->setSpeed(percentage, ForwardDir != 0);
  if (servo2 && servo2->isAbsolute())
    servo2->setPosition(percentage);
  else if (servo2)
    servo2->setSpeed(percentage, ForwardDir != 0);
  if (servo3 && servo3->isAbsolute())
    servo3->setPosition(percentage);
  else if (servo3)
    servo3->setSpeed(percentage, ForwardDir != 0);
  if (stepper)
    stepper->setSpeed(percentage, ForwardDir != 0);
  if (motorAttached)
  {
     if (ForwardDir)
     {
       digitalWrite(pwmPins[0], LOW);
       analogWrite(pwmPins[1], percentage);
     }
     else
     {
       digitalWrite(pwmPins[1], LOW);
       analogWrite(pwmPins[0], percentage);
     }
  }
  if (light0)
  {
    light0->setBrightness(percentage);
    light0->setDirection(ForwardDir != 0);
  }
  if (light1)
  {
    light1->setBrightness(percentage);
    light1->setDirection(ForwardDir != 0);
  }
  if (light2)
  {
    light2->setBrightness(percentage);
    light2->setDirection(ForwardDir != 0);
  }
  if (light3)
  {
    light3->setBrightness(percentage);
    light3->setDirection(ForwardDir != 0);
  }
}

/*
 * Called for the state of each of the functions, use the
 * the current function mapping to set of the stte for each
 * of the devices we control.
 */
static void SetFuncState(int function, boolean state)
{
  if ((functions[function] & FNSERVO0) && servo1)
  {
#if DEBUG
    Serial.println((state ? "Servo1 active" : "Servo1 inactive"));
#endif
    servo1->setActive(state);
  }
  if ((functions[function] & FNSERVO1) && servo2)
  {
#if DEBUG
    Serial.println((state ? "Servo2 active" : "Servo2 inactive"));
#endif
    servo2->setActive(state);
  }
  if ((functions[function] & FNSERVO2) && servo3)
  {
#if DEBUG
    Serial.println((state ? "Servo3 active" : "Servo3 inactive"));
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
  if (functions[function] & FNPWM)
  {
#if DEBUG
    Serial.println((state ? "PWM active" : "PWM inactive"));
#endif
    motorAttached = state;
    if (state == false)
    {
      digitalWrite(pwmPins[0], LOW);
      digitalWrite(pwmPins[1], LOW);
    }
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

/*
 * Called regularly to report the state of the function keys
 */
void notifyDccFunc( uint16_t Addr, uint8_t FuncNum, uint8_t FuncState)
{
  if (FuncNum == 1)
  {
    /* Function group 1 */
    if (FuncState & 0x10)
      SetFuncState(0, true);
    else
      SetFuncState(0, false);
    if (FuncState & 0x01)
      SetFuncState(1, true);
    else
      SetFuncState(1, false);
    if (FuncState & 0x02)
      SetFuncState(2, true);
    else
      SetFuncState(2, false);
    if (FuncState & 0x04)
      SetFuncState(3, true);
    else
      SetFuncState(3, false);
    if (FuncState & 0x08)
      SetFuncState(4, true);
    else
      SetFuncState(4, false);
  }
  else if (FuncNum == 2)
  {
    /* Function group 2 */
    if (FuncState & 0x01)
      SetFuncState(5, true);
    else
      SetFuncState(5, false);
    if (FuncState & 0x02)
      SetFuncState(6, true);
    else
      SetFuncState(6, false);
    if (FuncState & 0x04)
      SetFuncState(7, true);
    else
      SetFuncState(7, false);
    if (FuncState & 0x08)
      SetFuncState(8, true);
    else
      SetFuncState(8, false);
    if (FuncState & 0x10)
      SetFuncState(9, true);
    else
      SetFuncState(9, false);
    if (FuncState & 0x20)
      SetFuncState(10, true);
    else
      SetFuncState(10, false);
    if (FuncState & 0x40)
      SetFuncState(11, true);
    else
      SetFuncState(11, false);
    if (FuncState & 0x80)
      SetFuncState(12, true);
    else
      SetFuncState(12, false);
  }
  else if (FuncNum == 3)
  {
    /* Function group 3 */
    if (FuncState & 0x01)
      SetFuncState(13, true);
    else
      SetFuncState(13, false);
    if (FuncState & 0x02)
      SetFuncState(14, true);
    else
      SetFuncState(14, false);
    if (FuncState & 0x04)
      SetFuncState(15, true);
    else
      SetFuncState(15, false);
    if (FuncState & 0x08)
      SetFuncState(16, true);
    else
      SetFuncState(16, false);
    if (FuncState & 0x10)
      SetFuncState(17, true);
    else
      SetFuncState(17, false);
    if (FuncState & 0x20)
      SetFuncState(18, true);
    else
      SetFuncState(18, false);
    if (FuncState & 0x40)
      SetFuncState(19, true);
    else
      SetFuncState(19, false);
    if (FuncState & 0x80)
      SetFuncState(20, true);
    else
      SetFuncState(20, false);
  }
}

void notifyStepperPosition(DCCStepper *motor, unsigned int position)
{
  if (motor == stepper)
  {
    Dcc.setCV(CV_CURLSB, position & 0xff);
    Dcc.setCV(CV_CURMSB, (position >> 8) & 0xff);
  }
}

/*
 * The setup function is called once afer a reset of powerup event
 */
void setup()
{
  int i;
  
#if DEBUG || DEBUGCV
  Serial.begin(BAUDRATE);
#endif
  
  // Turn off the power to the servos by takign the control line high
  pinMode(CONTROL, OUTPUT);
  digitalWrite(CONTROL, HIGH);
  
  // Configure the DCC CV Programing ACK pin for an output
  pinMode(DccAckPin, OUTPUT);
  digitalWrite(DccAckPin, LOW);
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, DccInPin, 1);
  
  // If the saved CV value for the decoder does not match this version of
  // the code force a factry reset of the CV values
  if (Dcc.getCV(CV_VERSION_ID) != DCC_VERSION_ID)
  {
    FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
#if DEBUGCV
    Serial.print("Decoder version in CV's is ");
    Serial.println(Dcc.getCV(CV_VERSION_ID));
    Serial.print("Set factory default CV's ");
    Serial.println(FactoryDefaultCVIndex);
#endif
  }
 
#if DEBUGCV 
  for (i = 0; i < sizeof(FactoryDefaultCVs)/sizeof(CVPair); i++)
  {
    Serial.print("CV ");
    Serial.print(FactoryDefaultCVs[i].CV);
    Serial.print(" = ");
    Serial.println(Dcc.getCV(FactoryDefaultCVs[i].CV));
  }
#endif
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init(MAN_ID_DIY, DCC_VERSION_ID, FLAGS_MY_ADDRESS_ONLY|FLAGS_OUTPUT_ADDRESS_MODE, 0);
  
  // Create the instances of the servos, intiialise the limits and travel
  // times from the CV values
  servo1 = new DCCServo(servoPins[0], Dcc.getCV(CV_S0LIMIT0),
                      Dcc.getCV(CV_S0LIMIT1), Dcc.getCV(CV_S0TRAVEL),
                      Dcc.getCV(CV_S0FLAGS));
  servo2 = new DCCServo(servoPins[1], Dcc.getCV(CV_S1LIMIT0),
                      Dcc.getCV(CV_S1LIMIT1), Dcc.getCV(CV_S1TRAVEL),
                      Dcc.getCV(CV_S1FLAGS));
  servo3 = new DCCServo(servoPins[2], Dcc.getCV(CV_S2LIMIT0),
                      Dcc.getCV(CV_S2LIMIT1), Dcc.getCV(CV_S2TRAVEL),
                      Dcc.getCV(CV_S2FLAGS));
  unsigned int maxSteps = ((unsigned int)Dcc.getCV(CV_MAXMSB) * 256) + Dcc.getCV(CV_MAXLSB);
  stepper = new DCCStepper(Dcc.getCV(CV_STEPMODE), maxSteps,
                Dcc.getCV(CV_RATIO) * Dcc.getCV(CV_STEPS),
                Dcc.getCV(CV_MAXRPM), stepperPins[0], stepperPins[1], stepperPins[2], stepperPins[3]);
  unsigned int currentStep = ((unsigned int)Dcc.getCV(CV_CURMSB) * 256) + Dcc.getCV(CV_CURLSB);
  stepper->setCurrentPosition(currentStep);
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
  
  servo1->setActive(false);
  servo2->setActive(false);
  servo3->setActive(false);
}

/*
 * The loop function is called repeatedly after the setup has been completed.
 * We must call the various routines in the DCC, DDCServo and DCCStepper libraries
 * so that they may perform whatever actions they require.
 */
void loop()
{
  
  if (startms == 0)
    startms = millis();
  // After WAIT_MS turn the power on
  if (poweron == false && millis() > startms + (Dcc.getCV(CV_PWRDELAY) * 100))
  {
#if DEBUG
    Serial.println("Turn Power on");
#endif
    digitalWrite(CONTROL, LOW);
    poweron = true;
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
}
