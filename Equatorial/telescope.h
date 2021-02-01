#include <Arduino.h>



// VELOCIDAD DE COMUNICACION SERIAL
#define BAUDS 9600

// CONTEO DE TIEMPO PARA CALCULO DE LST
#define interval 1000


#define rad (M_PI / 180.0d)
#define DEG_45  162000L
#define DEG_90  324000L
#define DEG_180 648000L
#define DEG_270 972000L
#define DEG_360 1296000L



//------------------------------------------------------------------------------------------------------
// RTC - RELOJ
#define RST 5
#define DAT 6
#define CLK 7



//------------------------------------------------------------------------------------------------------
// KEYPAD
#define COLUMNS 4
#define ROWS 4
char keys[ROWS][COLUMNS] = {
    { '1', '2', '3', 'A' },
    { '4', '5', '6', 'B' },
    { '7', '9', '9', 'C' },
    { '*', '0', '#', 'D' }
};
byte PINSROWS[ROWS] = { 40, 42, 44, 46 };
byte PINSCOLUMNS[COLUMNS] = { 48, 50, 52, 54 };


//------------------------------------------------------------------------------------------------------
// JOYSTIC
#define X A1                            // Joystick X pin connected to A0 on the UNO
#define Y A2                            // Joystick Y pin connected to A1 on the UNO
#define INTERRUPT_PIN 2                             // Joystick switch connected to interrupt Pin 2 on UNO

long joystepX = 0;                                // Used to move steppers when using the Joystick
long joystepY = 0;                                // Used to move steppers when using the Joystick

// Variables used to store the IN and OUT points
volatile long XInPoint = 0;
volatile long ZInPoint = 0;
volatile long XOutPoint = 0;
volatile long ZOutPoint = 0;

volatile bool switchBtn = true;

long gotoposition[2];  // Used to move steppers using MultiStepper Control

const uint8_t DEBOUNCE_DELAY = 500;   // the debounce time; increase if the output flickers


//------------------------------------------------------------------------------------------------------
// MOTORES
#define DIR_AZ_MOTOR 9
#define STEP_AZ_MOTOR 8
#define DIR_ALT_MOTOR 11
#define STEP_ALT_MOTOR 10


#define DRIVEN_GEAR 98.0d
// #define DRIVEN_GEAR 63.0d
#define DRIVING_GEAR 21.0d
#define REDUCTION_RATIO (DRIVEN_GEAR / DRIVING_GEAR)                    // 1:3 gear reduction
#define MOTOR_STEPS 200.0d                                              // Stepper drive have 200 steps per revolution
#define MOTOR_MICROSTEPS_X_STEP 8.0d                                    // I'll use 1/16 microsteps mode to drive sidereal - also determines the LOWEST speed.
#define MOTOR_MICROSTEPS double(MOTOR_STEPS * MOTOR_MICROSTEPS_X_STEP)
#define TOTAL_MICROSTEPS_X_DEG_360 double(MOTOR_MICROSTEPS * REDUCTION_RATIO)
#define SIN_NOMBRE double(TOTAL_MICROSTEPS_X_DEG_360 / DEG_360)

long receivedSteps = 0L;                     // Number of steps
long receivedSpeed = 0L;                     // Steps / second
long receivedAcceleration = 0L;              // Steps / second^2
char receivedCommand;
long direction = 1L;                // = 1: positive direction, = -1: negative direction
bool newData, runallowed = false;           // booleans for new data from serial, and runallowed flag
// unsigned long distanceToGo = 0;








// VARIABLES PARA DEFINIR UN INVERVALO DE RECALCULO DE 1 SEGUNDO
unsigned int currentTime;
unsigned long previousMillis = 0;
unsigned long currentMillis;

unsigned long currentPosition;
unsigned long A_tel_from;