#include <Wire.h>
#include <LiquidCrystal_I2C.h>                                              // LCD
#include <stdlib.h>                                                         // 
#include <math.h>                                                           // MATEMATICA
#include <DS1302.h>                                                         // RTC
#include <AccelStepper.h>                                                   // STEPPER MOTOR
#include <MultiStepper.h>                                                   // MULTIPLE STEPPER MOTOR
#include <Keypad.h>                                                         // KEYPAD
#include "telescope.h"                                                      
#include <DueTimer.h>                                                       // INTERNAL TIME INTERRUPT PARA ARDUINO DUE



// **********************************************************************************************************************
// POSICION GEOGRAFICA
// **********************************************************************************************************************
class LatLng {
    private:
	    int _deg;
        int _min;
	    double _sec;
        bool _cardinal;					// true = Norte -- false = Sur */* true = Este | false = Oeste
    public:
        void setLatLng(int degreess, int minutes, double seconds, boolean cardinal) {
            _deg = degreess;
            _min = minutes;
            _sec = seconds;
            _cardinal = cardinal;
        };
        int getDeg() {
            return _deg;
        };
        int getMin() {
            return _min;
        };
        double getSec() {
            return _sec;
        };
        boolean getCardinal() {
            return _cardinal;
        };
};



// **********************************************************************************************************************
// CLASE QUE REPRESENTA UN DIA
// **********************************************************************************************************************
class Date {
    private:
        int _year;
        int _month;
        double _day;
    public:
        void setDate(int year, int month, double day) {
            _year = year;
            _month = month;
            _day = day;
        };
        int getYear() {
            return _year;
        };
        int getMonth() {
            return _month;
        };
        double getDay() {
            return _day;
        };
        void setDay(double day) {
            _day = day;
        }
};


// **********************************************************************************************************************
// CLASE QUE REPRESENTA LA HORA
// **********************************************************************************************************************
class Hour {
    private:
        int _hour;
        int _min;
        double _sec;
    public:
        void setTime(int hours, int minutes, double seconds) {
            _hour = hours;
            _min = minutes;
            _sec = seconds;
        };
        int getHour() {
            return _hour;
        };
        int getMin() {
            return _min;
        };
        double getSec() {
            return _sec;
        };
};


// **********************************************************************************************************************
// CLASE QUE REPRESENTA LA FECHA Y HORA - Hereda de Date y de Hour
// **********************************************************************************************************************
class DateTime : public Date, public Hour { };


// **********************************************************************************************************************
// CLASE QUE REPRESENTA UN SISTEMA DE COORDENADAS ECUATORIAL
// **********************************************************************************************************************
class Equatorial {
    private:
        long _RA;
        long _DEC;
    public:
        void setRA(long ra) {
            _RA = ra;
        };
        void setDEC(long dec) {
            _DEC = dec;
        };
        long getRA() {
            return _RA;
        };
        long getDEC() {
            return _DEC;
        };
};


// **********************************************************************************************************************
// INSTANCIACION DE OBJETOS
// **********************************************************************************************************************
LatLng LatSite;                                                                 // Latitud del sitio de observacion
LatLng LngSite;                                                                 // Latitud del sitio de observacion
Date LocalCivilDate;		                                                    // Fecha Civil Local
Hour LocalCivilTime;		                                                    // Tiempo Civil Local
Equatorial EquTeles;                                                            // Coordenadas ecuatoriales telescopio
Equatorial EquTarget;                                                           // Coordenadas ecuatoriales objetivo

// ACCELSTEPPER
AccelStepper stepperAz(AccelStepper::DRIVER, STEP_AZ_MOTOR, DIR_AZ_MOTOR);
AccelStepper stepperAlt(AccelStepper::DRIVER, STEP_ALT_MOTOR, DIR_ALT_MOTOR);
MultiStepper StepperControl;                                                    // Create instance of MultiStepper

// KEYPAD
Keypad keyPad(makeKeymap(keys), PINSROWS, PINSCOLUMNS, ROWS, COLUMNS);

// LCD DISPLAY
LiquidCrystal_I2C lcd(0x27, 20, 4);

// RTC - REAL TIME CLOCK
DS1302 rtc(RST, DAT, CLK);  // Tiempo de reloj
Time t;




// **********************************************************************************************************************
// GLOBAL VARIABLES
// **********************************************************************************************************************

// Variables para manejar la comunicacion serie
#define MAX_INPUT 20
char input[MAX_INPUT];      // Almacena la informacion recibida de Stellarium


const int buttonAlign = 53;         // pin in cui l'interruttore abilita l'allineamento con una stella
int buttonState = 0;                // variabile in cui arduino andrà a leggere lo stato logico di buttonAlign
long RAtel, DECtel;                 // coordinate del telescopio espresse in secondi
long RAtarget, DECtarget;           // coordinate del target espresse in secondi
long RAdist, RAdiff, DECdiff;       // differenza tra le coordinate del target e del telescopio

long memoDEC;                       // memorizza DECtarget (serve se si attraversa il polo celeste)
long latitude;
int A = 0; 
boolean circumpolar, enableGoTo;




void setup() {
    initialization();
    setLocalPosition();                             // Setea la latitud y longitud del lugar
    calcLocalDateTime();                            // Calcula Fecha y hora local
    pinMode(buttonAlign, INPUT);
    setTelescopeOrientation();
}


void loop() {
    recalculationOfTimes();                         // Recalcula cada un segundo tiempo local, universial, Sideral Local y Greenwich
    if (Serial.available() > 0) {
        handleIncomingByte(Serial.read());
    }
    diffBetweenTargetAndTelescope();

    if ((A == 1) && (DECdiff == 0)) {
        A = 0;
        if (RAtel >= 43200) {
            RAtel = RAtel - 43200;
        } else {
            RAtel = RAtel + 43200;
        }
        DECtarget = memoDEC;
        DECdiff = DECtarget - DECtel;
    }
    if ((A == 0) && (DECdiff == 0) && (RAdiff == 0)) {
        enableGoTo = false;
    }
    if((enableGoTo == true) && ((RAdiff != 0) || (DECdiff != 0))) {
        goToObject();
    }
}


// **********************************************************************************************************************
// INICIALIZA COMUNICACION, PANTALLA Y VARIABLES GLOBALES, CONFIGURA MOTORES
// **********************************************************************************************************************
void initialization() {
    Serial.begin(BAUDS);                        // Inicializa comunicacion serie 9600 baudios
    
    lcd.begin();                                // Inicializa LCD
    lcd.backlight();
    
    stepperAz.setMaxSpeed(2000);                // SPEED = Steps / second
    stepperAz.setAcceleration(100);             // ACCELERATION = Steps /(second)^2
    stepperAz.disableOutputs();                 // disable outputs
    
    // Interrupt externo. Ejecutado cada vez que se presiona el boton del Joystic
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptHandler, FALLING);

    // interrup de reloj para ejecutar la funcion run de los motores
    Timer1.attachInterrupt(timerISR);
	Timer1.start(50);

    circumpolar = false;
    enableGoTo = false;
}



void timerISR() {
    stepperAz.run();
}


// **********************************************************************************************************************
// SETEA LA LATITUD Y LONGITUD DEL SITIO DE OBSERVACION
// **********************************************************************************************************************
void setLocalPosition() {
	LatSite.setLatLng(19, 24, 15.48, true);                     // PERMITIR QUE EL USUARIO INGRESE LOS VALORES
	LngSite.setLatLng(99, 11, 34.09, false);                    // PERMITIR QUE EL USUARIO INGRESE LOS VALORES
    latitude = LatSite.getDeg() * 3600L + LatSite.getMin() * 60L + LatSite.getSec();
    if (!LatSite.getCardinal()) {
        latitude *= -1;
    }
    // IMPRIMIR EN PANTALLA LA UBICACION
    // lcdPrintLatLng(LatSite, 1, "Lat");
    // lcdPrintLatLng(LngSite, 1, "Lng");
}


// **********************************************************************************************************************
// SETEO DE LA FECHA Y HORA LOCAL TOMANDO SU VALOR DESDE EL RELOJ LOCAL (RTC SD1302)
// **********************************************************************************************************************
void calcLocalDateTime() {
	t = rtc.getTime();											        // Obtiene la Fehca y Hora local del reloj
	LocalCivilDate.setDate(t.year, t.mon, t.date);				        // Configura la Fecha local con la fecha de reloj
	LocalCivilTime.setTime(t.hour, t.min, t.sec);				        // Configura la Hora local con la fecha de reloj
    
    // IMPRIMIR EN PANTALLA LA FECHA Y HORA
    // lcdPrintDateTime(LocalCivilDate, LocalCivilTime);
}


// **********************************************************************************************************************
// RECALCULA FECHA Y HORA LOCAL, UNIVERSAL Y SIDERAL
void recalculationOfTimes() {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        calcLocalDateTime();
    }
}


// **********************************************************************************************************************
// CALCULO DE DIFERENCIA ENTRE COORDENADAS AZIMUTALES DE TELESCOPIO Y OBJETIVO
void diffBetweenTargetAndTelescope() {
    RAdiff = RAtarget - RAtel;
    DECdiff = DECtarget - DECtel;
}


// **********************************************************************************************************************
// SETEO DE LAS COORDENADAS AZIMUTALES INICIALES DEL TELESCOPIO Y DEL OBJETIVO
void setTelescopeOrientation() {
    RAtarget = RAtel = 9111L;                  // coordinata iniziale di AR (in secondi) per visualizzare il reticolo del telescopio
    DECtarget = DECtel = 321351L;           // coordinata iniziale di DEC (in secondi) per visualizzare il reticolo del telescopio
}
      

// **********************************************************************************************************************
// COMUNICACION SERIE NO BLOQUEANTE ENTRE STELLARIUM/ARDUINO DE ACUERDO AL PROTOCOLO MEADE LX200
void handleIncomingByte(const byte inByte) {
    /// http://www.gammon.com.au/serial
    /// process incoming serial data without blocking
    static unsigned int input_pos = 0;
    switch (inByte) {
        case '#':                                                       // end of incomming text (Ej: :GR# -> Get Telescope RA )
            input[input_pos] = 0;                                       // terminating null byte
            // terminator reached! process input here ...
            process_data(input);
            input_pos = 0;                                              // reset buffer for next time
            break;

        case '\r':   // discard carriage return
            break;

        default:
            // keep adding if not full ... allow for terminating null byte
            if (input_pos < (MAX_INPUT - 1)) {
                input[input_pos++] = inByte;
            }
            break;
    }
}


// **********************************************************************************************************************
// PROCESA LA INFORMACION RECIBIDA POR PUERTO SERIE Y ALMACENADA EN LA VARIABLE INPUT (DE ACUERDO AL PROTOCOLO MEADE LX200)
void process_data (const char * data) {
	// Telescope Command Groupings
	// Set Information 			S
	// Get Information 			G
	// Quit Command 			Q
	// Movement 				M

	// :GR# Get telescope right ascension
	// Stellarium solicita la ASCENSION RECTA del telescopio en el formtato HH:MM:SS#
	if (input[0] == ':' && input[1] == 'G' && input[2] == 'R') {
		sendTelescopeRA();
	} else
	// :GD# Get telescope declination
	// Stellarium solicita la DECLINACION del telescopio en el formato sDD*MM’SS#
	if (input[0] == ':' && input[1] == 'G' && input[2] == 'D') {
		sendTelescopeDEC();
	} else
	// con il comando #:Q# stellarium chiede l'arresto dei motori
	if (input[1] == ':' && input[2] == 'Q') {
		// Hacer que se detengan los motores
	} else
	// :Sr
	// Stellarium envia la coordenada de AR
	if (input[0] == ':' && input[1] == 'S' && input[2] == 'r') {
		getTargetRA();
	} else
	// :Sd
	// Con este comando Stellarium envia la coordenada de DEC
	if (input[0] == ':' && input[1] == 'S' && input[2] == 'd') {
		getTargetDEC();
	} else
	// :MS#
	// Comando de Stellarium que ordena el movimiento hacia el objetivo seleccionado
	//  	Stellarium espera como respuesta uno de tres valores (0, 1, 2)
	//		RESPUESTA:	0 			--> Significa Slew is posible (Es posible dirigirse al objeto seleccionado)
	//		RESPUESTA:	1<string># 	--> Significa Object Below Horizon w/string message
	//		RESPUESTA:	2<string># 	--> Significa Object Below Higher w/string message
	if (input[0] == ':' && input[1] == 'M' && input[2] == 'S') {
        Serial.print("0");
        // buttonState = digitalRead(buttonAlign);
        // if (buttonState == HIGH) {
        //     RAtel = RAtarget;
        //     DECtel = DECtarget;
        // }
        enableGoTo = true;
        lcd.setCursor(0,3);
        lcd.print("Entra");
    }
}
   
  
// **********************************************************************************************************************
// ENVIA VALOR ACTUAL DE AR DE TELESCOPIO A STELLARIUM CONFORME
// **********************************************************************************************************************
void sendTelescopeRA() {
    char txRA[10];
    long RAhours = RAtel / 3600;
    long RAmin = (RAtel - RAhours * 3600) / 60;
    long RAsec = (RAtel - RAhours * 3600) - RAmin * 60;
    sprintf(txRA, "%02d:%02d:%02d#", int(RAhours), int(RAmin), int(RAsec));
    Serial.print(txRA);
}
    

// **********************************************************************************************************************
// ENVIA VALOR ACTUAL DE DEC DE TELESCOPIO A STELLARIUM CONFORME
// **********************************************************************************************************************
void sendTelescopeDEC() {
    char DECsign;
    char txDEC[11];
    (DECtel < 0) ? DECsign = 45: DECsign = 43;
    long DECdeg = abs(DECtel) / 3600;
    long DECmin = (abs(DECtel) - DECdeg * 3600) / 60;
    long DECsec = (abs(DECtel) - DECdeg * 3600) - DECmin * 60;
    sprintf(txDEC, "%c%02d%c%02d:%02d#", DECsign, int(DECdeg), 42, int(DECmin), int(DECsec));
    Serial.print(txDEC);
}
    

// **********************************************************************************************************************
// RECIBE DESDE STELLARIUM LA ASCENSION RECTA DEL OBJETIVO
// **********************************************************************************************************************
void getTargetRA() {
    Serial.print("1");
    RAtarget = (atol(input + 3)) * 3600 + (atol(input + 6)) * 60 + atol(input + 9);
}
  

// **********************************************************************************************************************
// RECIBE DESDE STELLARIUM LA DECLINACION DEL OBJETIVO
// **********************************************************************************************************************    
void getTargetDEC() {
    Serial.print("1");
    DECtarget = (atol(input + 4)) * 3600 + (atol(input + 7)) * 60 + atol(input + 10);
    if (input[3] == '-') {
        DECtarget *=(-1);
    }
    long betaN, deltaN, betaS, deltaS;
    memoDEC = DECtarget;
    betaN = (324000 - DECtel) + (324000 - DECtarget);
    deltaN = betaN + ((648000 - betaN) / 2);
    betaS = ((-324000) - DECtel) + ((-324000) - DECtarget);
    deltaS = abs(betaS) + ((648000 - abs(betaS)) / 2);
            
    isCircumpolar();
    RAdist = RAtarget - RAtel;
    if (RAdist < (-43200)) {
        RAdist = RAdist + 86400;
    }
    if (RAdist > 43200) {
        RAdist = RAdist - 86400;
    }
    if ((latitude > 0) && (circumpolar = true) && ((abs(RAdist) * 15) > deltaN)) {
        DECtarget = 324000;
        A = 1;
    }    
    if ((latitude < 0) && (circumpolar = true) && ((abs(RAdist) * 15) > deltaS)) {
        DECtarget = (-324000);
        A = 1;
    }
}


void isCircumpolar() {
    if (((latitude > 0) && (DECtarget > (324000 - latitude))) || ((latitude < 0) && (DECtarget < ((-324000) - latitude)))) {
        circumpolar = true;
    } else {
        circumpolar = false;
    }
}

  
// **********************************************************************************************************************
// Se invoca dento del loop solo si la diferencia entre las coordenadas
// del telescopio y el objetivo es diferente de cero.
// Compara las coordenadas y elige el camino más corto para llegar al
// objetivo invocando a su vez la función encargada del movimiento deseado.
// **********************************************************************************************************************
void goToObject(){  
    if (A != 1) {
        if ((RAdiff >0 && RAdiff <= 43200) || (RAdiff <= (-43200))) {
            go_east();
        }
        if ((RAdiff > 43200) ||  (RAdiff <0 && RAdiff > (-43200))) {
            go_west();
        }
    }
    if (DECtarget > DECtel) {
        go_north();
    }
    if (DECtarget < DECtel) {
        go_south();
    } 
}



// **********************************************************************************************************************
// DECREMENTA AZIMUT DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE AZIMUT
// **********************************************************************************************************************
void go_east() {
    RAtel++;
    if (RAtel >= 86400) {
        RAtel = RAtel - 86400;
    }
    // inviare comando al motore AR
}


// **********************************************************************************************************************
// DECREMENTA AZIMUT DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE AZIMUT
// **********************************************************************************************************************
void go_west() {
    RAtel--;
    if (RAtel < 0) {
        RAtel = RAtel + 86400;
    }
    // inviare comando al motore AR
}


// **********************************************************************************************************************
// DECREMENTA AZIMUT DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE AZIMUT
// **********************************************************************************************************************
void go_north() {
    DECtel++;
    // inviare comando al motore DEC
}



// **********************************************************************************************************************
// DECREMENTA AZIMUT DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE AZIMUT
// **********************************************************************************************************************
void go_south() {
    DECtel--;
    // inviare comando al motore DEC
}





// -----------------------------------------------------------------------------------------------
// LCD DISPLAY
void clearLCDLine(byte line) {               
    lcd.setCursor(0,line);
    for(int n = 0; n < 20; n++) // 20 indicates symbols in line. For 2x16 LCD write - 16
    {
            lcd.print(" ");
    }
}


void lcdPrintManualOperation() {
    lcd.setCursor(0,1);
    lcd.print("Operacion manual");
}





// **********************************************************************************************************************
// FUNCIONES DE IMPRESION EN LCD
// **********************************************************************************************************************
void lcdPrintLatLng(LatLng latLng, byte row, char latOrLng[]) {
    if (latOrLng == "Lat") {
        lcd.setCursor(0, row);
        lcd.print("Lat: ");
        if (latLng.getCardinal() == true) {
            lcd.setCursor(17, row);
            lcd.print("N");
        } else {
            lcd.setCursor(17, row);
            lcd.print("S");
        }
    } else if (latOrLng == "Lng"){
        lcd.setCursor(0, row);
        lcd.print("Lng: ");
        if (latLng.getCardinal() == true) {
            lcd.setCursor(17, row);
            lcd.print("E");
        } else {
            lcd.setCursor(17, row);
            lcd.print("O");
        }
    }
    lcd.setCursor(5, row);
    lcd.print(latLng.getDeg());
    lcd.setCursor(7, row);
    lcd.print(":");
    lcd.setCursor(8, row);
    lcd.print(latLng.getMin());
    lcd.setCursor(10, row);
    lcd.print(":");
    lcd.setCursor(11, row);
    lcd.print(latLng.getSec(), 2);
}





void lcdPrintDateTime(Date date, Hour time) {
    // DATE    
    byte d = date.getDay();
    if (d < 10) {
        lcd.setCursor(0, 0);
        lcd.print(0);
        lcd.setCursor(1, 0);
        lcd.print(d);
    } else {
        lcd.setCursor(0, 0);
        lcd.print(d);
    }
    lcd.setCursor(2, 0);
    lcd.print("-");

    byte m = date.getMonth();
    if (m < 10) {
        lcd.setCursor(3, 0);
        lcd.print(0);
        lcd.setCursor(4, 0);
        lcd.print(m);
    } else {
        lcd.setCursor(3, 0);
        lcd.print(m);
    }

    lcd.setCursor(5, 0);
    lcd.print("-");

    lcd.setCursor(6, 0);
    lcd.print(date.getYear());
    

    // TIME
    byte h = time.getHour();
    if (h < 10) {
        lcd.setCursor(12, 0);
        lcd.print(0);
        lcd.setCursor(13, 0);
        lcd.print(h);
    } else {
        lcd.setCursor(12, 0);
        lcd.print(h);
    }

    lcd.setCursor(14, 0);
    lcd.print(":");

    byte min = time.getMin();
    if (min < 10) {
        lcd.setCursor(15, 0);
        lcd.print(0);
        lcd.setCursor(16, 0);
        lcd.print(min);
    } else {
        lcd.setCursor(15, 0);
        lcd.print(min);
    }

    lcd.setCursor(17, 0);
    lcd.print(":");

    byte sec = time.getSec();
    if (sec < 10) {
        lcd.setCursor(18, 0);
        lcd.print(0);
        lcd.setCursor(19, 0);
        lcd.print(sec);
    } else {
        lcd.setCursor(18, 0);
        lcd.print(sec);
    }
}







void lcdPrintLST(Hour time, byte row) {
    lcd.setCursor(0, row);
    lcd.print("LST:");
    byte h = time.getHour();
    if (h < 10) {
        lcd.setCursor(4, row);
        lcd.print(0);
        lcd.setCursor(5, row);
        lcd.print(h);
    } else {
        lcd.setCursor(4, row);
        lcd.print(h);
    }

    lcd.setCursor(6, row);
    lcd.print(":");

    byte min = time.getMin();
    if (min < 10) {
        lcd.setCursor(7, row);
        lcd.print(0);
        lcd.setCursor(8, row);
        lcd.print(min);
    } else {
        lcd.setCursor(7, row);
        lcd.print(min);
    }

    lcd.setCursor(9, row);
    lcd.print(":");

    byte sec = time.getSec();
    if (sec < 10) {
        lcd.setCursor(10, row);
        lcd.print(0);
        lcd.setCursor(11, row);
        lcd.print(sec);
    } else {
        lcd.setCursor(10, row);
        lcd.print(sec);
    }
}



void lcdPrintAz(Hour time) {
    lcd.setCursor(0, 3);
    lcd.print("LST:");
    byte h = time.getHour();
    if (h < 10) {
        lcd.setCursor(4, 3);
        lcd.print(0);
        lcd.setCursor(5, 3);
        lcd.print(h);
    } else {
        lcd.setCursor(4, 3);
        lcd.print(h);
    }

    lcd.setCursor(6, 3);
    lcd.print(":");

    byte min = time.getMin();
    if (min < 10) {
        lcd.setCursor(7, 3);
        lcd.print(0);
        lcd.setCursor(8, 3);
        lcd.print(min);
    } else {
        lcd.setCursor(7, 3);
        lcd.print(min);
    }

    lcd.setCursor(9, 3);
    lcd.print(":");

    byte sec = time.getSec();
    if (sec < 10) {
        lcd.setCursor(10, 3);
        lcd.print(0);
        lcd.setCursor(11, 3);
        lcd.print(sec);
    } else {
        lcd.setCursor(10, 3);
        lcd.print(sec);
    }
}




// **********************************************************************************************************************
// FUNCIONES MOVIMIENTO DE MOTOR
// **********************************************************************************************************************

// Rotacion relativa de motor 
void RotateRelative() {
    //We move X steps from the current position of the stepper motor in a given direction.
    //The direction is determined by the multiplier (+1 or -1)   
    runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepperAz.setMaxSpeed(receivedSpeed); //set speed
    stepperAz.move(direction * receivedSteps); //set relative distance and direction
}


// Rotacion absoluta de motor 
void RotateAbsolute() {
    //We move to an absolute position.
    //The AccelStepper library keeps track of the position.
    //The direction is determined by the multiplier (+1 or -1)
    //Why do we need negative numbers? - If you drive a threaded rod and the zero position is in the middle of the rod...
 
    runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepperAz.setMaxSpeed(receivedSpeed); //set speed
    stepperAz.moveTo(direction * receivedSteps); //set relative distance   
}



void GoHome() {  
    if (stepperAz.currentPosition() == 0) {
        Serial.println("We are at the home position.");
        stepperAz.disableOutputs(); //disable power
    } else {
        stepperAz.setMaxSpeed(400); //set speed manually to 400. In this project 400 is 400 step/sec = 1 rev/sec.
        stepperAz.moveTo(0); //set abolute distance to move
    }
}



// Define que tipo de movimiento realiza el motor (Relativo - Absoluto - GoHome)
void moveMotor(char moveType, unsigned long steps, unsigned long speed) {
    receivedCommand = moveType;    // pass byte to the receivedCommad variable
    newData = true;                     //indicate that there is a new data by setting this bool to true
    if (newData == true) {
        switch (receivedCommand) {
            case 'P':                   //P uses the move() function of the AccelStepper library, which means that it moves relatively to the current position.              
                receivedSteps = steps;        //value for the steps
                receivedSpeed = speed;        //value for the speed
                direction = 1;                    //We define the direction
                RotateRelative(); //Run the function
                //example: P2000 400 - 2000 steps (5 revolution with 400 step/rev microstepping) and 400 steps/s speed
                //In theory, this movement should take 5 seconds
                break;
            
            case 'N': //N uses the move() function of the AccelStepper library, which means that it moves relatively to the current position.
                receivedSteps = steps;        //value for the steps
                receivedSpeed = speed;        //value for the speed 
                direction = -1; //We define the direction
                RotateRelative(); //Run the function
                //example: N2000 400 - 2000 steps (5 revolution with 400 step/rev microstepping) and 500 steps/s speed; will rotate in the other direction
                //In theory, this movement should take 5 seconds
                break;

            case 'R': //R uses the moveTo() function of the AccelStepper library, which means that it moves absolutely to the current position.            
                receivedSteps = steps;        //value for the steps
                receivedSpeed = speed;        //value for the speed
                direction = 1; //We define the direction
                RotateAbsolute(); //Run the function
                //example: R800 400 - It moves to the position which is located at +800 steps away from 0.
                break;

            case 'r': //r uses the moveTo() function of the AccelStepper library, which means that it moves absolutely to the current position.            
                receivedSteps = steps;        //value for the steps
                receivedSpeed = speed;        //value for the speed
                direction = -1; //We define the direction
                RotateAbsolute(); //Run the function
                //example: r800 400 - It moves to the position which is located at -800 steps away from 0.
                break;

            case 'S': // Stops the motor
                stepperAz.stop(); //stop motor
                stepperAz.disableOutputs(); //disable power
                // Serial.println("Stopped."); //print action
                runallowed = false; //disable running
                break;


            // case 'A': // Updates acceleration
            //     runallowed = false; //we still keep running disabled, since we just update a variable
            //     stepperAz.disableOutputs(); //disable power
            //     receivedAcceleration = Serial.parseFloat(); //receive the acceleration from serial
            //     stepperAz.setAcceleration(receivedAcceleration); //update the value of the variable
            //     // Serial.print("New acceleration value: "); //confirm update by message
            //     // Serial.println(receivedAcceleration); //confirm update by message
            //     break;

        case 'H': //H: Homing
            runallowed = true;     
            // Serial.println("Homing"); //Print the message
            GoHome();// Run the function
            break;

        default:
            break;
        }
    }
    newData = false;
}















// **********************************************************************************************************************
// JOYSTIC CONTROL
// **********************************************************************************************************************
// Rutina de servicio de interrupción.
void interruptHandler() {
    static unsigned long last_call_interrupt = 0;
    unsigned long interrupt_length = millis();
    if (interrupt_length - last_call_interrupt > DEBOUNCE_DELAY) {
        switch (switchBtn) {
            case false:
                switchBtn = true;
                break;
            case true:
                switchBtn = false;
                break;
        }
    }
    last_call_interrupt = interrupt_length;
}



void joysticManualSlew() {
    if (switchBtn == true) {  // Enable moving of steppers using the Joystick
        if (analogRead(X) < 500) {
            joystepX = joystepX - 1;
        } else
        if (analogRead(X) > 900) {
            joystepX = joystepX + 1;
        }
        if (analogRead(Y) < 200) {         
            // joystepY=stepperAlt.currentPosition();         
            joystepY = joystepY - 1;
        }
        if (analogRead(Y) > 900) {
            // joystepY=stepperAlt.currentPosition();
            joystepY = joystepY + 1;
        }
        stepperAz.moveTo(joystepX);
        stepperAlt.moveTo(joystepY);
    }
}
