#include <Wire.h>
#include <LiquidCrystal_I2C.h>      // LCD
#include <stdlib.h>                 // 
#include <math.h>                   // MATEMATICA
#include <DS1302.h>                 // RTC
#include <AccelStepper.h>           // STEPPER MOTOR
#include <MultiStepper.h>           // MULTIPLE STEPPER MOTOR
#include <Keypad.h>                 // KEYPAD
#include "telescope.h"              
#include <DueTimer.h>               // INTERNAL TIME INTERRUPT PARA ARDUINO DUE


// **********************************************************************************************************************
// POSICION GEOGRAFICA
class LatLng {
    private:
	    int _deg;
        int _min;
	    double _sec;
        bool _cardinal;					// true = Norte -- false = Sur |||| true = Este | false = Oeste
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
// CLASE QUE REPRESENTA LA FECHA Y HORA
// Clase que hereda de Date y de Hour
class DateTime : public Date, public Hour { };


// **********************************************************************************************************************
// CLASE QUE REPRESENTA UN SISTEMA DE COORDENADAS ECUATORIAL
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
// CLASE QUE REPRESENTA UN SISTEMA DE COORDENADAS ALTAZIMUTAL
class Horizontal {
    private:
        long _Az;
        long _alt;
    public:
        void setAz(long Az) {
            _Az = Az;
        };
        void setAlt(long alt) {
            _alt = alt;
        };
        long getAz() {
            return _Az;
        };
        long getAlt() {
            return _alt;
        };
};


// **********************************************************************************************************************
// FUNCTIONS PROTOTYPES
DateTime LCTtoUTandGCD(Date date, Hour time, int timeZone, byte summerTime);
Hour DHtoHMS(double decimalHours);
double UTtoGST(double UD, double JD);
double GSTtoLST(double GST, LatLng SiteLng);
void EQtoAZ(Equatorial &Eq, Horizontal &Ho);
Date JDtoGCD(double JD);
double GCDtoJD(Date date);
void lcdPrintLatLng(LatLng latLng, byte row, char latOrLng[]);
void lcdPrintDateTime(Date date, Hour time);
void lcdPrintLST(Hour time, byte row);
void lcdPrintAz(Hour time);


// **********************************************************************************************************************
// INSTANCIACION DE OBJETOS
LatLng LatSite;                                                                 // Latitud del sitio de observacion
LatLng LngSite;                                                                 // Latitud del sitio de observacion

Date LocalCivilDate;		                                                    // Fecha Civil Local
Hour LocalCivilTime;		                                                    // Tiempo Civil Local

Equatorial EquTeles;                                                            // Coordenadas ecuatoriales telescopio
Horizontal HorTeles;                                                            // Coordenadas Altazimutales telescopio
Equatorial EquTarget;                                                           // Coordenadas ecuatoriales objetivo
Horizontal HorTarget;                                                           // Coordenadas Altazimutales objetivo

// ACCELSTEPPER INSTANCES FOR MOTORS
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
// https://24timezones.com/world_directory/current_mexico_city_time.php
                                    // Standard time zone:	        UTC/GMT -6 hours
                                    // Daylight saving time:	    +1 hour
                                    // Current time zone offset:	UTC/GMT -5 hours
int TIMEZONE = -6;                  // (Hours ahead of UTC)
byte DST = 1;                       // Value = 1 if summer time, in winter value = 0

const int buttonAlign = 53;
int buttonState = 0;
const unsigned int MAX_INPUT = 20;
char input[MAX_INPUT];              // Almacena la informacion recibida de Stellarium
long RAtel,                         // Coordenada Ascension Recta telescopio
    DECtel,                         // Coordenada Declinacion telescopio        
    RAtarget,                       // Coordenada Ascension Recta objetivo
    DECtarget;                      // Coordenada Declinacion objetivo
long A_diff, h_diff;                // Diferencia Azimut, Altura entre telescopio y objetivo
boolean enableGoTo, targeReached = false;
double GST, UT, JD;
long LST,
    A_tel,                          // Coordenada Azimut telescopio
    h_tel,                          // Coordenada altura telescopio
    H_tel,                          // Angulo Horario telescopio
    H_target,                       // Angulo horario objetivo
    h_target,                       // Coordenada altura objetivo
    A_target;                       // Coordenada Azimut objetivo 
double sinLatRAD, cosLatRAD;




// **********************************************************************************************************************
void setup() {
    start();
    setLocalPosition();                             // Setea la latitud y longitud del lugar
    calcLocalDateTime();                            // Calcula Fecha y hora local
    calcUnivlDateTimeAndGreenwichCalendarDate();
	calcSideralTimes();
    pinMode(buttonAlign, INPUT);
    telescopeOrientation();
}


void loop() {
    recalculationOfTimes();                         // Recalcula cada un segundo tiempo local, universial, Sideral Local y Greenwich
    joysticManualSlew();
    if (Serial.available() > 0) {
        handleIncomingByte(Serial.read());
    }
    diffBetweenTargetAndTelescope();				// Calcula dif entre coord. Azimutales del objetivo y telescopio
    if (enableGoTo == true) {
        goToObject();                               // Se ejecuta si dif entre coord. del elescopio y objetivo son dif de 0
    }
}


// **********************************************************************************************************************
// INICIALIZA COMUNICACION, PANTALLA Y VARIABLES GLOBALES, CONFIGURA MOTORES
void start() {
    // Inicializa comunicacion serie 9600 baudios
    Serial.begin(BAUDS);

    // Inicializa LCD
    lcd.begin();
    lcd.backlight();
    
    // Seteo de motores por defecto
    stepperAz.setMaxSpeed(2000);                      // SPEED = Steps / second
    stepperAz.setAcceleration(100);                   // ACCELERATION = Steps /(second)^2
    stepperAz.disableOutputs();                       // disable outputs
    
    
    // Interrupt externo. Ejecutado cada vez que se presiona el boton del Joystic
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interruptHandler, FALLING);

    // interrup de reloj para ejecutar la funcion run de los motores
    Timer1.attachInterrupt(timerISR);
	Timer1.start(50);
}



void timerISR() {
    stepperAz.run();
}



// **********************************************************************************************************************
// SETEO DE LAS COORDENADAS AZIMUTALES INICIALES DEL TELESCOPIO Y DEL OBJETIVO
void telescopeOrientation() {
    A_target = A_tel = 0L;
    h_target = h_tel = 0L;
}


// **********************************************************************************************************************
// RECALCULA FECHA Y HORA LOCAL, UNIVERSAL Y SIDERAL
void recalculationOfTimes() {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        calcLocalDateTime();
        calcUnivlDateTimeAndGreenwichCalendarDate();
        calcSideralTimes();
    }
}


// **********************************************************************************************************************
// SETEA LA LATITUD Y LONGITUD DEL SITIO DE OBSERVACION
void setLocalPosition() {
	LatSite.setLatLng(19, 24, 15.48, true);                     // PERMITIR QUE EL USUARIO INGRESE LOS VALORES
	LngSite.setLatLng(99, 11, 34.09, false);                    // PERMITIR QUE EL USUARIO INGRESE LOS VALORES
    double LATdeg = DMStoDD(LatSite.getDeg(), LatSite.getMin(), LatSite.getSec(), LatSite.getCardinal());
    sinLatRAD = sin(LATdeg * rad);
	cosLatRAD = cos(LATdeg * rad);

    // IMPRIMIR EN PANTALLA LA UBICACION
    // lcdPrintLatLng(LatSite, 1, "Lat");
    // lcdPrintLatLng(LngSite, 1, "Lng");
}


// **********************************************************************************************************************
// SETEO DE LA FECHA Y HORA LOCAL TOMANDO SU VALOR DESDE EL RELOJ LOCAL (RTC SD1302)
void calcLocalDateTime() {
	t = rtc.getTime();											        // Obtiene la Fehca y Hora local del reloj
	LocalCivilDate.setDate(t.year, t.mon, t.date);				        // Configura la Fecha local con la fecha de reloj
	LocalCivilTime.setTime(t.hour, t.min, t.sec);				        // Configura la Hora local con la fecha de reloj
    
    // IMPRIMIR EN PANTALLA LA FECHA Y HORA
    lcdPrintDateTime(LocalCivilDate, LocalCivilTime);
}


// **********************************************************************************************************************
// CALCULO DE LA FECHA Y HORA UNIVERSAL A PARTIR DE LA HORA LOCAL
void calcUnivlDateTimeAndGreenwichCalendarDate() {
    DateTime UTandGCD = LCTtoUTandGCD(LocalCivilDate, LocalCivilTime, TIMEZONE, DST);   // Local Civil Time a Universal Time
    UT = HMStoDH(UTandGCD.getHour(), UTandGCD.getMin(), UTandGCD.getSec());
    JD = GCDtoJD(UTandGCD);                             // Corroborar: http://www.jgiesen.de/astro/astroJS/siderealClock/
}


// **********************************************************************************************************************
// CALCULO DE GST Y LST A PARTIR DE LA FEHCA Y HORA UNIVERSAL
void calcSideralTimes() {
    GST = UTtoGST(UT, JD);                  // Corroborar mediante: https://community.dur.ac.uk/john.lucey/users/lst.html
    double lst = GSTtoLST(GST, LngSite);    // Corroborar mediante: http://www.jgiesen.de/astro/astroJS/siderealClock/
    LST = long(lst * 3600L);
    
    // IMPRIMIR EN PANTALLA LST
    lcdPrintLST(DHtoHMS(lst), 2);
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
		sendRA();
	} else
	// :GD# Get telescope declination
	// Stellarium solicita la DECLINACION del telescopio en el formato sDD*MM’SS#
	if (input[0] == ':' && input[1] == 'G' && input[2] == 'D') {
		sendDEC();
	} else
	// con il comando #:Q# stellarium chiede l'arresto dei motori
	if (input[1] == ':' && input[2] == 'Q') {
		// Hacer que se detengan los motores
	} else
	// :Sr
	// Stellarium envia la coordenada de AR
	if (input[0] == ':' && input[1] == 'S' && input[2] == 'r') {
		getRA();
	} else
	// :Sd
	// Con este comando Stellarium envia la coordenada de DEC
	if (input[0] == ':' && input[1] == 'S' && input[2] == 'd') {
		getDEC();
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
// CALCULO DE DIFERENCIA ENTRE COORDENADAS AZIMUTALES DE TELESCOPIO Y OBJETIVO
void diffBetweenTargetAndTelescope() {
    A_diff = A_target - A_tel;
    h_diff = h_target - h_tel;
}


// **********************************************************************************************************************
// ENVIA VALOR ACTUAL DE AR DE TELESCOPIO A STELLARIUM CONFORME
void sendRA() {
    char txRA[10];
    convertAZtoEQ();
    long RAhours = RAtel / 3600L;
    long RAmin = (RAtel - RAhours * 3600L) / 60L;
    long RAsec = (RAtel - RAhours * 3600L) - RAmin * 60L;
    sprintf(txRA, "%02d:%02d:%02d#", int(RAhours), int(RAmin), int(RAsec));
    Serial.print(txRA);
}


// **********************************************************************************************************************
// ENVIA VALOR ACTUAL DE DEC DE TELESCOPIO A STELLARIUM CONFORME
void sendDEC() {
    char DECsign;
    char txDEC[11];
    convertAZtoEQ();
    (DECtel < 0) ? DECsign = 45: DECsign = 43;
    long DECdeg = abs(DECtel) / 3600L;
    long DECmin = (abs(DECtel) - DECdeg * 3600L) / 60L;
    long DECsec = (abs(DECtel) - DECdeg * 3600L) - DECmin * 60L;
    sprintf(txDEC, "%c%02d%c%02d:%02d#", DECsign, int(DECdeg), 42, int(DECmin), int(DECsec));
    Serial.print(txDEC);
}


// **********************************************************************************************************************
// RECIBE DESDE STELLARIUM LA ASCENSION RECTA DEL OBJETIVO
void getRA() {
    Serial.print("1");
    RAtarget = (atol(input + 3)) * 3600 + (atol(input + 6)) * 60 + atol(input + 9);
    enableGoTo = false;
}


// **********************************************************************************************************************
// RECIBE DESDE STELLARIUM LA DECLINACION DEL OBJETIVO
void getDEC() {
    Serial.print("1");
    DECtarget = (atol(input + 4)) * 3600 + (atol(input + 7)) * 60 + atol(input + 10);
    if (input[3] == '-') {
        DECtarget *=(-1);
    }
    enableGoTo = false;
    convertEQtoAZ();
}


// **********************************************************************************************************************
// TRANSFORMACION DE COORDENADAS AZIMUTALES A ECUATORIALES
void convertAZtoEQ() {
    double A_telRAD = (A_tel / 3600.0d) * rad;         // Azimut telescopio de segundos de arco a grados deimales a radianes
    double h_telRAD = (h_tel / 3600.0d) * rad;         // Altura telescopio de segundos de arco a grados deimales a radianes
    double DECsinRAD = (sin(h_telRAD) * sinLatRAD) + (cos(h_telRAD) * cosLatRAD * cos(A_telRAD));   // Seno declinacion
    double DECtelRAD = asin(DECsinRAD);                                                             // Declinacion en radianes
    DECtel = long((DECtelRAD / rad) * 3600.0d);            // DECLINACION en segundos de arco
    double y = -cos(h_telRAD) * cosLatRAD * sin(A_telRAD);
    double x = sin(h_telRAD) - sinLatRAD * DECsinRAD;
    double H_telRAD = atan2(y, x);                      // ANGULO HORARIO en radianes
    double H_telDEG = (H_telRAD / rad);                 // ANGULO HORARIO en grados decimales
    H_telDEG = H_telDEG - (360.0d * floor(H_telDEG / 360.0d));
    H_telDEG = (H_telDEG / 15);                        // Angulo Horario pasado a horas
    H_tel = long(H_telDEG * 3600.0d);                    // Angulo horario en segundos de arco
    RAtel = LST - H_tel;
    if (RAtel < 0L) {
        RAtel = RAtel + 86400L;
    }
}


// **********************************************************************************************************************
// TRANSFORMACION DE COORDENADAS ECUATORIALES A AZIMUTALES
void convertEQtoAZ() {
    H_target = LST - RAtarget;
    if (H_target < 0) {
        H_target = H_target + 86400L;
    }
    double H_targetRAD = ((H_target / 3600.0d) * 15.0d) * rad;
    double DECtargetRAD = (DECtarget / 3600.0d) * rad;
    double hsinRAD = sin(DECtargetRAD) * sinLatRAD + cos(DECtargetRAD) * cosLatRAD * cos(H_targetRAD);
    double htargetRAD = asin(hsinRAD);
    h_target = long((htargetRAD / rad) * 3600.0d);
    
    double y = -cos(DECtargetRAD) * cosLatRAD * sin(H_targetRAD);
    double x = sin(DECtargetRAD) - sinLatRAD * hsinRAD;
    double A_targetRAD = atan2(y, x);
    double A_targetDEC = (A_targetRAD / rad);
    A_targetDEC = A_targetDEC - (360.0d * floor(A_targetDEC / 360.0d));
    A_target = long(A_targetDEC * 3600.d);
}
  

// **********************************************************************************************************************
// Se invoca dento del loop solo si la diferencia entre las coordenadas
// del telescopio y el objetivo es diferente de cero.
// Compara las coordenadas y elige el camino más corto para llegar al
// objetivo invocando a su vez la función encargada del movimiento deseado.
void goToObject() {
    if (((A_diff > 0) && (A_diff <= DEG_180)) || (A_diff <= (-DEG_180))) {
        increment_A_tel();

        // lcd.setCursor(0, 1);
        // lcd.print("1");
    } else
    if ((A_diff > DEG_180) || ((A_diff < 0) && (A_diff > (-DEG_180)))) {
        decrement_A_tel();

        // lcd.setCursor(2, 1);
        // lcd.print("2");
    }
    if (h_target > h_tel) {
        go_up();

        // lcd.setCursor(4, 1);
        // lcd.print("3");
    } else
    if (h_target < h_tel) {
        go_down();

        // lcd.setCursor(6, 1);
        // lcd.print("4");
    }
    
    if ((A_diff == 0) && (h_diff == 0)) {   // Si la diferencia entre objetivo y telescopio es cero, cancelar goto
         enableGoTo = false;
         targeReached = true;
        // lcd.setCursor(7, 1);
        // lcd.print(A_diff);
        // lcd.setCursor(15, 1);
        // lcd.print(h_diff);
    }
}


// **********************************************************************************************************************
// INCREMENTA AZIMUT DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE AZIMUT
void increment_A_tel() {
    //comando al motor AR
    long avance;
    if (stepperAz.distanceToGo() == 0) {
        moveMotor('P', long(round(abs(A_diff * SIN_NOMBRE))), 500L);
        A_tel_from = A_tel;
        // avance = long(1.0d / SIN_NOMBRE);
    }
    // A_tel = A_tel_from + map(stepperAz.currentPosition(), 0, long(abs(A_diff * SIN_NOMBRE)), 0, A_diff);
    // A_tel = A_tel_from + (stepperAz.currentPosition() * avance);
    delay(long(round(abs(A_diff * SIN_NOMBRE))/500L));
    A_tel++;
    if (A_tel >= DEG_360) {
        A_tel = A_tel - DEG_360;
    }
    currentPosition = stepperAz.currentPosition();
}



// **********************************************************************************************************************
// DECREMENTA AZIMUT DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE AZIMUT
void decrement_A_tel() {
    // comando al motor AR
    long  avance;
    if (stepperAz.distanceToGo() == 0) {
        moveMotor('N', long(round(abs(A_diff * SIN_NOMBRE))), 500L);
        A_tel_from = A_tel;
        // avance = long(1.0d / SIN_NOMBRE);
    }
    // A_tel = A_tel_from - map(stepperAz.currentPosition(), 0, long(abs(A_diff * SIN_NOMBRE)), 0, A_diff);
    // A_tel = A_tel_from - (stepperAz.currentPosition() * avance);
    delay(long(round(abs(A_diff * SIN_NOMBRE))/500L));
    A_tel--;

    if (A_tel < 0) {
        A_tel = A_tel + DEG_360;
    }
}


// **********************************************************************************************************************
// INCREMENTA ALTURA DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE ALTURA
void go_up() {
    h_tel++;
    // comando al motor DEC
}


// **********************************************************************************************************************
// DECREMENTA ALTURA DE TELESCOPIO CONFORME AL MOVIMIENTO DEL MOTOR DE ALTURA
void go_down() {
    h_tel--;
    // comando al motor DEC
}



// **********************************************************************************************************************
// HORAS DECIMALES A HORAS MINUTOS Y SEGUNDOS
Hour DHtoHMS(double decimalHours) {
    long hour = long(decimalHours);
	long min = long((decimalHours - hour) * 60.0d);
	double sec = (((decimalHours - hour) * 60.0d) - min) * 60.0d;
	Hour h;
    h.setTime(hour, min, sec);
    return h;
}



// **********************************************************************************************************************
// Local Civil Time (Tiempo local) a Greenwich Meridian Universal Time (UT) (in decimal hours)
DateTime LCTtoUTandGCD(Date date, Hour time, int timeZone, byte summerTime) {
	double LCTDH = HMStoDH(time.getHour(), time.getMin(), time.getSec());     // Local civil time in Decimal hour
    double UT = LCTDH - summerTime - timeZone;
    double GDay = date.getDay() + (UT/24.0d);
    date.setDay(GDay);
    double JD = GCDtoJD(date);
    Date d = JDtoGCD(JD);
    GDay = d.getDay();
    double GMonth = d.getMonth();
    double GYear = d.getYear();
    UT = 24.0d * (GDay - long(GDay));
    Hour H = DHtoHMS(UT);
    DateTime dateTime;
    dateTime.setDate(GYear, GMonth, GDay);
    dateTime.setTime(H.getHour(), H.getMin(), H.getSec());
    return dateTime;
}


// **********************************************************************************************************************
// Hours Minutes Seconds to Decimal Hours
double HMStoDH(int hs, int min, double sec) {
	double s = sec / 60.0d;
    double m = (s + min) / 60.0d;
    double h = hs + m;
	return h;
}


// **********************************************************************************************************************
// Greenwich Calendar Date to Julian Date
double GCDtoJD(Date date) {
    int year = date.getYear();
    int month = date.getMonth();
    double day = date.getDay();
    // double t = HMStoDH(time.getHour(), time.getMin(), time.getSec());
	long A, B, C, D, E;
	// Calculo del dia juliano o fecha juliana
	// número de días y fracción transcurridos desde el mediodía del 1º de enero del año 4713 a. C
	if (month <= 2) {		// Si el mes es enero o febrero (1 o 2)
		month += 12;		// Sumar 12 al mes
		year -= 1;			// Restar 1 al año
	}

	A = long(year / 100L);
	B = long(A / 4L);
	C = 2 - A + B;
	D = long(365.25d * year);
	E = long(30.6001d * (month + 1));
	double JD = C + day + D + E + 1720994.5d;
	return JD;
}


// **********************************************************************************************************************
// Julian Date to Greenwich Calendar Day
Date JDtoGCD(double JD) {
	long I = long(JD + 0.5d);
	double F = JD + 0.5d - I;
	long A = long((I - 1867216.25d) / 36524.25d);
	long B = I;
	if (I > 2299160L) {
		B = I + 1 + A - long(A / 4);
	}
	long C = B + 1524;
	long D = long((C - 122.1d)/365.25d);
	long E = long(365.25d * D);
	long G = long((C - E) / 30.6001d);
	double d = C - E + F - long(30.6001d * G);

	double m = (G - 13);
	if (G < 13.5) {
		m = G - 1;
	}
	double y = (D - 4715);
	if (m > 2.5) {
		y = (D - 4716);
	}

	Date date;
    date.setDate(y, m, d);
 	return date;

}


// **********************************************************************************************************************
// Universal Time to Greenwich Sideral Time
double UTtoGST(double UT, double JD) {
	double S = JD - (UT/24.0d) - 2451545.0d;
	double T = S / 36525L;
	double T0 = 6.697374558d + (2400.051336d * T) + (0.000025862d * T * T);
	T0 = T0 - (24 * long(T0 / 24.0d));
	double A = UT * 1.002737909d;
	double GST = T0 + A;
	GST = GST - (24.0d * long(GST / 24.0d));
	return GST;
}


// **********************************************************************************************************************
double GSTtoLST(double GST, LatLng SiteLng) {
    double offset = DDtoDH(DMStoDD(SiteLng.getDeg(), SiteLng.getMin(), SiteLng.getSec(), SiteLng.getCardinal()));
    double LST = GST + offset;
    // LST = LST - (24.0d * long(LST/24.0d));
    if (LST < 0) {
        LST = LST + 24;
    } else if (LST >= 24) {
        LST = LST -24;   
    }
    return LST;
}


// **********************************************************************************************************************
// Decimal Degrees to Decimal Hours conversion
double DDtoDH(double DD) {	// DD Angle expressed in Decimal Degrees
	double dh = (DD / 15.0d);
	return dh;
}


// **********************************************************************************************************************
// DEGREES MINUTES AND SECONDS TO DECIMAL DEGREES
double DMStoDD(int deg, int min, double sec, boolean cardinal) {
	double s = sec / 60.0d;
    double m = (s + min) / 60.0d;
    double d = deg + m;
    if (cardinal == true) {
        return d;
    } else {
        return -d;
    }
}
















// -----------------------------------------------------------------------------------------------
// JOYSTIC CONTROL
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
        
        // if ((stepperAz.distanceToGo() !=0) || (stepperAlt.distanceToGo() !=0)) {
            // stepperAz.runSpeedToPosition();
            // stepperAlt.runSpeedToPosition();
        // }
    }
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















/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
// void EQtoAZ(Equatorial &Eq, Horizontal &Ho) {
//     long Ra = Eq.getRA();
//     long Dec = Eq.getDEC();
//     double Ha = LST - Ra;       // Hour Angular
//     if (Ha < 0) {
//         Ha = Ha + 86400;
//     }
//     double HaRAD = ((Ha / 3600.d) * 15.d) * rad;

//     double DecRAD = (Dec / 3600.0d) * rad;

//     double altSinRAD = sin(DecRAD) * sinLatRAD + cos(DecRAD) * cosLatRAD * cos(HaRAD);
//     double altRAD = asin(altSinRAD);
//     Ho.setAlt(long((altRAD / rad) * 3600.0d));

//     double y = -cos(DecRAD) * cosLatRAD * sin(HaRAD);
//     double x = sin(DecRAD) - sinLatRAD * altSinRAD;
//     double AzRAD = atan2(y, x);
//     double AzDEG = (AzRAD / rad);

//     AzDEG = AzDEG - (360.0d * int(AzDEG / 360.0d));

//     Ho.setAz(long(AzDEG * 3600.d));
// }
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/
/*************************************************************************************************/


























































void RotateRelative() {
    //We move X steps from the current position of the stepper motor in a given direction.
    //The direction is determined by the multiplier (+1 or -1)
   
    runallowed = true; //allow running - this allows entering the RunTheMotor() function.
    stepperAz.setMaxSpeed(receivedSpeed); //set speed
    stepperAz.move(direction * receivedSteps); //set relative distance and direction
}



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

