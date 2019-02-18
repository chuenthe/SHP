#define DEBUG //just comment this out if the code does not require any more testing. All serial reporting will then be excluded from compiling.

#ifdef DEBUG
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT(x)  Serial.print(x)
#define DEBUG_BEGIN(x)	Serial.begin(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#define DEBUG_BEGIN(x)
#endif

//Pins
const int VinputPin = A1;
const int VpressurePin = A5;
const int pwmPIN = 5;
const int buttonPin = 4;
const int hallPin = 8;

//definition of colour LED Pins
#define RED 10
#define BLUE 12
#define GREEN 11
#define LED_OFF 13 //not connected, so LED off

const int timeFactor = 8; //This is a factor to apply to all time intervals if the internal clock is sped up


//Pressure Sensor
int psensor = 500;  //250 for a sensor of 2.5MPa (500 for 5Mpa)
int Pmax = 100; // 1800Kpa / 180 meters head
int PmaxDefault = 300; //Max Pressure that is set if there is no saved pressure setting
int Vpressure = 0;
int Pressure = 0;

long idle = 80000; //10sec Idle time
unsigned long idlepressure = 160000; //20sec Idle time for a high pressure event
unsigned long VocSampleRate = 2400000; //5min Sample time between Vmpp adjustments
unsigned long VocCounter = 0;
unsigned long counter = 0;
unsigned long idlecounter = 0;

//Voltage
float k = 0.8; // Vmpp = Voc x k
const unsigned int startvolts = 58000; // Voc needed to attempt pump start
unsigned int maintainvolts = 55000; // below this voltage pump cannot run so will stop, idle and reset
long maxpowvolts = 58000; // Default Vmpp
long Voc = 0;
int vdiv = 0;
unsigned long mvolts = 0;
long Vdiv5V = 119000; // Voltage divider output at 5V
int VdivMapAdj = 536;
float dropoutFactor = 0.77; // percentage of MPPT at which state goes into reset when in running state
unsigned long volt_diff;

//PWM stuff
int upPWMdelay = 1000; // change to alter acceleration speed
int downPWMdelay = 100; // change to alter deceleration speed
int pwmDelay; // speed loaded with either upPWMdelay or downPWMdelay
int pwm = 0; // 92 = approx 1.42v to throttle
int pwmUpStep = 1;
int pwmDownStep = 1;
boolean inStartup = true;
const int pwmMin = 80;
const int pwmMax = 200;
long pwmTimeStamp = 0;
boolean ignitionSwitch = true;

int upPWMdelaySU = 100 * timeFactor; //determines acceleration speed during startup
int downPWMdelaySU = 100 * timeFactor; //determines deceleration speed during startup
int pwmUpStepSU = 1; //pwm increase step during startup
int pwmDownStepSU = 1; //pwm decrease step during startup
int pwmUpperLimSU = 85; //pwm limit for startup -> run status

int upPWMdelayRun = 3 * timeFactor; //determines acceleration speed during normal running
int downPWMdelayRun = 3 * timeFactor; //determines deceleration speed during normal running
int pwmUpStepRun = 1; //pwm increase step during normal running
int pwmDownStepRun = 1; //pwm decrease step during normal running

//Hall Sensor
boolean hallState = LOW;
boolean hallLastState = LOW;
int magCounter = 0;
const int numMagnets = 28;
float RPM;


//LED Stuff
int ledState = HIGH;
int ledColour = GREEN;
boolean holdStatus = 0; //Does the status need to be held on the LED? 
long holdTime = 2000 * timeFactor; //How long does the status need to be held on the LED
long lastStatusTime = 0; //Time stamp for holding
boolean blink;
const int blinkShort = 200 * timeFactor; //short blink time
const int blinkLong = 800 * timeFactor; //long blink time
int blinkInterval; //Measures time between LED blinks
long lastBlinkTime = 0; //Time stamp for blinking

						//Button Stuff
int buttonRead; //current button read
int buttonState; //confirmed button state after debounce
int lastButtonState = LOW; //compared with buttonread to detect something is happening
long lastDebounceTime = 0; //Time stamp for debouncing
long debounceDelay = 50 * timeFactor; //Debounce time

									  //Serial Printout
long lastReportTime = 0;
long reportInterval = 1000 * timeFactor;

//Debugging
int dbMaxVolts = 0; //To capture maximum voltage
int dbMinVolts; //To capture minimum voltage measurement
//const int dbArraySize = 1;
//unsigned int dbArray [3] [dbArraySize];
//int dbArrayCounter = 0; //dbArrayCounter points to the OLDEST element in dbArray

// States
#define S_Reset 0
#define S_Check 1
#define S_IdlePressure 2
#define S_Run 3
#define S_Idle 4
#define S_VocCheck 5
#define S_Pressure_Set 6 //New State needs to be defined
#define S_ERROR 7
#define S_OFF 8

//definition of LED States
#define D_RUNNING 0
#define D_IDLE 1
#define D_SET_PRESSURE 2
#define D_IDLE_PRESSURE 3
#define D_ERROR_1 4
#define D_ERROR_EEPROM 5
#define D_OFF 6


int state = S_Reset; // Initial state
int LEDstate = D_RUNNING;
int oldLEDState = LEDstate; //to detect change in state

unsigned long loopInTime = 0;
unsigned long loopTime = 0;

//EEPROM stuff
#include <EEPROM.h>
int Add1 = 0;
int Add2 = 0;
int Add3 = 0;

void setup() {

	DEBUG_BEGIN(9600);

	pinMode(VinputPin, INPUT);
	pinMode(VpressurePin, INPUT);
	pinMode(buttonPin, INPUT);
	pinMode(hallPin, INPUT);
	pinMode(RED, OUTPUT);
	pinMode(GREEN, OUTPUT);
	pinMode(BLUE, OUTPUT);

	TCCR0B = TCCR0B & 0b11111000 | 0x02; // speeds up clock by 8 - remember that 8000 millis = 1 second

	EEread();
	F_VocCheck();
}

void loop() {
	//loopInTime = micros(); // for debugging
	//ReadVolts();
	//readHall(); //only activate if hall sensor is active
	setState();
	readButton(); //read the pushbutton
	showLED(); //Displays the current status on the LED
#ifdef DEBUG
	DebugValues();
	report();
#endif //DEBUG 
	//loopTime = (micros() - loopInTime) / timeFactor; //for debugging
}




void setState() {
	switch (state) {
	case S_Reset:
		LEDstate = D_IDLE;
		F_Reset();
		break;
	case S_Check:
		F_Check();
		break;
	case S_IdlePressure:
		LEDstate = D_IDLE_PRESSURE;
		F_IdlePressure();
		break;
	case S_Run:
		LEDstate = D_RUNNING;
		F_Run();
		break;
	case S_Idle:
		LEDstate = D_IDLE;
		F_Idle();
		break;
	case S_VocCheck:
		F_VocCheck();
		break;
	case S_ERROR:
		LEDstate = D_ERROR_EEPROM;
		break;
	case S_OFF:
		LEDstate = D_OFF;
		break;
	}
}

void readButton() { //this routine checks and debounces a digital push button and hands it to void actionbutton()
	buttonRead = digitalRead(buttonPin);
	if (buttonRead != lastButtonState) {
		lastDebounceTime = millis();
	}
	if ((millis() - lastDebounceTime) > debounceDelay) {
		if (buttonRead != buttonState) {
			buttonState = buttonRead;
			if (buttonState == HIGH) {
				actionButton(); //this is where the action happens
			}
		}
	}
	lastButtonState = buttonRead;
}

void actionButton() { //this gets called if a digital button is pushed
	if (mvolts < 1000) {
		Pmax = PmaxDefault;
		EEwrite();
	}
	else {
		Pmax = Pressure + 20;
		EEwrite();
	}
	LEDstate = D_SET_PRESSURE;
}

void ReadVolts() {
	vdiv = analogRead(VinputPin);
	mvolts = map(vdiv, 0, 1023, VdivMapAdj, Vdiv5V);
}

void ReadPressure() {
	Vpressure = analogRead(VpressurePin);
	Pressure = map(Vpressure - 102, 0, 820, 0, psensor);
}

void F_Reset() {
	pwm = 70;
	ReadVolts();
	inStartup = true;
	if (ignitionSwitch == true) {
		if (mvolts > 1000) {
			ignitionSwitch = false;
			delay(800);
		}
	}
	else {
		if (mvolts >= startvolts) {
			state = S_Check;

		}
		else {
			state = S_Idle;
			idlecounter = millis() + idle;
		}
	}
}

void F_Check() {
	ReadVolts();
	ReadPressure();

	if (inStartup == true) {
		maintainvolts = maxpowvolts;
		upPWMdelay = upPWMdelaySU;
		downPWMdelay = downPWMdelaySU;
		pwmUpStep = pwmUpStepSU;
		pwmDownStep = pwmDownStepSU;
		if (pwm > pwmUpperLimSU) inStartup = false;
	}
	else {
		maintainvolts = maxpowvolts * dropoutFactor;
		upPWMdelay = upPWMdelayRun;
		downPWMdelay = downPWMdelayRun;
		volt_diff = (maxpowvolts - mvolts) / 1000;
		pwmUpStep = pwmUpStepRun;
		pwmDownStep = pwmDownStepRun;
	}
	if (mvolts < 1000) { //ignition circuit broken by on/off switch or float sensor
		analogWrite(pwmPIN, 0);
		state = S_OFF;
		ignitionSwitch = true;
	}
	else if (millis() >= VocCounter) {
		state = S_VocCheck;
	}
	else if (Pressure > Pmax) {
		counter = millis() + idlepressure;
		state = S_IdlePressure;
	}
	else if (mvolts >= maintainvolts) {
		state = S_Run;
	}
	else {
		state = S_Idle;
		idlecounter = millis() + idle;
		pwm = 70;
	}
}

void F_IdlePressure() {
	if (millis() < counter) {
		analogWrite(pwmPIN, 0);
		ReadVolts();
		if (mvolts <= 1000) {
			state = S_OFF;
		}
		else {
			state = S_IdlePressure;
		}
	}
	else {
		state = S_Reset;
	}
}

void F_Run() {
	if ((millis() - pwmTimeStamp) > pwmDelay) {
		if (mvolts > maxpowvolts) {
			pwmDelay = upPWMdelay;
			pwm = pwm + pwmUpStep;
			if (pwm > pwmMax) {
				pwm = pwmMax;
			}
		}
		else {
			pwmDelay = downPWMdelay;
			pwm = pwm - pwmDownStep;
			if (pwm < pwmMin) {
				pwm = pwmMin;
			}
		}
		analogWrite(pwmPIN, pwm);
		pwmTimeStamp = millis();
	}
	state = S_Check;
}

void F_Idle() {
	if (millis() < idlecounter) {
		analogWrite(pwmPIN, 0);
	}
	else {
		state = S_Reset;
	}
}

void F_VocCheck() {
	analogWrite(pwmPIN, 0);
	delay(2000); // 0.5 seconds
	ReadVolts();
	DEBUG_PRINT("mvolts= ");
	DEBUG_PRINT(mvolts);
	DEBUG_PRINT(", BFmaxPvolts= ");
	DEBUG_PRINT(maxpowvolts);
	maxpowvolts = mvolts * k;
	VocCounter = millis() + VocSampleRate;
	DEBUG_PRINT(", AFTmaxPvolts= ");
	DEBUG_PRINTLN(maxpowvolts);
	state = S_Reset;
}

void setLED() {
	if (holdStatus == false) {
		switch (LEDstate) {
		case D_RUNNING:
			ledColour = GREEN;
			blink = false;
			holdStatus = false;
			break;
		case D_IDLE:
			ledColour = GREEN;
			blink = true;
			blinkInterval = blinkLong;
			holdStatus = false;
			break;
		case D_IDLE_PRESSURE:
			ledColour = BLUE;
			blink = true;
			blinkInterval = blinkShort;
			holdStatus = false;
			break;
		case D_SET_PRESSURE:
			ledColour = BLUE;
			blink = false;
			holdStatus = true; //holds this state for a while before allowing switch.
			break;
		case D_ERROR_1:
			ledColour = RED;
			blink = false;
			holdStatus = false;
			break;
		case D_ERROR_EEPROM:
			ledColour = RED;
			blink = true;
			blinkInterval = blinkShort;
			holdStatus = false;
			break;
		case D_OFF:
			ledColour = LED_OFF;
			blink = false;
			holdStatus = false;
			break;
		default:
			break;
		}
		oldLEDState = LEDstate;
		lastBlinkTime = millis();
		lastStatusTime = millis();
	}
	else {
		if ((millis() - lastStatusTime) > holdTime) holdStatus = false;
	}
}

void showLED() {
	if (LEDstate != oldLEDState) setLED();
	digitalWrite(RED, LOW);
	digitalWrite(BLUE, LOW);
	digitalWrite(GREEN, LOW);
	if (blink == true) {
		if ((millis() - lastBlinkTime) > blinkInterval) {
			ledState = !ledState;
			lastBlinkTime = millis();
		}
	}
	else {
		ledState = HIGH;
	}
	digitalWrite(ledColour, ledState);
}

void EEread() {
	Add1 = EEPROM.read(7);
	Add2 = EEPROM.read(15);
	Add3 = EEPROM.read(39);
	DEBUG_PRINT("EEPROM Values     ");
	DEBUG_PRINT(Add1);
	DEBUG_PRINT("   ");
	DEBUG_PRINT(Add2);
	DEBUG_PRINT("   ");
	DEBUG_PRINTLN(Add3);
	if (Add1 == Add2) {
		Pmax = Add1;
	}
	else if (Add1 == Add3) {
		Pmax = Add1;
	}
	else if (Add2 == Add3) {
		Pmax = Add2;
	}
	else {
		Pmax = PmaxDefault;
		state = S_ERROR;
	}
	DEBUG_PRINT("Pmax from EEPROM = ");
	DEBUG_PRINTLN(Pmax);
}

void EEwrite() {
	EEPROM.write(7, Pmax);
	EEPROM.write(15, Pmax);
	EEPROM.write(39, Pmax);
	DEBUG_PRINTLN("EEPROM WRITE");
	DEBUG_PRINT("Current Pressure = ");
	DEBUG_PRINT(Pressure);
	DEBUG_PRINT("   Pressure Sensor Set = ");
	DEBUG_PRINTLN(Pmax);
}

void DebugValues() {
	if (mvolts > dbMaxVolts) dbMaxVolts = mvolts;
	if (mvolts < dbMinVolts) dbMinVolts = mvolts;
	/*dbArray[0][dbArrayCounter] = millis();
	dbArray[1][dbArrayCounter] = mvolts;
	dbArray[2][dbArrayCounter] = pwm;
	dbArrayCounter = (dbArrayCounter + 1) % dbArraySize; //dbArrayCounter points to the OLDEST element in dbArray
	*/
}

void report() {
	int timeSinceLastReport = millis() - lastReportTime;
	if (timeSinceLastReport > reportInterval) {
		RPM = magCounter / 2.0 / timeSinceLastReport * 1000 * timeFactor / numMagnets;
		lastReportTime = millis();
		magCounter = 0;
		Printout();
		dbMaxVolts = 0; //resets the maximum voltage tracker
		dbMinVolts = 100000; //resets the minimum voltage tracker
	}

}

void Printout() {

	//		DEBUG_PRINT(" loop: ");
	//		DEBUG_PRINT(loopTime);
	DEBUG_PRINT(" SU: ");
	DEBUG_PRINT(inStartup);
	DEBUG_PRINT(" State: ");
	DEBUG_PRINT(state);
	DEBUG_PRINT(" ,LED: ");
	DEBUG_PRINT(LEDstate);
	DEBUG_PRINT(" ,Curr Press= ");
	DEBUG_PRINT(Pressure);
	DEBUG_PRINT(" ,Pmax=  ");
	DEBUG_PRINT(Pmax);
	DEBUG_PRINT(" ,pwm=  ");
	DEBUG_PRINT(pwm);
	DEBUG_PRINT(", mvolts= ");
	DEBUG_PRINT(mvolts);
	DEBUG_PRINT(", maxPvolts= ");
	DEBUG_PRINTLN(maxpowvolts);
	//DBPrintArray();

}


/*
void DBPrintArray() {
	int dbAIndex = dbArrayCounter;
	for (int i = 0; i < dbArraySize; i++) {
		DEBUG_PRINT(dbArray[0][dbAIndex]);
		DEBUG_PRINT(", ");
		DEBUG_PRINT(dbArray[0][dbAIndex]);
		DEBUG_PRINT(", ");
		DEBUG_PRINTLN(dbArray[0][dbAIndex]);
		dbAIndex = (dbAIndex +1 ) % dbArraySize;
	}
}
*/

void readHall() {
	hallState = digitalRead(hallPin);
	if (hallState != hallLastState) {
		magCounter++;
		hallLastState = hallState;
	}
}