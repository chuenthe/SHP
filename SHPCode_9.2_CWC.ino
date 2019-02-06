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

const int timeFactor = 8; //This is a factor to apply to all time intervals if the internal clock is sped up

int Pmax = 100; // 1800Kpa / 180 meters head
float k = 0.8; // Vmpp = Voc x k
int upspeed = 1000; // change to alter acceleration speed
int downspeed = 100; // change to alter deceleration speed
int psensor = 500;  //250 for a sensor of 2.5MPa (500 for 5Mpa)

long idle = 80000; //10sec Idle time
unsigned long idlepressure = 160000; //20sec Idle time for a high pressure event
unsigned long VocSampleRate = 2400000; //5min Sample time between Vmpp adjustments
unsigned long VocCounter = 0;
unsigned long counter = 0;
unsigned long idlecounter = 0;

const unsigned int startvolts = 65000; // Voc needed to attempt pump start
const unsigned int maintainvolts = 40000; // below this voltage pump cannot run so will stop, idle and reset
long maxpowvolts = 58000; // Default Vmpp
long Voc = 0;
int vdiv = 0;
unsigned long mvolts = 0;
int Vpressure = 0;
int Pressure = 0;
long Vdiv5V = 119000; // Voltage divider output at 5V
int VdivMapAdj = 536;

int pwm = 0; // 92 = approx 1.42v to throttle
const int pwmMin = 80;
const int pwmMax = 255;

//Hall Sensor
boolean hallState

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
long lastPrintoutTime = 0;
long PrintoutInterval = 500 * timeFactor;

// States
#define S_Reset 0
#define S_Check 1
#define S_IdlePressure 2
#define S_Run 3
#define S_Idle 4
#define S_VocCheck 5
#define S_Pressure_Set 6 //New State needs to be defined

//definition of LED States
#define D_RUNNING 0
#define D_IDLE 1
#define D_SET_PRESSURE 2
#define D_IDLE_PRESSURE 3
#define D_ERROR_1 4
#define D_ERROR_2 5


int state = S_Reset; // Initial state
int LEDstate = D_RUNNING;
int oldLEDState = LEDstate; //to detect change in state

#include <EEPROM.h>

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

	if (EEPROM.read(7) != 0) {
		Pmax = EEPROM.read(7);
		DEBUG_PRINT("Pmax from EEPROM = ");
		DEBUG_PRINTLN(Pmax);
	}
}

void loop() {
	ReadVolts();
	readHall();
	setState();
	readButton(); //read the pushbutton
	if (LEDstate != oldLEDState) setLED();
	showLED(); //Displays the current status on the LED
	Printout();
}

void readHall() {

}

void setState() {
	switch (state) {
	case S_Reset:
		//DEBUG_PRINTLN("F_Reset");
		F_Reset();
		break;
	case S_Check:
		//DEBUG_PRINTLN("F_Check");
		F_Check();
		break;
	case S_IdlePressure:
		//DEBUG_PRINTLN("F_IdlePressure");
		LEDstate = D_IDLE_PRESSURE;
		F_IdlePressure();
		break;
	case S_Run:
		//DEBUG_PRINTLN("F_Run");
		LEDstate = D_RUNNING;
		F_Run();
		break;
	case S_Idle:
		//DEBUG_PRINTLN("F_Idle");
		LEDstate = D_IDLE;
		F_Idle();
		break;
	case S_VocCheck:
		//DEBUG_PRINTLN("F_VocCheck");
		F_VocCheck();
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
	Pmax = Pressure + 20;
	EEPROM.write(7, Pmax);
	DEBUG_PRINT("Current Pressure = ");
	DEBUG_PRINT(Pressure);
	DEBUG_PRINT("   Pressure Sensor Set = ");
	DEBUG_PRINTLN(Pmax);
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
	if (mvolts >= startvolts) {
		state = S_Check;
	}
	else {
		state = S_Idle;
		idlecounter = millis() + idle;
	}
}

void F_Check() {
	ReadVolts();
	ReadPressure();
	if (mvolts < 1000) { //ignition circuit broken by on/off switch or float sensor
		analogWrite(pwmPIN, 0);
		state = S_Reset;
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
	}
}

void F_IdlePressure() {
	if (millis() < counter) {
		analogWrite(pwmPIN, 0);
		ReadVolts();
		if (mvolts <= 1000) {
			state = S_Reset;
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
	if (mvolts > maxpowvolts) {
		//counter = millis() + upspeed;
		pwm = pwm + 1;
		if (pwm > pwmMax) {
			pwm = pwmMax;
		}
		analogWrite(pwmPIN, pwm);
		delay(2000);
	}
	else {
		//counter = millis() + downspeed;
		pwm = pwm - 1;
		if (pwm < pwmMin) {
			pwm = pwmMin;
		}
		analogWrite(pwmPIN, pwm);
		delay(20);
	}
	//while (millis() < counter) {
	// analogWrite(pwmPIN, pwm);
	//}
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
		case D_ERROR_2:
			ledColour = RED;
			blink = true;
			blinkInterval = blinkShort;
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

void Printout() {
	if ((millis() - lastPrintoutTime) > PrintoutInterval) {
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
		lastPrintoutTime = millis();
	}
}
