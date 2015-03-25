
////////////////////
//Pin Definitions //
////////////////////
#define GPO_1			3
#define PUSH_BUT	10
#define LED_1			13

///////////////
// Constants //
///////////////
#define V_REF			2.500 // Nominal voltage of high-accuracy reference (±0.08%)
#define ADC_MAX		4095.0
#define PWM_RES		11
#define PWM_MAX		2047.0
#define PWM_FREQ	23437

#define PWM_A_PIN 5
#define PWM_B_PIN 3
#define PWM_C_PIN 25

#define COUNT_TIME	10000
#define CUT_TIME		10000
#define ARMING_TIME	2000

/////////////
// Classes //
/////////////
class PushButton {
private:
	unsigned int pin;
	bool stateFlip; // True if button true is electrical low
	unsigned int debounceTime;

	elapsedMicros debounceTimer;
	bool lastStateRead;
	bool stateRead;
	bool lastState;
	bool debouncing;

public:
	bool state;
	bool rose, fell, changed; // Reset upon next read
	

	void init(unsigned int pin, bool stateFlip, unsigned int debounceTime) {
		this->pin = pin;
		this->stateFlip = stateFlip;
		this->debounceTime = debounceTime;

		pinMode(pin, INPUT);
		lastState = (stateFlip) ? !digitalRead(pin) : digitalRead(pin);
		state = lastState;
		debouncing = false;
		debounceTimer = 0;
		rose = false;
		fell = false;
		changed = false;
	}

	bool read() {
		changed = false;
		rose = false;
		fell = false;

		stateRead = digitalRead(pin) ^ stateFlip;
		if (stateRead != lastStateRead) {
			debouncing = true;
			lastStateRead = stateRead;
			debounceTimer = 0;
		}
		else if (debouncing && debounceTimer >= debounceTime) {
			debouncing = false;
			if (stateRead != state) {
				state = stateRead;
				changed = true;
				if (state) rose = true;
				else fell = true;
			}
		}

		return state;
	}
};

class GPOinterface {
private:
	unsigned int pin;
	unsigned int maxVoltage;

public:
	void init(unsigned int pin, float maxVoltage) {
		this->pin = pin;
		this->maxVoltage = constrain(maxVoltage, 0.0, 5.0);

		pinMode(pin, OUTPUT);
	}

	void on(float power, float batVolt) {
		unsigned int maxPWM = PWM_MAX*(maxVoltage/batVolt) + 0.5;

		if (power >= 1.00)
			analogWrite(pin, maxPWM);
		else if (power <= 0.00)
			analogWrite(pin, 0);
		else
			analogWrite(pin, int(maxPWM*power + 0.5));
	}

	void off() {
		analogWrite(pin, 0);
	}
};

//////////////////////
// Global Variables //
//////////////////////
GPOinterface paraNichrome;
PushButton button;					// Multipurpose user interface button

enum State {MONITORING, ARMING, COUNTING_DOWN, CUTTING};

State currentState;
elapsedMillis timer;

////////////////////
// Main Functions //
////////////////////
///
void setup() {
	analogWriteFrequency(PWM_A_PIN, PWM_FREQ); // Ideal frequency for 12-bit pwm
	analogWriteFrequency(PWM_B_PIN, PWM_FREQ);
	analogWriteFrequency(PWM_C_PIN, PWM_FREQ);
	analogWriteResolution(PWM_RES);

	button.init(PUSH_BUT, false, 10000);
	paraNichrome.init(GPO_1, 1.4);

	pinMode(LED_1, OUTPUT);

	currentState = MONITORING;

	Serial.begin(115200);
}

void loop() {
	
	button.read();

	switch(currentState) {
		default:
		case MONITORING:
			if (button.rose) {
				digitalWriteFast(LED_1, HIGH);
				currentState = ARMING;
				timer = 0;
				Serial.print("Arming\n");
			}
			break;
		case ARMING:
			if (button.fell) {
				digitalWriteFast(LED_1, LOW);
				currentState = MONITORING;
				Serial.print("Abort\n\n");
			}
			else if (timer >= ARMING_TIME) {
				digitalWriteFast(LED_1, LOW);
				currentState = COUNTING_DOWN;
				timer = 0;
				Serial.print("Count Down\n");
			}
			break;
		case COUNTING_DOWN:
			if (button.rose) {
				// Abort countdown
				currentState = MONITORING;
				digitalWriteFast(LED_1, LOW);
				Serial.print("Abort\n\n");
			}
			else {
				if (timer >= COUNT_TIME) {
					digitalWriteFast(LED_1, LOW);
					currentState = CUTTING;
					paraNichrome.on(1.0, 4.2);
					timer = 0;
					Serial.print("Cutting\n");
				}
				else {
					if (timer % 1000 < 500) digitalWriteFast(LED_1, LOW);
					else digitalWriteFast(LED_1, HIGH);
				}
			}
			break;
		case CUTTING:
			if (button.rose) {
				// Abort cut operation
				currentState = MONITORING;
				digitalWriteFast(LED_1, LOW);
				paraNichrome.off();
				Serial.print("Abort\n\n");
			}
			else {
				if (timer >= CUT_TIME) {
					digitalWriteFast(LED_1, LOW);
					currentState = MONITORING;
					paraNichrome.off();
					Serial.print("Timeout\n");
				}
				else {
					if (timer % 200 < 100) digitalWriteFast(LED_1, LOW);
					else digitalWriteFast(LED_1, HIGH);
				}
			}
			break;
	}


}