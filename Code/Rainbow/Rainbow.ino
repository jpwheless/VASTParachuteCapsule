#define RGB_R			3
#define RGB_B			4
#define RGB_G			5
#define PUSH_BUT	10

#define V_REF			2.500 // Nominal voltage of high-accuracy reference (±0.08%)
#define ADC_MAX		4095.0
#define PWM_RES		11
#define PWM_MAX		2047.0
#define PWM_FREQ	23437

#define PWM_A_PIN 5
#define PWM_B_PIN 3
#define PWM_C_PIN 25

#define INC_TIME	4000
#define RED				0
#define GREEN			1
#define ANTIRED		2
#define BLUE			3
#define ANTIGREEN	4
#define ANTIBLUE	5

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

		stateRead = (stateFlip) ? !digitalRead(pin) : digitalRead(pin);
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

class RGBinterface {
private:
	unsigned int pinRed;
	unsigned int pinGreen;
	unsigned int pinBlue;
	bool sinkDrive;

public:
	void init(unsigned int pinRed, unsigned int pinGreen, unsigned int pinBlue, bool sinkDrive) {
		this->pinRed = pinRed;
		this->pinGreen = pinGreen;
		this->pinBlue = pinBlue;
		this->sinkDrive = sinkDrive;

		pinMode(pinRed, OUTPUT);
		pinMode(pinGreen, OUTPUT);
		pinMode(pinBlue, OUTPUT);

		color(0,0,0);
	}

	// Maps 0-255 inputs to exponential pwm outputs
	void color(int r, int g, int b) {
		if (sinkDrive) {
			if (r >= 255) analogWrite(pinRed, 0);
			else if (r <= 0) analogWrite(pinRed, PWM_MAX);
			else if (r <= 4) analogWrite(pinRed, PWM_MAX-1);
			else {
				analogWrite(pinRed, PWM_MAX - int((PWM_MAX/65025.0)*pow(r, 2.0) + 0.5));
			}

			if (g >= 255) analogWrite(pinGreen, 0);
			else if (g <= 0) analogWrite(pinGreen, PWM_MAX);
			else if (g <= 4) analogWrite(pinGreen, PWM_MAX-1);
			else {
				analogWrite(pinGreen, PWM_MAX - int((PWM_MAX/65025.0)*pow(g, 2.0) + 0.5));
			}

			if (b >= 255) analogWrite(pinBlue, 0);
			else if (b <= 0) analogWrite(pinBlue, PWM_MAX);
			else if (b <= 4) analogWrite(pinBlue, PWM_MAX-1);
			else {
				analogWrite(pinBlue, PWM_MAX - int((PWM_MAX/65025.0)*pow(b, 2.0) + 0.5));
			}
		}
		else {
			if (r >= 255) analogWrite(pinRed, PWM_MAX);
			else if (r <= 0) analogWrite(pinRed, 0);
			else if (r <= 4) analogWrite(pinRed, 1);
			else {
				analogWrite(pinRed, int((PWM_MAX/65025.0)*pow(r, 2.0) + 0.5));
			}

			if (g >= 255) analogWrite(pinGreen, PWM_MAX);
			else if (g <= 0) analogWrite(pinGreen, 0);
			else if (g <= 4) analogWrite(pinGreen, 1);
			else {
				analogWrite(pinGreen, int((PWM_MAX/65025.0)*pow(g, 2.0) + 0.5));
			}

			if (b >= 255) analogWrite(pinBlue, PWM_MAX);
			else if (b <= 0) analogWrite(pinBlue, 0);
			else if (b <= 4) analogWrite(pinBlue, 1);
			else {
				analogWrite(pinBlue, int((PWM_MAX/65025.0)*pow(b, 2.0) + 0.5));
			}
		}
	}
};

RGBinterface rgb;
PushButton button;

int currentColor;
elapsedMicros timer;
unsigned int i;
bool toggle;

void setup() {
	analogWriteFrequency(PWM_A_PIN, PWM_FREQ); // Ideal frequency for 12-bit pwm
	analogWriteFrequency(PWM_B_PIN, PWM_FREQ);
	analogWriteFrequency(PWM_C_PIN, PWM_FREQ);
	analogWriteResolution(PWM_RES);

	button.init(PUSH_BUT, true, 8000);
	rgb.init(RGB_R, RGB_G, RGB_B, true);

	currentColor = RED;
	toggle = false;
	i = 0;

	Serial.begin(115200);
}

void loop() {

	button.read();

	if (button.rose) {
		if (toggle) {
			toggle = false;
			rgb.color(0, 0, 0);
		}
		else {
			toggle = true;
			currentColor = RED;
			timer = 0;
			i = 0;
		}
	}

	if (toggle) {
		if (timer >= INC_TIME) {
			i++;
			if (i > 255) {
				i = 0;
				currentColor++;
				if (currentColor > 5) currentColor = 0;
			}
			timer = 0;
		}
		switch (currentColor) {
			case 0:
				rgb.color(i, 0, 0);
				break;
			case 1:
				rgb.color(255, i, 0);
				break;
			case 2:
				rgb.color((255-i), 255, 0);
				break;
			case 3:
				rgb.color(0, 255, i);
				break;
			case 4:
				rgb.color(0, (255-i), 255);
				break;
			case 5:
				rgb.color(0, 0, (255-i));
				break;
		}
	}
	
}

void debug() {
	Serial.print(currentColor);
	Serial.print("\t");
	Serial.print(i);
	Serial.print("\t");
}