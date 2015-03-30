#include <SdFat.h>
#include <Wire.h>
#include <EEPROM.h>
//#include <MPU9150_9Axis_MotionApps41.h>
//#include <helper_3dmath.h>
#include "gpsConfig.h"

// TODO:
// 	Write GPS code
// 	Write IMU Code
// 	Fix barometer math
// 	Write logic code

////////////////////
//Pin Definitions //
////////////////////
#define SD_CS			2
#define RGB_R			3
#define RGB_B			4
#define RGB_G			5
#define GPS_EXINT	6
#define GPS_TX		7
#define GPS_RX		8
#define LED_2			9
#define PUSH_BUT	10
#define SD_MOSI		11
#define SD_MISO		12
#define SD_SCK		13
#define THERM_2		14
#define THERM_1		15
#define BAT_VSNS	16
#define IMU_INT		17
#define I2C_SDA		18
#define I2C_SLCK	19
#define GPO_2			20
#define GPO_3			21
#define GPO_4			22
#define GPO_1			23

///////////////
// Constants //
///////////////
#define V_REF			2.500 // Nominal voltage of high-accuracy reference (±0.08%)
#define ADC_MAX		4095.0
#define ADC_RES		12
#define PWM_RES		11
#define PWM_MAX		2047.0
#define PWM_FREQ	23437

#define BAT_VSNS_MULT	2.005

#define PWM_A_PIN 5
#define PWM_B_PIN 3
#define PWM_C_PIN 25

#define MS5607_ADDR		0x77 // 6 MSBs of address. B111011x is complete address, with x being ~CSB
#define MPU9250_ADDR	0x69 // 6 MSBs of address. B111011x is complete address, with x being CSB

#define CSV_BUFFER_SIZE	16
#define LOG_HEADER			"Time,Lat,Long,Alt,RoC,GndSpd,GndCrs,Acc,nSats,ExtTemp,BatTemp,IntTemp,Pres,BatVolt,BatCap,BatHeat\n"
#define LOG_PERIOD			1000 // ms
#define CSV_TIME				0
#define CSV_LAT					1
#define CSV_LONG				2
#define CSV_ALT					3
#define CSV_ROC					4
#define CSV_GNDSPD			5
#define CSV_GNDCRS			6
#define CSV_ACC					7
#define CSV_NSATS				8
#define CSV_EXTTEMP			9
#define CSV_BATTEMP			10
#define CSV_INTTEMP			11
#define CSV_PRES				12
#define CSV_BATVOLT			13
#define CSV_BATCAP			14
#define CSV_BATHEAT			15

#define PUBX00_TIME		2		// (float)	hhmmss.ss
#define PUBX00_LAT		3		// (float)	ddmm.mmmmm
#define PUBX00_NS			4		// (char)		North or South latitude
#define PUBX00_LONG		5		// (float)	dddmm.mmmmm
#define PUBX00_EW			6		// (char)		East or West longitude
#define PUBX00_ALT		7		// (float)	altitude ref. datum ellipsoid (m)
#define PUBX00_LOCK		8		// (string)	G3 or D3 or RK = full 3D fix; 
#define PUBX00_HACC		9		// (float)	horizontal acuracy estimate (m)
#define PUBX00_VACC		10	// (float)	vertical acuracy estimate (m)
#define PUBX00_SOG		11	// (float)	speed over ground (km/hr)
#define PUBX00_COG		12	// (float)	course over ground (deg)
#define PUBX00_VVEL		13	// (float)	Vertical velocity, positive downwards (m/s)
#define PUBX00_NSAT		18	// (int)		Number of satellites used in nav solution

#define TIMEZONE			-700	// Relative to UTC (hours*100)

#define CUT_ARM_TIME		60000
#define CUT_EXEC_TIME		60000

#define PARA_ARM_TIME		60000
#define PARA_EXEC_TIME	60000


//////////////////
// I2C Commands //
//////////////////
#define MS5607_RST		0x1E
#define MS5607_PCONV	0x48 // 4096 OSR (9.04ms)
#define MS5607_TCONV	0x58 // 4096 OSR (9.04ms)
#define MS5607_AREAD	0x00
#define MS5607_PREAD	0xA0 // Address of first PROM

/////////////
// Classes //
/////////////

// Front-end for SdFat library, handles logging of data
class SDinterface {
private:
	SdFat sd;
	SdFile file;
	int chipSelect;
	char fileName[11];
	
public:
	String cvsBuffer[CSV_BUFFER_SIZE];
	bool cardOK;

	bool init(unsigned int chipSelect) {
		this->chipSelect = chipSelect;
		// Get old file number
		uint16_t oldFileNumber = 0;
		oldFileNumber |= EEPROM.read(1); // LSB
		oldFileNumber |= EEPROM.read(0) << 8; // MSB

		// Increment. Will overflow eventually (after 65,535 new files), but that should be fine.
		uint16_t fileNumber = oldFileNumber + 1;

		if (!sd.begin(chipSelect, SPI_FULL_SPEED)) return false;

		String s = String(fileNumber) + ".csv";
		s.toCharArray(fileName, 11);

		if(!file.open(fileName, O_CREAT | O_WRITE | O_APPEND | O_TRUNC)) return false;
		file.print(LOG_HEADER);
		file.close();

		EEPROM.write(1, fileNumber & 0xff); // LSB
		EEPROM.write(0, (fileNumber >> 8) & 0xff); // MSB

		cardOK = true;
		return true;
	}


	void buffer(float data, int posCSV, int prec) { // Add values to buffer
		char tempBuffer[20];
		dtostrf(data, 20, prec, tempBuffer);
		cvsBuffer[posCSV] = String(tempBuffer);
		cvsBuffer[posCSV].trim();
	}
	void buffer(unsigned int data, int posCSV) { // Add values to buffer
		cvsBuffer[posCSV] = String(data);
	}
	void buffer(long int data, int posCSV) { // Add values to buffer
		cvsBuffer[posCSV] = String(data);
	}
	void buffer(unsigned long int data, int posCSV) { // Add values to buffer
		cvsBuffer[posCSV] = String(data);
	}

	bool logToSD() { // Write buffer to sd card as a line of CSVs
		if (file.open(fileName, O_CREAT | O_WRITE | O_APPEND)) {
			for (int i = 0; i < CSV_BUFFER_SIZE; i++) {
	    	file.print(cvsBuffer[i]);
	    	if (i < CSV_BUFFER_SIZE - 1) file.print(",");
			}
			file.print("\n");
			cardOK = file.close();
			return cardOK;
		}
	}
};

// LMT84LP analog thermometer interface
class LMT84LPinterface {
private:
	unsigned int sigPin;
	float avgTConst;
	float averageTemp;
	float cor1;
	float cor2;

public:
	float millivolts;
	float celsius;

	// Assign signal pin and the time constant in seconds for readAverage()
	// Set time constant to logging interval
	void init(unsigned int sigPin, float tConst, float cor1, float cor2) {
		this->sigPin = sigPin;
		this->cor1 = cor1;
		this->cor2 = cor2;
		pinMode(sigPin, INPUT);

		if (avgTConst > 0.0 && avgTConst <= 10.0) this->avgTConst = 0.63212*(1.0/tConst);
		else this->avgTConst = 0.63212;

		// Initialize average
		float thisVoltage;
		for (int i = 0; i <5; i++) {
			thisVoltage += cor1*(1000.0*V_REF/ADC_MAX)*float(analogRead(sigPin)) + cor2;
		}
		millivolts = (thisVoltage/5.0);
		celsius = (5.506 - sqrt(30.316 + .00704*(870.6 - millivolts)))/(-.00352) + 30.0;
	}

	// Updates class temperature variable
	// Call many times between logging to SD card for best sampling
	void read(unsigned long microsElapsed) {
		float thisVolt = cor1*(1000.0*V_REF/ADC_MAX)*float(analogRead(sigPin)) + cor2;
		float tc = avgTConst*1.0e-6*float(microsElapsed);

		if (tc < 1.0) // Needs to be less than time constant for calculation to make sense
			millivolts = (1.0-tc)*millivolts + tc*thisVolt;
		else millivolts = thisVolt;

		celsius = (5.506 - sqrt(30.316 + .00704*(870.6 - millivolts)))/(-.00352) + 30.0;
	}

	float convertCtoF(float celsius) {
		return celsius*1.8 + 32.0;
	}
};

class MS5607interface {
private:
	unsigned int readType;

	uint16_t cal[6];

	elapsedMicros timeSinceConv;

	// Return 0 for no measurement, 1 for temperature, 2 for pressure
	unsigned int read() {
		static bool whichMeas = 0; // If 1, pressure.  If 0, temperature.

		// ADC in MS5607 takes about 9.04ms max to complete comversion at 4096 OSR
		if (timeSinceConv > 9040) {
			Wire.beginTransmission(address);
			Wire.write(MS5607_AREAD);
			Wire.endTransmission();
			Wire.requestFrom(address, 3); // 24 bits total

			uint32_t raw = 0;
			raw |= Wire.read() << 16;
			raw |= Wire.read() << 8;
			raw |= Wire.read();

			Wire.beginTransmission(address);
			if (whichMeas) {
				rawPres = raw;
				Wire.write(MS5607_TCONV);
			}
			else {
				rawTemp = raw;
				Wire.write(MS5607_PCONV);
			}
			Wire.endTransmission();
			whichMeas = !whichMeas;
			timeSinceConv = 0;

	    if (!whichMeas) return 2;
	    else return 1;
		}
		else {
			return 0;
		}
	}

public:
	float kPa;
	float celsius;
	bool measComplete;
	int32_t rawTemp;
	int32_t rawPres;
	int32_t intPres;
	int32_t intTemp;
	int address;

	bool init(uint8_t addr) {
		this->address = addr;
		kPa = 0;
		celsius = 0;

		Wire.beginTransmission(address);
	  Wire.write(MS5607_RST);
	 	if (Wire.endTransmission()) return false;
	 	delayMicroseconds(2500); // Need to wait while device resets

		// Read calibration values
		for (unsigned int i = 1; i <= 6; i++){
			cal[i-1] = 0;

			Wire.beginTransmission(address);
	    Wire.write(MS5607_PREAD | (i << 1));
	    if (Wire.endTransmission()) return false;
	    Wire.requestFrom(address, 2); // 6 2-byte calibration values

	    cal[i-1] |= Wire.read() << 8;
	    cal[i-1] |= Wire.read();
	  }

		Wire.beginTransmission(address);
    Wire.write(MS5607_TCONV);
    if (Wire.endTransmission()) return false;
    timeSinceConv = 0;

    measComplete = false;

    return true;
	}

	bool measure() {
		static int64_t dT;
		static int64_t offset;
		static int64_t sensitivity;

		if (measComplete) {
			measComplete = false;
		}

		readType = read();
		switch (readType) {
			default:
			case 0:
				return false;
				break;
			case 1: // Calculate temperature
				dT = rawTemp - (int64_t(cal[4]) << 8);
				intTemp = 2000 + ((dT*int64_t(cal[5])) >> 23);

				offset = (int64_t(cal[1]) << 17) + (int64_t(cal[3])*dT >> 6);
				sensitivity = (int64_t(cal[0]) << 16) + ((int64_t(cal[2])*dT) >> 7);

				//Serial.print(offset);
				//Serial.println(sensitivity);

				// Second-order correction for low temperature
				if (intTemp < 204700) {
					int64_t temp2;
					int64_t off2;
					int64_t sens2;

					temp2 = (dT*dT) >> 31;
					off2 = (61*(int64_t(intTemp)-2000)*(int64_t(intTemp)-2000)) >> 4;
					sens2 = 2*(int64_t(intTemp)-2000)*(int64_t(intTemp)-2000);

					//Serial.print(offset);
					//Serial.println(sensitivity);

					if (intTemp < -1500) {
						off2 += 15*(int64_t(intTemp)+1500)*(int64_t(intTemp)+1500);
						sens2 += 8*(int64_t(intTemp)+1500)*(int64_t(intTemp)+1500);

						//Serial.print(offset);
						//Serial.println(sensitivity);
					}

					intTemp -= temp2;
					offset -= off2;
					sensitivity -= sens2;
				}
				//Serial.println();
				return false;
				break;
			case 2:
				// Calculate pressure using temperature correction				
				intPres = (((rawPres*sensitivity) >> 21) - offset) >> 15;

				celsius = 0.01*intTemp;
				kPa = 0.001*intPres;

				measComplete = true;
				return true;
				break;
		}
	}

	float convertCtoF(float celsius) {
		return celsius*1.8 + 32.0;
	}
};

/*
class MPU9250interface {
private:
	//MPU9150 accelGyroMag;
public:
	int16_t acl[3]; // 0:x, 1:y, 2:z
	int16_t gyr[3];
	int16_t mag[3];



	void init() {
		//accelGyroMag.initialize();
		//accelGyroMag.dmpInitialize
		//accelGyroMag.testConnection()
	}

	void read() {
		//accelGyroMag.getMotion9(&acl[0], &acl[1], &acl[2], &gyr[0], &gyr[1], &gyr[2], &mag[0], &mag[1], &mag[2],);
	}


};
*/

class MAXM8interface {
private:
	String sentence;
	bool receiving;
	elapsedMicros timer;
	
	// Calcs and sends checksum after message.
	// Msg sequence: header, header, class, ID, payload, ... payload, checkA, checkB
	void sendUBX(const uint8_t message[], const int messageLength) {
		uint8_t checkA = 0;
		uint8_t checkB = 0;
		Serial3.write(g::ubx_header[0]);
		Serial3.write(g::ubx_header[1]);
		for(int i = 0; i < messageLength; i++) {
			Serial3.write(message[i]);
			checkA = checkA + message[i];
			checkB = checkB + checkA;
		}
		Serial3.write(checkA);
		Serial3.write(checkB);
		Serial3.println();
		Serial3.flush(); // Wait for buffer to empty
	}

	int getUbxAck(const uint8_t message[]) {
		uint8_t checkA = 0;
		uint8_t checkB = 0;
		uint8_t ackSentence[] = {0,0,0,0,0,0,0,0,0,0};
		elapsedMicros timer = 0;

		// Look for ACK message in incoming serial stream
		int byteCount = 0;
		while (timer < 2000 && byteCount < 10) {
			if (Serial3.available()) {
				uint8_t checkByte = Serial3.read();
				if (byteCount < 3) {
					if (checkByte == g::ubx_ack_header[byteCount]) {
						ackSentence[byteCount] = checkByte;
						byteCount++;
					}
					else {
						byteCount = 0;
					}
				}
				else {
					ackSentence[byteCount] = checkByte;
					byteCount++;
				}
				timer = 0;
			}
		}

		if (byteCount == 0)	{
			return -4; // Ack Error: timeout
		}
		if (byteCount < 10) {
			return -3; // Ack Error: message too short
		}
		for (int i = 2; i < 8; i++) {
			checkA = checkA + ackSentence[i];
			checkB = checkB + checkA;
		}
		if (ackSentence[8] != checkA || ackSentence[9] != checkB) {
			return -2; // Ack Error: bad checksum
		}
		if (ackSentence[6] != message[0] || ackSentence[7] != message[1]) {
			return -1; // Ack Error: response to wrong command
		}
		if (ackSentence[3] != 0x01) {
			return 0; // Message is ACK-NAK
		}
		else {
			return 1; // Reached end of checks, message is ACK-ACK
		}
	}

	bool sendUntilAck(const uint8_t message[], const int messageLength) {
		unsigned int tries = 0;
		int ackReturn = -5;
		do {
			sendUBX(message, messageLength);
			do {
				delay(10);
				ackReturn = getUbxAck(message);
			} while (ackReturn == -1);
			if (ackReturn == 1) return true;
			else if (ackReturn == 0) return false;
			tries++;
		} while (tries < 5);
		return false;
	}

	// Set UART1 to 115200 baud
	// Relies on ubx_cfg_prt[1] being set to 115200
	bool changeBaudRate() {
		unsigned int tries = 0;
		int ackReturn = -5;
		do {
			sendUBX(g::ubx_cfg_prt[1], 24);
			Serial3.begin(115200);
			sendUBX(g::ubx_cfg_prt[1], 24);
			do {
				delay(10);
				ackReturn = getUbxAck(g::ubx_cfg_prt[1]);
			} while (ackReturn == -1);
			if (ackReturn == 1) return true;
			else if (ackReturn == 0) return false;
			Serial3.begin(9600);
			tries++;
		} while (tries < 5);
		return false;
	}

	// Load settings into gps module
	bool config() {
		// Check to see if GPS is configured by sending request for data
		Serial3.begin(115200);
		if (sendUntilAck(g::ubx_cfg_ant, 8))  {
			//Serial.println("GPS already configured.");
			return true; // Good to go!
		}
		
		//Serial.println("GPS is not configured.");
		// GPS did not respond at pre-configured baud rate, so try default
		Serial3.begin(9600);
		if (!sendUntilAck(g::ubx_cfg_ant, 8)) {
			//Serial.println("GPS not responding.");
			return false; // Comm error?
		}

		// GPS responded at default comm rate, so it needs configuring
		if (!changeBaudRate()) {
			//Serial.println("GPS baud rate could not be configured.");
			return false;
		}
		
		int cfgFailure = 0;

		// Start sending settings en masse
		if (!sendUntilAck(g::ubx_cfg_gnss,			48)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_itfm,			12)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_logfilt, 	16)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_nav5,			40)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_navx5, 		44)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_nmea,			24)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_odo,				24)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_pm2,				48)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_rate,			10)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_rinv,			28)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_sbas,			12)) cfgFailure++;
		if (!sendUntilAck(g::ubx_cfg_usb,			 112)) cfgFailure++;
		for (int i = 0; i < 3; i++) {
			if (!sendUntilAck(g::ubx_cfg_inf[i],	14))	cfgFailure++;
		}
		for (int i = 0; i < 50; i++) {
			if (!sendUntilAck(g::ubx_cfg_msg[i],	12))	cfgFailure++;
		}
		for (int i = 0; i < 4; i++) {
			if (i != 1) // UART1 cfg already sent and confirmed
				if (!sendUntilAck(g::ubx_cfg_prt[i], 24)) cfgFailure++;
		}
		for (int i = 0; i < 2; i++) {
			if (!sendUntilAck(g::ubx_cfg_tp5[i],	 36))	cfgFailure++;
		}

		if (cfgFailure == 0) return true;
		else  return false;
	}

	// Returns true if sentence parsed
	bool parseSentence() {
		int sentenceEnd = sentence.length() - 1;
		bool sentenceValid = false;
		int valStart = 0;
		int valEnd = 0;
		int valIndex = 0;
		String value;
		
		valEnd = sentence.indexOf(',', valStart);
		if (valEnd > 0) {
			if (valEnd + 3 <= sentenceEnd) valEnd += 3; // Skip comma and include msgID
			value = sentence.substring(valStart, valEnd);
			if (value == "$PUBX,00") {
				sentenceValid = true;
				valStart = valEnd + 1; // Skip the next comma delimiter
				valIndex += 2;
			}
		}

		if (sentenceValid) {
			while (valEnd < sentenceEnd) {
				valEnd = sentence.indexOf(',', valStart); // index of comma
				if (valEnd < 0) valEnd = sentenceEnd; // indexOf returns -1 if no token found

				value = sentence.substring(valStart, valEnd); // Start inclusive, end exclusive
			
				switch(valIndex) {
					case PUBX00_TIME:
						time = value.toFloat() + TIMEZONE;
						if (time < 0) time += 2400;
						else if (time > 2400) time -= 2400;
						break;
					case PUBX00_LAT:
						latitude = value.toFloat();
						break;
					case PUBX00_NS:
						// Represent southern latitude as negative
						if (value == "S") latitude = -latitude;
						break;
					case PUBX00_LONG:
						longitude = value.toFloat();
						break;
					case PUBX00_EW:
						// Represent eastern latitude as negative
						if (value == "E") longitude = -longitude;
						break;
					case PUBX00_ALT:
						altitude = value.toFloat();
						break;
					case PUBX00_LOCK:
						if (value == "G3" || value == "D3" || value == "RK") gpsLock = true;
						else gpsLock = false;
						break;
					case PUBX00_HACC:
						accuracy = value.toFloat();
						break;
					case PUBX00_VACC: // This comes after PUBX00_HACC
						accuracy = sqrt(pow(accuracy, 2.0) + pow(value.toFloat(), 2.0));
						break;
					case PUBX00_SOG:
						groundSpeed = 0.2777778*value.toFloat(); // km/hr to m/s
						break;
					case PUBX00_COG:
						groundCourse = value.toFloat();
						break;
					case PUBX00_VVEL:
						rateOfClimb = -value.toFloat(); // Need to change sign
						break;
					case PUBX00_NSAT:
						nSats = value.toInt();
						break;
				}

				valStart = valEnd + 1;
				valIndex ++;
			}
			badData = false;
			newData = true;
			sentence = "";
			return true;
		}
		else {
			badData = true;
			sentence = "";
			return false;
		}
	}

public:
	bool newData;				// Reset next time measure() is called
	bool badData;				// 
	bool gpsLock;				// True if 3d gps lock is maintained
	float time; 				// hhmmss.ss; 24-hour format
	float latitude;			// ddmm.mmmmm; Positive if North, negative if South
	float longitude;		// dddmm.mmmmm; Positive if West, negative if East
	float altitude;			// Meters
	float rateOfClimb;	// Meters/second; Positive for increasing altitude
	float groundSpeed;	// Meters/second;
	float groundCourse; // 0deg is North (duh)
	float accuracy;			// Spherical accuracy estimate
	unsigned int nSats;	// number of satellites

	bool init() {
		receiving = false;
		newData = false;
		badData = false;
		timer = 0;
		sentence.reserve(256);

		return config();
	}

	void receive() {
		newData = false;

		if (receiving) {
			while(Serial3.available()) {
				sentence += char(Serial3.read());
				timer = 0;
			}
			if (timer >= 1000) { // 9600 baud is 833 us/byte
				receiving = false;
				parseSentence();
			}
		}
		else if(Serial3.available()) {
			sentence += char(Serial3.read());
			receiving = true;
			timer = 0;
		}
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
		off();
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

class LEDinterface {
private:
	unsigned int pin;

public:
	void init(unsigned int pin) {
		this->pin = pin;
		pinMode(pin, OUTPUT);

		write(0);
	}

	void write(unsigned int value) {
		if (value >= 255) analogWrite(pin, PWM_MAX);
		else if (value <= 0) analogWrite(pin, 0);
		else if (value <= 4) analogWrite(pin, 1);
		else {
			analogWrite(pin, int((PWM_MAX/65025.0)*pow(value, 2.0) + 0.5));
		}
	}
};

class BatterySense {
private:
	static const float voltLookup[];
	static const float capLookup[];
	static const int tblSize;

	unsigned int snsPin;
	unsigned int adcReading;
	float avgTConst;
	float voltMult;
	float capStart;

	// Search for value in tableOne, then interpolate between tableTwo
	// Assumes tableOne is sorted in ascending order
	float interpolate(float value, const float tableOne[], const float tableTwo[], const int tableSize) {
		if (value < tableOne[0]) return tableTwo[0];
		if (value > tableOne[tableSize-1]) return tableTwo[tableSize-1];

		int first = 0;
		int last = tableSize - 1;
		int middle;

		// Binary search to find the table values bracketing the lookup value
		while (last - first > 1) {
			middle = (last + first)/2;
			if (value > tableOne[middle])
				first = middle;
			else if (value < tableOne[middle]) last = middle;
			else return capLookup[middle];
		}

		// Linear interpolation
		return tableTwo[first] + (tableTwo[last] - tableTwo[first])*(value - tableOne[first])/(tableOne[last] - tableOne[first]);
	}

public:
	float voltage;
	bool empty;

	void init(unsigned int batSensePin, float voltageMultiplier, float avgTConst) {
		this->snsPin = batSensePin;
		this->voltMult = voltageMultiplier;
		if (avgTConst > 0.0 && avgTConst <= 100.0) this->avgTConst = 0.63212*(1.0/avgTConst);
		else this->avgTConst = 0.063212;

		float thisVoltage;
		for (int i = 0; i <5; i++) {
			thisVoltage += (V_REF/ADC_MAX)*voltMult*float(analogRead(snsPin));
		}
		voltage = (thisVoltage/5.0);

		// Assume that this was taken when millis() was roughly equal to zero
		capStart = getCap();
	}

	void read(unsigned long microsElapsed) {
		float thisVoltage = (V_REF/ADC_MAX)*voltMult*float(analogRead(snsPin));
		float tc = avgTConst*1.0e-6*float(microsElapsed);

		if (tc < 1.0) // Needs to be less than time constant for calculation to make sense
			voltage = (1.0-tc)*voltage + tc*thisVoltage;
		else voltage = thisVoltage;

		if (empty && voltage > 3.0) empty = false;
		else if (voltage < 2.8) empty = true;
	}

	float getCap() { // Interpolate lookup tables for accurate capacity
		return interpolate(voltage, voltLookup, capLookup, tblSize);
	}
};

class Rainbow {
private:
	int currentColor;
	RGBinterface *rgb;
	elapsedMicros timer;
	int i;

public:
	bool active;

	void init(RGBinterface *rgb) {
		this->rgb = rgb;
		active = false;
		timer = 0;
		i = 0;
	}

	void cycle() {
		if (active) {
			if (timer >= 4000) {
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
					rgb->color(i, 0, 0);
					break;
				case 1:
					rgb->color(255, i, 0);
					break;
				case 2:
					rgb->color((255-i), 255, 0);
					break;
				case 3:
					rgb->color(0, 255, i);
					break;
				case 4:
					rgb->color(0, (255-i), 255);
					break;
				case 5:
					rgb->color(0, 0, (255-i));
					break;
			}
		}
	}

	void on() {
		active = true;
		currentColor = 0;
		timer = 0;
		i = 0;
	}

	void off() {
		active = false;
		rgb->color(0, 0, 0);
	}
};

// Specifically for Panasonic NCR18650B
const float BatterySense::voltLookup[] = { // Assumes < 0.2A current draw
	2.805,2.900,2.984,3.075,3.150,3.192,3.241,3.274,3.301,3.327,3.367,3.406,3.430,3.453,
	3.478,3.495,3.511,3.530,3.541,3.558,3.571,3.583,3.599,3.616,3.630,3.653,3.674,3.697,
	3.725,3.753,3.774,3.799,3.823,3.860,3.888,3.909,3.939,3.967,4.016,4.053,4.081,4.107,4.156};
const float BatterySense::capLookup[] = { // Assumes < 0.2A current draw
	0.000,0.097,0.644,1.396,2.285,2.969,4.336,6.593,9.738,12.06,14.66,17.33,19.38,21.84,
	24.78,26.90,29.16,32.24,34.29,37.09,39.83,41.88,44.34,47.28,49.60,52.34,54.87,57.19,
	60.00,62.32,64.72,67.25,69.98,74.22,77.37,79.69,82.49,85.02,88.79,92.48,95.28,97.67,100.0};
const int BatterySense::tblSize = 43;

class Heater {
private:
	LMT84LPinterface *thermometer;
	unsigned int heaterPin;
	float cP;
	float cI;

public:
	float target;
	float error;
	float dutyCycle;
	float integral;

	void init(unsigned int heaterPin, LMT84LPinterface *thermometer, float target, float cP, float cI) {
		this->heaterPin = heaterPin;
		this->thermometer = thermometer;
		this->target = target;
		this->cP = cP;
		this->cI = cI;
	}

	void update(unsigned long microsElapsed) {
		thermometer->read(microsElapsed);
		error = target - thermometer->celsius;
		integral = constrain(integral + error*cI*microsElapsed*1.0e-6, -1.0, 1.0);
		dutyCycle = constrain(error*cP + integral, 0.0, 1.0);

		analogWrite(heaterPin, int(PWM_MAX*dutyCycle + 0.5));
	}
};

enum stateEnum {MONITORING, ARMED, EXECUTING, COMPLETE};

/*
class Parachute {
private:
GPOinterface *wire;

	bool maxAltIncreasedLast;
	bool outsideBoundariesLast;
	bool aboveArmAltLast;
	bool fallingLast;

public:
	stateEnum state;
	bool maxAltIncreased;
	bool outsideBoundaries;
	bool aboveArmAlt;
	bool falling;

	ElapsedMillis cutTimer;
	ElapsedMillis timerA;
	ElapsedMillis timerB;

	void init(GPOinterface &wire) {
		this->wire = wire;
		state = MONITORING;
		maxAltIncreasedLast = false;
		outsideBoundariesLast = false;
		aboveArmAltLast = false;
		fallingLast = false;
		maxAltIncreased = false;
		outsideBoundaries = false;
		aboveArmAlt = false;
		falling = false;
	}

	void update() {
		switch(state) {
			default:
			case MONITORING:
				if (aboveArmAlt) {
					if (!aboveArmAltLast) timerA = 0;
					if (timer >= 30*1000) state = ARMED;
				}
				aboveArmAltLast = aboveArmAlt;
				break;
			case ARMED:
				if (outsideBoundaries) {
					if (!outsideBoundariesLast) timerA = 0;
					if (timerA >= 60*1000) {
						state = EXECUTING;
						wire.on(1.0, 4.2);
						timerA = 0;
					}
				}
				outsideBoundariesLast = outsideBoundaries;
				if (falling) {
					if (!fallingLast) timerB = 0;
					if (timerB >= 10*1000) {
						state = EXECUTING;
						wire.on(1.0, 4.2);
						timerA = 0;
					}
				}
				fallingLast = falling;
				if (cutTimer >= 3*3600*1000) {
					state = EXECUTING;
					wire.on(1.0, 4.2);
					timerA = 0;
				}
				break;
			case EXECUTING:
				if (timerA >= 10*1000) {
					state = COMPLETE;
					wire.off();
				}
			case COMPLETE:
				break;
		}
	}

};

class CutDown {
private:
	GPOinterface *wire;
	Battery *battery;

	bool maxAltIncreasedLast;
	bool outsideBoundariesLast;
	bool aboveArmAltLast;
	bool fallingLast;

public:
	stateEnum state;
	bool maxAltIncreased;
	bool outsideBoundaries;
	bool aboveArmAlt;
	bool falling;

	elapsedMillis cutTimer;
	elapsedMillis timerA;
	elapsedMillis timerB;

	void init(GPOinterface &wire, Battery &battery) {
		this->wire = wire;
		this->battery = battery;

		state = MONITORING;
		maxAltIncreasedLast = false;
		outsideBoundariesLast = false;
		aboveArmAltLast = false;
		fallingLast = false;
		maxAltIncreased = false;
		outsideBoundaries = false;
		aboveArmAlt = false;
		falling = false;
	}

	void update() {
		switch(state) {
			default:
			case MONITORING:
				if (aboveArmAlt) {
					if (!aboveArmAltLast) timerA = 0;
					if (timer >= 30*1000) state = ARMED;
				}
				aboveArmAltLast = aboveArmAlt;
				break;
			case ARMED:
				if (outsideBoundaries) {
					if (!outsideBoundariesLast) timerA = 0;
					if (timerA >= 60*1000) {
						state = EXECUTING;
						wire.on(1.0, battery.voltage);
						timerA = 0;
					}
				}
				outsideBoundariesLast = outsideBoundaries;
				if (falling) {
					if (!fallingLast) timerB = 0;
					if (timerB >= 10*1000) {
						state = EXECUTING;
						wire.on(1.0, battery.voltage);
						timerA = 0;
					}
				}
				fallingLast = falling;
				if (cutTimer >= 3*3600*1000) {
					state = EXECUTING;
					wire.on(1.0, battery.voltage);
					timerA = 0;
				}
				break;
			case EXECUTING:
				if (timerA >= 10*1000) {
					state = COMPLETE;
					wire.off();
				}
			case COMPLETE:
				break;
		}
	}
};
*/

//////////////////////
// Global Variables //
//////////////////////
SDinterface sdCard;
LMT84LPinterface thermBat;
LMT84LPinterface thermExt;
MS5607interface barometer;
//MPU9250interface inertial;
MAXM8interface gps;
PushButton button;
GPOinterface paraNichrome;
GPOinterface cutNichrome;
GPOinterface testNichrome;
BatterySense battery;
RGBinterface rgb;
LEDinterface led;
Heater batteryHeat;
Rainbow rainbowLED;

elapsedMicros loopTimer;
elapsedMillis blinkTimer;
elapsedMillis cutdownTimer;

bool lightToggle;
bool loggingEnabled;

stateEnum cutdownState;
stateEnum parachuteState;

////////////////////
// Main Functions //
////////////////////

void setup() {
	Serial.begin(115200);

	analogWriteFrequency(PWM_A_PIN, PWM_FREQ); // Ideal frequency for 11-bit pwm
	analogWriteFrequency(PWM_B_PIN, PWM_FREQ);
	analogWriteFrequency(PWM_C_PIN, PWM_FREQ);
	analogWriteResolution(PWM_RES);
	analogReadResolution(ADC_RES);

	Wire.begin();

	thermBat.init(THERM_1, 1.0, 1.0176, -2.0087);
	thermExt.init(THERM_2, 1.0, 1.0176, -2.0087);
	paraNichrome.init(GPO_2, 1.4);
	//cutNichrome.init(GPO_1, 4.2);
	//testNichrome.init(GPO_4, 4.2);
	batteryHeat.init(GPO_3, &thermBat, 25.0, 1.0, 0.5);
	battery.init(BAT_VSNS, BAT_VSNS_MULT, 1.0);
	button.init(PUSH_BUT, true, 8000);
	rgb.init(RGB_R, RGB_G, RGB_B, true);
	rainbowLED.init(&rgb);
	led.init(LED_2);

	if (!gps.init()) while(1) {
		Serial.println("GPS initialization failed.");
		rgb.color(127,0,0);
		delay(500);
		rgb.color(0,0,0);
		delay(500);
	}
	if (!barometer.init(MS5607_ADDR)) while(1) {
		Serial.println("Barometer initialization failed.");
		rgb.color(127,0,0);
		delay(500);
		rgb.color(0,0,0);
		delay(500);
	}
	/*
	if (!sdCard.init(SD_CS)) while(1) {
		Serial.println("SD card initialization failed.");
		rgb.color(127,0,0);
		delay(500);
		rgb.color(0,0,0);
		delay(500);
	}
	*/

	loopTimer = 0;
	blinkTimer = 0;
}

void loop() {	

	gps.receive();
	battery.read(loopTimer);
	batteryHeat.update(loopTimer);
	if (!barometer.measComplete) barometer.measure();
	thermExt.read(loopTimer);
	rainbowLED.cycle();
	button.read();

	loopTimer = 0;

	if (gps.newData) {
		barometer.measure();

		sdCard.buffer(gps.time, CSV_TIME, 1);
		sdCard.buffer(gps.latitude, CSV_LAT, 5);
		sdCard.buffer(gps.longitude, CSV_LONG, 5);
		sdCard.buffer(gps.altitude, CSV_ALT, 2);
		sdCard.buffer(gps.rateOfClimb, CSV_ROC, 2);
		sdCard.buffer(gps.groundSpeed, CSV_GNDSPD, 2);
		sdCard.buffer(gps.groundCourse, CSV_GNDCRS, 1);
		sdCard.buffer(gps.accuracy, CSV_ACC, 2);
		sdCard.buffer(gps.nSats, CSV_NSATS);
		sdCard.buffer(thermExt.celsius, CSV_EXTTEMP, 2);
		sdCard.buffer(thermBat.celsius, CSV_BATTEMP, 2);
		sdCard.buffer(barometer.celsius, CSV_INTTEMP, 2);
		sdCard.buffer(barometer.kPa, CSV_PRES, 2);
		sdCard.buffer(battery.voltage, CSV_BATVOLT, 2);
		sdCard.buffer(battery.getCap(), CSV_BATCAP, 2);
		sdCard.buffer(batteryHeat.dutyCycle, CSV_BATHEAT, 2);

		for (int i = 0; i < CSV_BUFFER_SIZE; i++) {
			Serial.print(sdCard.cvsBuffer[i]);
			Serial.print(",");
		}
		Serial.println();

		//if (rainbowLED.active) sdCard.logToSD();

		blinkTimer = 0;
		lightToggle = true;
		led.write(255);
	}
	if (lightToggle && blinkTimer > 200) {
		led.write(0);
	}

	if (button.fell) {
		paraNichrome.off();
		rainbowLED.off();
	}
	else if (button.rose) {
		paraNichrome.on(1.0, battery.voltage);
		rainbowLED.on();
	}

	// Balloon cutdown sequence:
	// Arm cutdown if:
	//		altitude > 10000ft for more than 60s
	// Initiate cutdown if:
	// 		(New max altitude is unset for more than 10mins OR
	// 		Location exceeds boundary box for more than 60s)
	// 
	// Parachute deployment sequence:
	// Arm parachute if:
	// 		(RoC <= -20 ft/min for more than 20s AND
	// 		currentAltitude < maxAltitude-1000ft for more than 20s)
	// Deploy parachute if:
	// 		(currentAltitude < 10000ft AND 
	// 		RoC <= -20 ft/min)
	// 		
	
}