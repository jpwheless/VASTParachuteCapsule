void setup() {
	Serial.begin(115200); // 115200
	Serial3.begin(115200);

	pinMode(13, OUTPUT);
}

void loop() {
	while(Serial3.available()) {
		Serial.write(Serial3.read());
		digitalWriteFast(13, HIGH);
	}
	while(Serial.available()) {
		Serial3.write(Serial.read());
		digitalWriteFast(13, HIGH);
	}
	digitalWriteFast(13, LOW);
}