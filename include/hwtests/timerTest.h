volatile int interrupts;
int totalInterrupts;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTime() {
	portENTER_CRITICAL_ISR(&timerMux);
	interrupts++;
	digitalWrite(18, !digitalRead(18));
	portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {

	Serial.begin(9600);
	pinMode(18, OUTPUT);
	// Configure Prescaler to 80, as our timer runs @ 80Mhz
	// Giving an output of 80,000,000 / 80 = 1,000,000 ticks / second
	timer = timerBegin(0, 80, true);                
	timerAttachInterrupt(timer, &onTime, true);    
	// attachInterrupt timer
	timerAlarmWrite(timer, 50000, true);			
	timerAlarmEnable(timer);
}

void loop() {
	if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    Serial.println("Received from laptop: "+data);
	if (data[0] == 'x') timerAttachInterrupt(timer, &onTime, true);
	if (data[0] == 'y') timerDetachInterrupt(timer);
	// if (data[0] == 'x') timerAlarmEnable(timer);
	// if (data[0] == 'y') timerAlarmDisable(timer);
	};
	if (interrupts > 0) {
		portENTER_CRITICAL(&timerMux);
		interrupts--;
		portEXIT_CRITICAL(&timerMux);
		totalInterrupts++;
		Serial.print("totalInterrupts");
		Serial.println(totalInterrupts);
		
	}
}