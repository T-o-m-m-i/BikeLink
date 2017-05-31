#include "Libs/RHReliableDatagram.h"
#include "Libs/RH_RF95.h"
#include <SPI.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define RFM95_RST 9
#define RF95_FREQ 868.0
#define RF95_PWR 17
#define BATTERY_PIN A0
#define BTN_INTERRUPT_PIN 3
#define BUZZER_PIN A3
#define LATCH_PIN 7
#define SCL_PIN 6
#define SDA_PIN 5

uint8_t * digitalValues;
//char const handShakeCode[] = "6d950a629f27ee411602f1640a8492b1";

struct dataStruct
{
	bool alarmState;
	uint16_t TX_Voltage; //ADC = 10-bit = 0-1023

} SensorReadings;

uint16_t RX_Voltage;
uint8_t rssiValue;
volatile uint8_t buttonState = 0;
volatile uint8_t lastButtonState = 0;
volatile boolean alarmState = false;
volatile boolean buzzerState = true;
volatile boolean displayUpdated = false;

bool TX_responded_OK = false;

// Instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

void buttonInterrupt(void)
{
	cli();
	static unsigned long last_interrupt_time = 0;
	unsigned long interrupt_time = millis();
	// If interrupts come faster than 200ms, assume it's a bounce and ignore
	if (interrupt_time - last_interrupt_time > 200)
	{
		buttonState++;
		if( buttonState % 4 == 0 )
			buttonState = 0;

		displayUpdated = false;

		if(alarmState)
			buzzerState = false;

	}
	last_interrupt_time = interrupt_time;

	//Serial.print("Button: ");
	//Serial.print(buttonState);
	//Serial.println("");
	sei();
}

void setup()
{
	// ADC0
	pinMode(A0, INPUT);

	// Button pin
	pinMode(3,INPUT_PULLUP);

	// Buzzer pin
	pinMode(BUZZER_PIN, OUTPUT);

	// Display pins
	pinMode(LATCH_PIN, OUTPUT);
	pinMode(SCL_PIN, OUTPUT);
	pinMode(SDA_PIN, OUTPUT);

	// Reset pin
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);

	// Manual reset the radio
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);
	delay(10);

	Serial.begin(9600);
	while (!Serial) ; // Wait for serial port to be available

	// Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
	if (!manager.init())
		Serial.println("init failed");

	if (!driver.setFrequency(RF95_FREQ)) {
		while (1);
	}

	// The default transmitter power is 13dBm, using PA_BOOST.
	// If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
	// you can set transmitter powers from 5 to 23 dBm:
	driver.setTxPower(17, false);
	// If you are using Modtronix inAir4 or inAir9,or any other module which uses the
	// transmitter RFO pins and not the PA_BOOST pins
	// then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
	// Failure to do that will result in extremely low transmit powers.
	//  driver.setTxPower(14, true);
	// You can optionally require this module to wait until Channel Activity
	// Detection shows no activity on the channel before transmitting by setting
	// the CAD timeout to non-zero:
	//driver.setCADTimeout(1000);
	manager.setRetries(5);
	manager.setTimeout(500);

	attachInterrupt(digitalPinToInterrupt(BTN_INTERRUPT_PIN), buttonInterrupt, RISING);


	// Allocates the specified number of bytes and initializes them to zero
	digitalValues = (uint8_t *)malloc(3 * sizeof(uint8_t));
	memset(digitalValues, 0, 3 * sizeof(uint8_t));

	rssiValue = 0;
}

uint8_t data[] = "And hello back!";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

static unsigned long last_time = 0;

void loop()
{
	// Don't put this on the stack:
	uint8_t buf[sizeof(SensorReadings)];
	uint8_t len = sizeof(buf);
	uint8_t from = 0x00;

	if (manager.available())
	{
		Serial.println("RX manager.available()");
		// Wait for a message addressed to us from the client
		if (manager.recvfromAck(buf, &len, &from))
		{
			memcpy(&SensorReadings, buf, sizeof(SensorReadings));

			Serial.print("RX got message from : 0x");
			Serial.print(from, HEX);
			Serial.print(": ");
			Serial.print(SensorReadings.TX_Voltage);
			Serial.print(": ");
			Serial.println(SensorReadings.alarmState);

			// Set the alarmState ON permanently
			if(!alarmState)
				alarmState = SensorReadings.alarmState;

			rssiValue = driver.lastRssi();

			flashDisplay();
			displayUpdated = false;
			//while(!manager.sendtoWait(data, sizeof(data), from))
			//{
			//	Serial.println("RX Failed 'sendtoWait'");
			//}
			//Serial.println("RX Succesfull 'sendtoWait'");


			// Send a reply back to the originator client
			//if (!manager.sendtoWait(data, sizeof(data), from))
			//  Serial.println("RX sendtoWait failed");
		}
	}

	if(alarmState && buzzerState)
	{
		//Buzzzz
		alarmSound();
	}

	unsigned long receive_time = millis();

	if(!displayUpdated)
		updateDisplay(buttonState);



	if (receive_time - last_time > 10000)
	{
		// Set the alarmState ON permanently
		if(!alarmState)
			alarmState = true;

		rssiValue = 0;
		Serial.println("TX silent too long!");
	}
	last_time = receive_time;

}

void updateDisplay(uint8_t state)
{
	switch(state)
	{
		case 0:{
				//Display off
				displayOff();
				displayUpdated = true;
				break;}
		case 1:{
				//Display RSSI
				displayNumber(rssiValue, false);
				//Serial.println(rssiValue);
				displayUpdated = true;
				break;}
		case 2:{
				//Display TX voltage
				long raw = SensorReadings.TX_Voltage;
				int voltage = (int)map(raw, 0, 1023, 0, 432);
				displayNumber(voltage, false);
				//Serial.println(voltage);
				displayUpdated = true;
				break;}
		case 3:{
				//Display RX voltage
				readVoltage();
				int voltage = (int)map(RX_Voltage, 0, 1023, 0, 432);
				displayNumber(voltage, false);
				//Serial.println(voltage);
				break;}

		default:
				//Display off
				//display(0);
			/*	digitalWrite(LATCH_PIN, LOW);
				int byte;
				digitalValues[0]=B11111100;
				digitalValues[1]=B01100000;
				digitalValues[2]=B11011010;

				for (byte = 3 - 1; byte >= 0; byte--)
				{
					shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~digitalValues[byte]);
				}
				digitalWrite(LATCH_PIN, HIGH);
				digitalWrite(LATCH_PIN, LOW);*/

				break;

	}
}

int LED_SEG_TAB[]={
  0xfc,0x60,0xda,0xf2,0x66,0xb6,0xbe,0xe0,0xfe,0xf6,0x01,0xee,0x3e,0x1a,0x7a,0x9e,0x8e,0x01,0x00};
//0     1    2     3    4    5    6    7    8    9   dp   .    a    b    c    d    e    f   off

void displayNumber(int value, boolean leadingZero)
{
	int a,b,c;
	a = value / 100;
	value = value % 100;
	b = value / 10;
	value = value % 10;
	c = value;

	if (leadingZero==false) // Remove leading zeros
	{
		if (a==0 && b>0)
		{
			a = 18;
		}
		if (a==0 && b==0 && c>0)
		{
			a = 18;
			b = 18;
		}
		if (a==0 && b==0 && c==0)
		{
			a = 18;
			b = 18;
			c = 18;
		}
	}

	digitalWrite(LATCH_PIN, LOW);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~LED_SEG_TAB[c]);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~LED_SEG_TAB[b]);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~LED_SEG_TAB[a]);
	digitalWrite(LATCH_PIN, HIGH);
}
// Turns off all the segments
void displayOff(void)
{
	digitalWrite(LATCH_PIN, LOW);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~0);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~0);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~0);
	digitalWrite(LATCH_PIN, HIGH);
}

void flashDisplay(void)
{
	digitalWrite(LATCH_PIN, LOW);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, 0);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, 0);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, 0);
	digitalWrite(LATCH_PIN, HIGH);

	delay(10);

	digitalWrite(LATCH_PIN, LOW);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~0);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~0);
	shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~0);
	digitalWrite(LATCH_PIN, HIGH);
}
// Alarm sound
void alarmSound(void)
{
	// Play a note on BUZZER_PIN for 500 ms:
	tone(BUZZER_PIN, 494, 500);
	delay(500);
	tone(BUZZER_PIN, 294, 500);
	delay(500);

	// Turn off tone function for BUZZER_PIN:
	noTone(BUZZER_PIN);
}

void readVoltage(void)
{
	RX_Voltage = analogRead(BATTERY_PIN);
}
