#include "Libs/RHReliableDatagram.h"
#include "Libs/RH_RF95.h"
#include <ShiftDisplay.h>
#include <SPI.h>
#include "Libs/ShiftRegister74HC595.h"

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

// create shift register object (number of shift registers, data pin, clock pin, latch pin)
//ShiftRegister74HC595 led (3, 5, 6, 7);

ShiftDisplay led(LATCH_PIN, SCL_PIN, SDA_PIN, COMMON_ANODE, 3);

struct dataStruct
{
	bool alarmState;
	uint16_t TX_Voltage; //ADC = 10-bit = 0-1023

} SensorReadings;

uint16_t RX_Voltage;
volatile uint8_t buttonState = 0;
volatile boolean alarmState = false;
bool TX_responded_OK = false;

// Instance of the radio driver
RH_RF95 driver;

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);

// Run this code if Button Interrupt was triggered
void buttonInterrupt(void)
{
	detachInterrupt(digitalPinToInterrupt(BTN_INTERRUPT_PIN));
	buttonState++;
	if( buttonState % 4 == 0 )
		buttonState = 0;

	Serial.print("Button: ");
	Serial.print(buttonState);
	Serial.println("");
	delay(100);
	attachInterrupt(digitalPinToInterrupt(BTN_INTERRUPT_PIN), buttonInterrupt, RISING);
}

void setup()
{
	//ADC0
	pinMode(A0, INPUT);

	//Button pin
	pinMode(3,INPUT_PULLUP);

	//Buzzer pin
	pinMode(BUZZER_PIN, OUTPUT);

	//Display pins
	/*pinMode(LATCH_PIN, OUTPUT);
	pinMode(SCL_PIN, OUTPUT);
	pinMode(SDA_PIN, OUTPUT);*/

	//Reset pin
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);

	//Manual reset the radio
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


	// allocates the specified number of bytes and initializes them to zero
	digitalValues = (uint8_t *)malloc(3 * sizeof(uint8_t));
	memset(digitalValues, 0, 3 * sizeof(uint8_t));


}

uint8_t data[] = "And hello back!";
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];

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

			if(!alarmState)
				alarmState = SensorReadings.alarmState;

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

	if(alarmState)
	{
		//Buzzzz
		alarmSound();
	}

	updateDisplay(buttonState);

}

void updateDisplay(uint8_t state)
{
	switch(state)
	{
		case 0:{
				led.print(1000, 1);
				led.print(1000, 2);
				led.print(1000, 3);
				led.print(1000, 4);
				led.print(1000, 5);
				led.print(1000, "001", ALIGN_RIGHT);


				//Display off
				//display(0);
				//led.print(1000, "941");
				break;}
		case 1:{
				//Display off
				//display(1);
				//long raw = SensorReadings.TX_Voltage;
				//long voltage = map(raw, 0, 1020, 0, 420);
				led.set("G");
				led.show(1000);
				break;}
		case 2:{
				//Display off
				//display(1);
				readVoltage();
				//long raw = RX_Voltage;
				//long voltage = map(raw, 0, 1020, 0, 420);
				digitalWrite(LATCH_PIN, LOW);
				int byte;
				digitalValues[0]=B11111100;
				digitalValues[1]=B01100000;
				digitalValues[2]=B11011010;

				for (byte = 3 - 1; byte >= 0; byte--)
				{
					shiftOut(SDA_PIN, SCL_PIN, LSBFIRST, ~digitalValues[byte]);
				}
				digitalWrite(LATCH_PIN, HIGH);
				digitalWrite(LATCH_PIN, LOW);
				break;}
		default:
				//Display off
				//display(0);

				break;

	}
	//attachInterrupt(digitalPinToInterrupt(BTN_INTERRUPT_PIN), buttonInterrupt, RISING);
}

void alarmSound(void)
{
	// play a note on pin 7 for 500 ms:
	tone(BUZZER_PIN, 494, 500);
	delay(500);
	tone(BUZZER_PIN, 294, 500);
	delay(500);

	// turn off tone function for pin 7:
	noTone(BUZZER_PIN);
}

void readVoltage()
{
	RX_Voltage = analogRead(BATTERY_PIN);
}
