#include "Libs/RHReliableDatagram.h"
#include "Libs/RH_RF95.h"
#include "Libs/Bildr_ADXL345.h"
#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define RFM95_RST 9
#define RF95_FREQ 868.0
#define RF95_PWR 17
#define BATTERY_PIN A0
#define ACC_INTERRUPT_PIN 3

volatile boolean activity = false;
volatile boolean activited = false;
volatile bool watchdogActivated = false;
uint8_t sleepIterations = 0;
volatile uint8_t maxSleepIterations = 3;

//char const handShakeCode[] = "6d950a629f27ee411602f1640a8492b1";

struct dataStruct
{
	bool alarmState;
	uint16_t TX_Voltage; //ADC = 10-bit = 0-1023

} SensorReadings;

bool RX_responded_OK = false;

uint8_t data[] = "Hello World!";
// Dont put this on the stack:
//uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t buf[sizeof(SensorReadings)] = {0};


// Singleton instance of the radio driver
RH_RF95 driver;
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);

ADXL345 adxl;

// Run this code if Activity Interrupt was triggered
void activityInterrupt(void)
{
	detachInterrupt(digitalPinToInterrupt(ACC_INTERRUPT_PIN));

	activity = true;
	SensorReadings.alarmState = activity;

	//Set WDT faster
	maxSleepIterations = 1;

	//Set Acc to Idle? -> 0.1uA
}

// The Watchdog Time-out Interrupt
ISR(WDT_vect)
{
	watchdogActivated = true;
}

void setup()
{
	SensorReadings.TX_Voltage = 0;
	SensorReadings.alarmState = false;

	//Interrupt pin
	pinMode(ACC_INTERRUPT_PIN, INPUT);
	//ADC0
	pinMode(BATTERY_PIN, INPUT);
	//Reset pin
	pinMode(RFM95_RST, OUTPUT);
	digitalWrite(RFM95_RST, HIGH);

	// manual reset
	digitalWrite(RFM95_RST, LOW);
	delay(10);
	digitalWrite(RFM95_RST, HIGH);

	delay(10);

	// Rocket Scream Mini Ultra Pro with the RFM95W only:
	// Ensure serial flash is not interfering with radio communication on SPI bus
	//  pinMode(4, OUTPUT);
	//  digitalWrite(4, HIGH);

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

	accSetup();

	// Setup the watchdog timer to run an interrupt which
	// wakes the Arduino from sleep every 8 seconds.

	// Note that the default behavior of resetting the Arduino
	// with the watchdog will be disabled.

	// This next section of code is timing critical, so interrupts are disabled.
	// See more details of how to change the watchdog in the ATmega328P datasheet
	// around page 50, Watchdog Timer.
	noInterrupts();

	// Set the watchdog reset bit in the MCU Status Segister to 0.
	MCUSR &= ~(1<<WDRF);

	// Set WDCE and WDE bits in the Watchdog Timer Control Register.
	WDTCSR |= (1<<WDCE) | (1<<WDE);

	// Set Watchdog Clock Prescaler bits to a value of 8 seconds.
	WDTCSR = (1<<WDP0) | (1<<WDP3);

	// Enable watchdog as interrupt only (no reset).
	WDTCSR |= (1<<WDIE);

	// Enable interrupts again.
	interrupts();

	attachInterrupt(digitalPinToInterrupt(ACC_INTERRUPT_PIN), activityInterrupt, RISING);
	handShake();
}

void loop()
{
	if(watchdogActivated || (activity && !activited))
	{
		watchdogActivated = false;
		sleepIterations += 1;

		if((sleepIterations >= maxSleepIterations) || (activity && !activited))
		{
			if(activity)
				activited = true;

			// Reset the number of sleep iterations.
			sleepIterations = 0;

			Serial.println("TX In loop() ");

			readVoltage();


			//int x,y,z;
			//adxl.readAccel(&x, &y, &z);

			// Output x,y,z values - Commented out
			//Serial.print("ALARM: "); Serial.print(activity); Serial.println("  ");
			//Serial.print("X: "); Serial.print(x); Serial.print("  ");
			//Serial.print("Y: "); Serial.print(y); Serial.print("  ");
			//Serial.print("Z: "); Serial.print(z); Serial.println("  ");

			byte interrupts = adxl.getInterruptSource();

			//activity
			if(adxl.triggered(interrupts, ADXL345_ACTIVITY))
			{
				Serial.println("activity");
				//add code here to do when activity is sensed
			}

			//inactivity
			if(adxl.triggered(interrupts, ADXL345_INACTIVITY))
			{
				Serial.println("inactivity");
				//add code here to do when inactivity is sensed
			}

			sendValues();
		}
	}
}

// Put the Arduino to sleep.
void sleep()
{
	Serial.println("TX Sleepy Time Now");
	Serial.flush();

	//Shut down the radio
	driver.sleep();

	// Set sleep to 'full power down'.  Only external interrupts or
	// the watchdog timer can wake the CPU!
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);

	// Turn off the ADC while asleep.
	power_adc_disable();

	// Enable sleep and enter sleep mode.
	sleep_mode();

	// CPU is now asleep and program execution completely halts!
	// Once awake, execution will resume at this point.

	// When awake, disable sleep mode and turn on all devices.
	sleep_disable();
	power_all_enable();

	//Wake up the radio
	driver.setModeIdle();
}
void sendValues()
{
	uint8_t len = sizeof(SensorReadings);

	//Load message into data-array
	memcpy (buf, &SensorReadings, len);

	if(manager.sendtoWait(buf, len, SERVER_ADDRESS))
		Serial.println("TX Succesfull 'sendtoWait'");
	else
		Serial.println("TX Failed 'sendtoWait'");

	sleep();
}

void readVoltage()
{
	SensorReadings.TX_Voltage = analogRead(BATTERY_PIN);
}

void handShake()
{
	adxl.getInterruptSource();

	readVoltage();

	uint8_t len = sizeof(SensorReadings);

	//Load message into data-array
	memcpy (buf, &SensorReadings, len);

	while(!RX_responded_OK)
	{
		while(!manager.sendtoWait(buf, len, SERVER_ADDRESS))
		{
			Serial.println("TX Failed 'sendtoWait'");
		}
		Serial.println("TX Succesfull 'sendtoWait'");
		RX_responded_OK = true;
	}
	sleep();
}

void accSetup()
{
	adxl.powerOn();
	adxl.set_bw(ADXL345_BW_6);
	adxl.setLowPower(0);
	//adxl.set_bw(ADXL345_BW_25);
	//adxl.setActivityAc(0);
	adxl.setInterruptLevelBit(0);

	//set activity/ inactivity thresholds (0-255)
	adxl.setActivityThreshold(20); //62.5mg per increment
	//adxl.setInactivityThreshold(75); //62.5mg per increment
	//adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?

	//look of activity movement on this axes - 1 == on; 0 == off
	adxl.setActivityX(1);
	adxl.setActivityY(1);
	adxl.setActivityZ(1);

	//look of inactivity movement on this axes - 1 == on; 0 == off
	//adxl.setInactivityX(1);
	//adxl.setInactivityY(1);
	//adxl.setInactivityZ(1);

	adxl.setInterruptMapping(ADXL345_INT_ACTIVITY_BIT, ADXL345_INT1_PIN);
	//adxl.setInterruptMapping(ADXL345_INT_INACTIVITY_BIT, ADXL345_INT1_PIN);

	adxl.setInterrupt(ADXL345_INT_ACTIVITY_BIT, 1);
	//adxl.setInterrupt(ADXL345_INT_INACTIVITY_BIT, 0);
	//adxl.printAllRegister();
}

