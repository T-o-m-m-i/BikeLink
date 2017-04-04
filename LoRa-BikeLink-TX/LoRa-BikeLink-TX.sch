EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:MyLib
LIBS:LoRa-BikeLink-TX-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RFM95 U1
U 1 1 58AC423B
P 8150 3600
F 0 "U1" H 8150 3150 60  0000 C CNN
F 1 "RFM95" H 8150 4050 60  0000 C CNN
F 2 "MyLib:RFM95" H 8150 3700 60  0001 C CNN
F 3 "" H 8150 3700 60  0001 C CNN
	1    8150 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 3850 8800 3850
Wire Wire Line
	8700 3550 9850 3550
Text Label 7600 3350 2    60   ~ 0
Lora_MISO
Text Label 6100 4400 0    60   ~ 0
Lora_MOSI
Text Label 6100 4300 0    60   ~ 0
Lora_MISO
Text Label 6100 4200 0    60   ~ 0
Lora_SCK
Text Label 7600 3450 2    60   ~ 0
Lora_MOSI
Text Label 7600 3550 2    60   ~ 0
Lora_SCK
Text Label 7600 3750 2    60   ~ 0
Lora_RESET
$Comp
L CONN_01X02 P1
U 1 1 58AED1B7
P 9350 4000
F 0 "P1" H 9350 4150 50  0000 C CNN
F 1 "CONN_01X02" V 9450 4000 50  0000 C CNN
F 2 "MyLib:SMA_EDGE" H 9350 4000 50  0001 C CNN
F 3 "" H 9350 4000 50  0000 C CNN
	1    9350 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8700 3950 9150 3950
Wire Wire Line
	8800 4050 9150 4050
$Comp
L CONN_01X04 P2
U 1 1 58AEE323
P 2300 1250
F 0 "P2" H 2300 1500 50  0000 C CNN
F 1 "CONN_01X04" V 2400 1250 50  0000 C CNN
F 2 "MyLib:TPS61200_Tiny_Module" H 2300 1250 50  0001 C CNN
F 3 "" H 2300 1250 50  0000 C CNN
	1    2300 1250
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR2
U 1 1 58AEEA76
P 2600 1550
F 0 "#PWR2" H 2600 1300 50  0001 C CNN
F 1 "GND" H 2600 1400 50  0000 C CNN
F 2 "" H 2600 1550 50  0000 C CNN
F 3 "" H 2600 1550 50  0000 C CNN
	1    2600 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 1300 2600 1300
Wire Wire Line
	2600 1300 2600 1550
Wire Wire Line
	2500 1100 2700 1100
Wire Wire Line
	2500 1200 2700 1200
Wire Wire Line
	2500 1400 2700 1400
Text Label 2700 1400 0    60   ~ 0
PWR_OUT
Text Label 2700 1200 0    60   ~ 0
BATT_IN
Text Label 2700 1100 0    60   ~ 0
PWR_EN
$Comp
L GND #PWR5
U 1 1 58AF049E
P 4100 5300
F 0 "#PWR5" H 4100 5050 50  0001 C CNN
F 1 "GND" H 4100 5150 50  0000 C CNN
F 2 "" H 4100 5300 50  0000 C CNN
F 3 "" H 4100 5300 50  0000 C CNN
	1    4100 5300
	1    0    0    -1  
$EndComp
$Comp
L ADXL345_brkout U2
U 1 1 58AF0D5E
P 2650 3900
F 0 "U2" H 2000 4350 60  0000 C CNN
F 1 "ADXL345_brkout" H 2350 4250 60  0000 C CNN
F 2 "MyLib:ADXL345_BRKOUT" H 2650 3900 60  0001 C CNN
F 3 "" H 2650 3900 60  0001 C CNN
	1    2650 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR3
U 1 1 58AF1B0F
P 3700 4400
F 0 "#PWR3" H 3700 4150 50  0001 C CNN
F 1 "GND" H 3700 4250 50  0000 C CNN
F 2 "" H 3700 4400 50  0000 C CNN
F 3 "" H 3700 4400 50  0000 C CNN
	1    3700 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 4400 3700 3800
Wire Wire Line
	3700 3800 3000 3800
Text Label 3000 3900 0    60   ~ 0
I2C_SCL
Text Label 3000 4000 0    60   ~ 0
I2C_SDA
Text Label 1700 3700 2    60   ~ 0
PWR_OUT
Text Label 6100 3700 0    60   ~ 0
PWR_OUT
Wire Wire Line
	4750 3700 4100 3700
Wire Wire Line
	4100 3700 4100 5300
Text Label 5100 5150 3    60   ~ 0
I2C_SCL
Text Label 5250 5150 3    60   ~ 0
I2C_SDA
Text Label 1700 4000 2    60   ~ 0
ACC_INT1
Text Label 4750 3900 2    60   ~ 0
ACC_INT1
Text Label 9850 3050 0    60   ~ 0
PWR_OUT
NoConn ~ 1700 3800
NoConn ~ 1700 3900
NoConn ~ 8700 3750
NoConn ~ 8700 3650
NoConn ~ 8700 3250
NoConn ~ 5600 5150
NoConn ~ 5750 5150
Text Label 6100 4100 0    60   ~ 0
ADC_0
$Comp
L R R1
U 1 1 58B570FF
P 4050 1250
F 0 "R1" V 4130 1250 50  0000 C CNN
F 1 "39k" V 4050 1250 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3980 1250 50  0001 C CNN
F 3 "" H 4050 1250 50  0000 C CNN
	1    4050 1250
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 58B57434
P 4050 1550
F 0 "R2" V 4130 1550 50  0000 C CNN
F 1 "120k" V 4050 1550 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 3980 1550 50  0001 C CNN
F 3 "" H 4050 1550 50  0000 C CNN
	1    4050 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 1400 4050 1400
Text Label 4050 1100 0    60   ~ 0
BATT_IN
$Comp
L GND #PWR4
U 1 1 58B578F5
P 4050 1700
F 0 "#PWR4" H 4050 1450 50  0001 C CNN
F 1 "GND" H 4050 1550 50  0000 C CNN
F 2 "" H 4050 1700 50  0000 C CNN
F 3 "" H 4050 1700 50  0000 C CNN
	1    4050 1700
	1    0    0    -1  
$EndComp
Text Label 4150 1400 0    60   ~ 0
ADC_0
Wire Wire Line
	6750 3650 7600 3650
$Comp
L GND #PWR7
U 1 1 58B59888
P 9850 3950
F 0 "#PWR7" H 9850 3700 50  0001 C CNN
F 1 "GND" H 9850 3800 50  0000 C CNN
F 2 "" H 9850 3950 50  0000 C CNN
F 3 "" H 9850 3950 50  0000 C CNN
	1    9850 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 3050 9850 3650
$Comp
L C C1
U 1 1 58B598FF
P 9850 3800
F 0 "C1" H 9875 3900 50  0000 L CNN
F 1 "0.1u" H 9875 3700 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 9888 3650 50  0001 C CNN
F 3 "" H 9850 3800 50  0000 C CNN
	1    9850 3800
	1    0    0    -1  
$EndComp
Connection ~ 9850 3550
Wire Notes Line
	3700 650  3700 2350
Wire Notes Line
	2000 700  2000 2300
NoConn ~ 6100 3400
$Comp
L ARDUPROMINI uP1
U 1 1 58C14A6F
P 5450 3700
F 0 "uP1" H 5400 4150 60  0000 C CNN
F 1 "ARDUPROMINI" H 5400 4250 60  0000 C CNN
F 2 "MyLib:ArduProMini_only_sideholes" H 6050 5300 60  0001 C CNN
F 3 "" H 6050 5300 60  0000 C CNN
	1    5450 3700
	1    0    0    -1  
$EndComp
Text Label 8700 3450 0    60   ~ 0
Lora_DIO0
Text Label 4750 3800 2    60   ~ 0
Lora_DIO0
Text Label 8700 3350 0    60   ~ 0
Lora_DIO1
Text Label 4750 4000 2    60   ~ 0
Lora_DIO1
Text Notes 2300 850  0    60   ~ 0
TPS61200_Tiny_Module
Text Notes 1000 850  0    60   ~ 0
Battery connector
Text Notes 3900 900  0    60   ~ 0
Vin = 4.3 V\nR1 =  39kΩ\nR2 =  120kΩ\nVout = 3.25 V
NoConn ~ 3000 3700
NoConn ~ 3000 4100
Text Label 6100 3500 0    60   ~ 0
GND
Text Label 7600 3250 2    60   ~ 0
GND
Wire Wire Line
	8800 3850 8800 4050
Text Label 7600 3950 2    60   ~ 0
GND
Wire Wire Line
	6750 3650 6750 4500
Wire Wire Line
	6750 4500 6100 4500
Text Label 4750 4500 2    60   ~ 0
Lora_RESET
NoConn ~ 4750 3400
NoConn ~ 4750 3500
NoConn ~ 4750 3600
NoConn ~ 6100 3600
$Comp
L GND #PWR6
U 1 1 58DA903A
P 8900 4100
F 0 "#PWR6" H 8900 3850 50  0001 C CNN
F 1 "GND" H 8900 3950 50  0000 C CNN
F 2 "" H 8900 4100 50  0000 C CNN
F 3 "" H 8900 4100 50  0000 C CNN
	1    8900 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8900 4100 8900 4050
Connection ~ 8900 4050
NoConn ~ 7600 3850
Text Notes 10050 3800 0    60   ~ 0
1206
Text Notes 3750 1300 0    60   ~ 0
0805
Text Notes 3750 1600 0    60   ~ 0
0805
NoConn ~ 2700 1100
NoConn ~ 5200 2300
NoConn ~ 5300 2300
NoConn ~ 5400 2300
NoConn ~ 5500 2300
NoConn ~ 5600 2300
NoConn ~ 5700 2300
NoConn ~ 6100 4000
NoConn ~ 6100 3900
NoConn ~ 6100 3800
NoConn ~ 4750 4100
NoConn ~ 4750 4200
NoConn ~ 4750 4300
NoConn ~ 4750 4400
NoConn ~ 1700 4100
$Comp
L CONN_01X01 J1
U 1 1 58E3BD9A
P 1600 1200
F 0 "J1" H 1600 1300 50  0000 C CNN
F 1 "CONN_01X01" V 1700 1200 50  0000 C CNN
F 2 "Wire_Pads:SolderWirePad_single_SMD_5x10mm" H 1600 1200 50  0001 C CNN
F 3 "" H 1600 1200 50  0001 C CNN
	1    1600 1200
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X01 J2
U 1 1 58E3BDDD
P 1600 2400
F 0 "J2" H 1600 2500 50  0000 C CNN
F 1 "CONN_01X01" V 1700 2400 50  0000 C CNN
F 2 "Wire_Pads:SolderWirePad_single_SMD_5x10mm" H 1600 2400 50  0001 C CNN
F 3 "" H 1600 2400 50  0001 C CNN
	1    1600 2400
	1    0    0    -1  
$EndComp
Text Label 1400 1200 2    60   ~ 0
BATT_IN
$Comp
L GND #PWR1
U 1 1 58E3BE4A
P 1100 2500
F 0 "#PWR1" H 1100 2250 50  0001 C CNN
F 1 "GND" H 1100 2350 50  0000 C CNN
F 2 "" H 1100 2500 50  0000 C CNN
F 3 "" H 1100 2500 50  0000 C CNN
	1    1100 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 2400 1100 2400
Wire Wire Line
	1100 2400 1100 2500
$Comp
L CONN_01X01 J3
U 1 1 58E3C9BD
P 1600 1750
F 0 "J3" H 1600 1850 50  0000 C CNN
F 1 "CONN_01X01" V 1700 1750 50  0000 C CNN
F 2 "Wire_Pads:SolderWirePad_single_SMD_5x10mm" H 1600 1750 50  0001 C CNN
F 3 "" H 1600 1750 50  0001 C CNN
	1    1600 1750
	1    0    0    -1  
$EndComp
Text Label 1400 1750 2    60   ~ 0
BATT_IN
$EndSCHEMATC
