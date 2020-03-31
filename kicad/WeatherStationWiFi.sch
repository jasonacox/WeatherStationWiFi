EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "ESP8266 NodeMCU Based Solar Powered WiFi Weather Station"
Date "25 March 2020"
Rev "1"
Comp "Circuit Diagram"
Comment1 "https://github.com/jasonacox/WeatherStationWiFi"
Comment2 "Author: Jason A. Cox"
Comment3 ""
Comment4 "ESP8266 NodeMCU Based Solar Powered WiFi Weather Station"
$EndDescr
Wire Wire Line
	5650 3450 5400 3450
Wire Wire Line
	6250 3450 6350 3450
$Comp
L ESP8266:NodeMCU_1.0_(ESP-12E) U1
U 1 1 5E4A3029
P 4600 3450
F 0 "U1" H 4600 4537 60  0000 C CNN
F 1 "ESP8266 NodeMCU" H 4600 4431 60  0000 C CNB
F 2 "" H 4000 2600 60  0000 C CNN
F 3 "" H 4000 2600 60  0000 C CNN
	1    4600 3450
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R5
U 1 1 5E4FFD1C
P 5800 3450
F 0 "R5" V 6005 3450 50  0000 C CNN
F 1 "440" V 5914 3450 50  0000 C CNN
F 2 "" V 5840 3440 50  0001 C CNN
F 3 "~" H 5800 3450 50  0001 C CNN
	1    5800 3450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_US R3
U 1 1 5E4E56A9
P 6900 1500
F 0 "R3" V 6695 1500 50  0000 C TNN
F 1 "1K" V 6786 1500 50  0000 C TNN
F 2 "" V 6940 1490 50  0001 C CNN
F 3 "~" H 6900 1500 50  0001 C CNN
	1    6900 1500
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 E1-RainDetect
U 1 1 5E4BCB5E
P 6550 1400
F 0 "E1-RainDetect" H 6468 1617 50  0000 C CNN
F 1 "Water Electrode" H 6468 1526 50  0000 C CNN
F 2 "" H 6550 1400 50  0001 C CNN
F 3 "~" H 6550 1400 50  0001 C CNN
	1    6550 1400
	-1   0    0    -1  
$EndComp
$Comp
L Device:Solar_Cell SC1
U 1 1 5E80B496
P 2450 5750
F 0 "SC1" H 2558 5846 50  0000 L CNN
F 1 "5V_Solar_Cell" H 2558 5755 50  0000 L CNN
F 2 "" V 2450 5810 50  0001 C CNN
F 3 "~" V 2450 5810 50  0001 C CNN
	1    2450 5750
	1    0    0    -1  
$EndComp
$Comp
L tp4056:TP4056 PWR1
U 1 1 5E80F11A
P 3600 5750
F 0 "PWR1" H 3600 6287 60  0000 C CNN
F 1 "TP4056" H 3600 6181 60  0000 C CNN
F 2 "" H 3600 5750 60  0001 C CNN
F 3 "" H 3600 5750 60  0001 C CNN
	1    3600 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5550 2450 5500
Wire Wire Line
	2450 5500 3100 5500
Wire Wire Line
	2450 5850 2450 5900
Wire Wire Line
	2450 5900 3100 5900
$Comp
L Device:Battery_Cell BT1
U 1 1 5E81579B
P 4300 5750
F 0 "BT1" H 4418 5846 50  0000 L CNN
F 1 "3.7V Li-ion Battery" H 4418 5755 50  0000 L CNN
F 2 "" V 4300 5810 50  0001 C CNN
F 3 "~" V 4300 5810 50  0001 C CNN
	1    4300 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 5600 4200 5600
Wire Wire Line
	4200 5600 4200 5550
Wire Wire Line
	4200 5550 4300 5550
Wire Wire Line
	4100 5800 4200 5800
Wire Wire Line
	4200 5800 4200 5850
Wire Wire Line
	4200 5850 4300 5850
Wire Wire Line
	7050 5900 7050 5150
Connection ~ 7050 4050
$Comp
L Regulator_Linear:MCP1700-3302E_TO92 U2
U 1 1 5E82D5F4
P 5800 5150
F 0 "U2" V 5846 5045 50  0000 R CNN
F 1 "MCP1700-3302E_TO92" V 5755 5045 50  0000 R CNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 5800 4950 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001826D.pdf" H 5800 5150 50  0001 C CNN
	1    5800 5150
	0    1    -1   0   
$EndComp
$Comp
L Device:C C2
U 1 1 5E83D991
P 5350 5700
F 0 "C2" H 5465 5746 50  0000 L CNN
F 1 "0.1uF" H 5465 5655 50  0000 L CNN
F 2 "" H 5388 5550 50  0001 C CNN
F 3 "~" H 5350 5700 50  0001 C CNN
	1    5350 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5800 5500 5800 5450
Wire Wire Line
	5350 5500 5350 5550
Wire Wire Line
	5350 5500 5800 5500
Wire Wire Line
	5800 5500 5800 5550
Connection ~ 5350 5500
Connection ~ 5800 5500
Wire Wire Line
	4100 5900 5350 5900
Wire Wire Line
	5350 5900 5350 5850
Wire Wire Line
	5350 5900 5800 5900
Wire Wire Line
	5800 5900 5800 5850
Connection ~ 5350 5900
Connection ~ 5800 5900
Wire Wire Line
	5800 5900 7050 5900
$Comp
L Device:CP1 C3
U 1 1 5E839DCD
P 5800 5700
F 0 "C3" H 5686 5654 50  0000 R CNN
F 1 "100uF" H 5686 5745 50  0000 R CNN
F 2 "" H 5800 5700 50  0001 C CNN
F 3 "~" H 5800 5700 50  0001 C CNN
	1    5800 5700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5800 4850 5800 4150
Wire Wire Line
	5800 4150 5400 4150
Wire Wire Line
	6100 5150 7050 5150
Connection ~ 7050 5150
Wire Wire Line
	7050 5150 7050 4050
Wire Wire Line
	6950 3550 6950 3450
Wire Wire Line
	5400 2950 5700 2950
Wire Wire Line
	5700 2950 5700 2450
Wire Wire Line
	5400 2850 5550 2850
Wire Wire Line
	5550 2850 5550 2300
Wire Wire Line
	5550 2300 5700 2300
Wire Wire Line
	3800 3650 3400 3650
Wire Wire Line
	3400 3650 3400 2150
Wire Wire Line
	3400 2150 3800 2150
Wire Wire Line
	5400 3350 5650 3350
Wire Wire Line
	5650 3350 5650 3150
Wire Wire Line
	5650 3150 6350 3150
Wire Wire Line
	6350 3150 6350 3450
Wire Wire Line
	5400 4050 7050 4050
Wire Wire Line
	6350 3150 6650 3150
Connection ~ 6350 3150
Wire Wire Line
	7400 3150 7400 1750
Wire Wire Line
	7400 1750 5550 1750
Wire Wire Line
	5550 1750 5550 2000
Wire Wire Line
	5550 2000 5700 2000
$Comp
L Device:CP1 C1
U 1 1 5E4AEB85
P 7400 1250
F 0 "C1" H 7286 1204 50  0000 R CNN
F 1 "220uF" H 7286 1295 50  0000 R CNN
F 2 "" H 7400 1250 50  0001 C CNN
F 3 "~" H 7400 1250 50  0001 C CNN
	1    7400 1250
	1    0    0    1   
$EndComp
$Comp
L Transistor_BJT:2N2219 Q1
U 1 1 5E4A8EE0
P 7800 1400
F 0 "Q1" H 7990 1446 50  0000 L CNN
F 1 "2N2222" H 7990 1355 50  0000 L CNN
F 2 "Package_TO_SOT_THT:TO-39-3" H 8000 1325 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/2N2219-D.PDF" H 7800 1400 50  0001 L CNN
	1    7800 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4050 8400 4050
Wire Wire Line
	7900 1200 7900 1100
Wire Wire Line
	7600 1400 7400 1400
Wire Wire Line
	5400 3650 7900 3650
Connection ~ 7400 1750
$Comp
L Device:R_US R4
U 1 1 5E4C5B97
P 7400 3350
F 0 "R4" V 7605 3350 50  0000 C CNN
F 1 "5.1K" V 7514 3350 50  0000 C CNN
F 2 "" V 7440 3340 50  0001 C CNN
F 3 "~" H 7400 3350 50  0001 C CNN
	1    7400 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 3200 7400 3150
Connection ~ 7400 3150
Text Notes 7100 4650 0    39   ~ 8
GND
Text Notes 5600 4650 0    39   ~ 8
3.3V
Text Notes 2800 5450 0    39   ~ 8
0-5V
Text Notes 2800 6000 0    39   ~ 8
GND
Wire Wire Line
	7400 3500 7400 3550
Wire Wire Line
	7400 3550 6950 3550
Wire Wire Line
	7250 3150 7400 3150
$Comp
L Device:LED D1
U 1 1 5E4A7CE1
P 6100 3450
F 0 "D1" H 6093 3666 50  0000 C CNN
F 1 "LED" H 6093 3575 50  0000 C CNN
F 2 "" H 6100 3450 50  0001 C CNN
F 3 "~" H 6100 3450 50  0001 C CNN
	1    6100 3450
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7400 1500 7400 1750
Wire Wire Line
	7050 1500 7400 1500
Wire Wire Line
	6750 1400 7400 1400
Connection ~ 7400 1400
Wire Wire Line
	7900 1600 7900 3650
Wire Wire Line
	7400 1100 7900 1100
Wire Wire Line
	7900 1100 8400 1100
Wire Wire Line
	8400 1100 8400 4050
Connection ~ 7900 1100
Wire Notes Line
	5350 3550 5400 3550
Wire Wire Line
	6950 3550 5400 3550
Connection ~ 6950 3550
Wire Wire Line
	3800 3750 3300 3750
Connection ~ 5550 2000
Wire Wire Line
	3300 2000 5550 2000
Wire Wire Line
	3300 3750 3300 2000
$Comp
L Sensor:BME_BMP280 U3
U 1 1 5E864BAC
P 6300 2250
F 0 "U3" H 6730 2387 50  0000 L CNN
F 1 "BME_BMP280" H 6730 2296 50  0000 L CNN
F 2 "Package_LGA:Bosch_LGA-8_2.5x2.5mm_P0.65mm_ClockwisePinNumbering" H 6250 1750 50  0001 C CNN
F 3 "https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280-DS002.pdf" H 6300 2050 50  0001 C CNN
F 4 "Pressure &" H 6730 2205 50  0000 L CNN "Sensor"
F 5 "Humidity Sensor" H 6730 2114 50  0000 L CNN "Sensor2"
	1    6300 2250
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 5E94E5D2
P 3800 2400
F 0 "R2" V 3595 2400 50  0000 C TNN
F 1 "100K" V 3686 2400 50  0000 C TNN
F 2 "" V 3840 2390 50  0001 C CNN
F 3 "~" H 3800 2400 50  0001 C CNN
	1    3800 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 3750 4500 3750
Wire Wire Line
	4500 3750 4500 4150
Wire Wire Line
	4500 4150 5400 4150
Connection ~ 3800 3750
Connection ~ 5400 4150
Wire Wire Line
	5400 3350 4650 3350
Wire Wire Line
	4650 3350 4650 3650
Wire Wire Line
	4650 4050 5400 4050
Connection ~ 5400 3350
Connection ~ 5400 4050
Wire Wire Line
	3800 3650 4650 3650
Connection ~ 3800 3650
Connection ~ 4650 3650
Wire Wire Line
	4650 3650 4650 4050
$Comp
L Device:R_US R1
U 1 1 5E961855
P 4500 5100
F 0 "R1" V 4295 5100 50  0000 C TNN
F 1 "51K" V 4386 5100 50  0000 C TNN
F 2 "" V 4540 5090 50  0001 C CNN
F 3 "~" H 4500 5100 50  0001 C CNN
	1    4500 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 4700 3600 2750
Wire Wire Line
	3600 2750 3800 2750
Wire Wire Line
	3800 2550 3800 2750
Connection ~ 3800 2750
Wire Wire Line
	3800 2250 3800 2150
Connection ~ 3800 2150
Wire Wire Line
	3800 2150 5700 2150
Wire Wire Line
	4100 5500 4500 5500
Wire Wire Line
	3600 4700 4500 4700
Wire Wire Line
	4500 5250 4500 5500
Connection ~ 4500 5500
Wire Wire Line
	4500 4950 4500 4700
Wire Wire Line
	4500 5500 5350 5500
Text Notes 3800 4650 0    39   ~ 0
Voltage Sensor
$Comp
L Sensor_Temperature:DS18B20 One-Wire
U 1 1 5E4AD1F4
P 6950 3150
F 0 "One-Wire" V 6562 3150 50  0000 C TNN
F 1 "DS18B20" V 6641 3150 50  0000 C TNN
F 2 "Package_TO_SOT_THT:TO-92_Inline" H 5950 2900 50  0001 C CNN
F 3 "http://datasheets.maximintegrated.com/en/ds/DS18B20.pdf" H 6800 3400 50  0001 C CNN
	1    6950 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R_US R2
U 1 1 5E817560
P 9600 5200
F 0 "R2" V 9395 5200 50  0000 C TNN
F 1 "51K" V 9486 5200 50  0000 C TNN
F 2 "" V 9640 5190 50  0001 C CNN
F 3 "~" H 9600 5200 50  0001 C CNN
	1    9600 5200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 5E819D3A
P 9600 5500
F 0 "R1" V 9395 5500 50  0000 C TNN
F 1 "100K" V 9486 5500 50  0000 C TNN
F 2 "" V 9640 5490 50  0001 C CNN
F 3 "~" H 9600 5500 50  0001 C CNN
	1    9600 5500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E81E950
P 9600 5650
F 0 "#PWR?" H 9600 5400 50  0001 C CNN
F 1 "GND" H 9605 5477 50  0000 C CNN
F 2 "" H 9600 5650 50  0001 C CNN
F 3 "" H 9600 5650 50  0001 C CNN
	1    9600 5650
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5E81F47D
P 9600 5000
F 0 "#PWR?" H 9600 4850 50  0001 C CNN
F 1 "+5V" H 9615 5173 50  0000 C CNN
F 2 "" H 9600 5000 50  0001 C CNN
F 3 "" H 9600 5000 50  0001 C CNN
	1    9600 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 5350 10150 5350
Connection ~ 9600 5350
Text Notes 10450 5650 1    50   ~ 0
A0 - Analog Input
Wire Wire Line
	9600 5050 9600 5000
Text Notes 9300 4750 0    50   ~ 0
Vin (from OUT+)
Text Notes 10050 5300 0    50   ~ 0
Vout
Text Notes 10350 5500 1    50   ~ 0
ESP8266
Wire Notes Line
	8650 4350 10700 4350
Wire Notes Line
	10700 4350 10700 6000
Wire Notes Line
	10700 6000 8650 6000
Wire Notes Line
	8650 6000 8650 4350
Text Notes 9200 4500 0    50   ~ 0
Voltage Divider Circuit
$EndSCHEMATC
