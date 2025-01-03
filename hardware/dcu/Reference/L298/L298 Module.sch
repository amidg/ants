EESchema Schematic File Version 4
LIBS:L293D Module-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "L298 Motor Driver Module"
Date "2020-05-10"
Rev "V1.0"
Comp "www.ArnabKumarDas.com"
Comment1 "Open Source Hardware"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L pspice:DIODE D3
U 1 1 5EB7C085
P 7950 3250
F 0 "D3" V 7903 3381 50  0000 L CNN
F 1 "1N4007" V 7996 3381 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 7950 3250 50  0001 C CNN
F 3 "~" H 7950 3250 50  0001 C CNN
	1    7950 3250
	0    1    -1   0   
$EndComp
$Comp
L pspice:DIODE D2
U 1 1 5EB7C0E9
P 7950 2750
F 0 "D2" V 7903 2881 50  0000 L CNN
F 1 "1N4007" V 7996 2881 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 7950 2750 50  0001 C CNN
F 3 "~" H 7950 2750 50  0001 C CNN
	1    7950 2750
	0    1    -1   0   
$EndComp
$Comp
L pspice:DIODE D7
U 1 1 5EB7C344
P 9350 3250
F 0 "D7" V 9303 3381 50  0000 L CNN
F 1 "1N4007" V 9396 3381 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 9350 3250 50  0001 C CNN
F 3 "~" H 9350 3250 50  0001 C CNN
	1    9350 3250
	0    1    -1   0   
$EndComp
$Comp
L pspice:DIODE D6
U 1 1 5EB7C34A
P 9350 2750
F 0 "D6" V 9303 2881 50  0000 L CNN
F 1 "1N4007" V 9396 2881 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 9350 2750 50  0001 C CNN
F 3 "~" H 9350 2750 50  0001 C CNN
	1    9350 2750
	0    1    -1   0   
$EndComp
$Comp
L Device:C C3
U 1 1 5EB7C450
P 5850 2300
F 0 "C3" H 5965 2347 50  0000 L CNN
F 1 "100n" H 5965 2254 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 5888 2150 50  0001 C CNN
F 3 "~" H 5850 2300 50  0001 C CNN
	1    5850 2300
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7805 U1
U 1 1 5EB7C898
P 2900 3400
F 0 "U1" H 2900 3645 50  0000 C CNN
F 1 "L7805" H 2900 3552 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-3_Horizontal_TabDown" H 2925 3250 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 2900 3350 50  0001 C CNN
	1    2900 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5EB7C973
P 3700 3550
F 0 "C2" H 3821 3597 50  0000 L CNN
F 1 "47u" H 3821 3504 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 3738 3400 50  0001 C CNN
F 3 "~" H 3700 3550 50  0001 C CNN
	1    3700 3550
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 5EB7C9D7
P 2150 3550
F 0 "C1" H 2271 3597 50  0000 L CNN
F 1 "47u" H 2271 3504 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 2188 3400 50  0001 C CNN
F 3 "~" H 2150 3550 50  0001 C CNN
	1    2150 3550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J4
U 1 1 5EB7CC36
P 8700 2750
F 0 "J4" V 8665 2830 50  0000 L CNN
F 1 "MOTOR1 OUT" V 8572 2830 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 8700 2750 50  0001 C CNN
F 3 "~" H 8700 2750 50  0001 C CNN
	1    8700 2750
	0    1    -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J2
U 1 1 5EB7CE19
P 5450 3150
F 0 "J2" H 5450 2800 50  0000 C CNN
F 1 "MOTOR1 IN" H 5450 2900 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5450 3150 50  0001 C CNN
F 3 "~" H 5450 3150 50  0001 C CNN
	1    5450 3150
	-1   0    0    1   
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J1
U 1 1 5EB7D07C
P 1650 3600
F 0 "J1" H 1730 3592 50  0000 L CNN
F 1 "POWER" H 1730 3499 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 1650 3600 50  0001 C CNN
F 3 "~" H 1650 3600 50  0001 C CNN
	1    1650 3600
	-1   0    0    1   
$EndComp
$Comp
L Device:C C4
U 1 1 5EB7D0CE
P 7350 2300
F 0 "C4" H 7465 2347 50  0000 L CNN
F 1 "100n" H 7465 2254 50  0000 L CNN
F 2 "Capacitor_THT:C_Disc_D3.0mm_W2.0mm_P2.50mm" H 7388 2150 50  0001 C CNN
F 3 "~" H 7350 2300 50  0001 C CNN
	1    7350 2300
	1    0    0    -1  
$EndComp
$Comp
L Motor:Motor_DC M1
U 1 1 5EB7D717
P 8700 3150
F 0 "M1" V 8500 3300 50  0000 C CNN
F 1 "Motor_DC" V 8500 3050 50  0000 C CNN
F 2 "" H 8700 3060 50  0001 C CNN
F 3 "~" H 8700 3060 50  0001 C CNN
	1    8700 3150
	0    1    -1   0   
$EndComp
Wire Wire Line
	7950 3050 7950 2950
Wire Wire Line
	7950 2950 8400 2950
Connection ~ 7950 2950
Wire Wire Line
	8700 2950 8900 2950
Wire Wire Line
	9350 3050 9350 2950
Connection ~ 9350 2950
Wire Wire Line
	8400 3150 8400 2950
Connection ~ 8400 2950
Wire Wire Line
	8400 2950 8600 2950
Wire Wire Line
	8900 3150 8900 2950
Connection ~ 8900 2950
Wire Wire Line
	8900 2950 9350 2950
Wire Wire Line
	7950 3450 8650 3450
Connection ~ 8650 3450
Wire Wire Line
	8650 3450 9350 3450
Wire Wire Line
	7950 2550 8650 2550
Wire Wire Line
	9800 2950 9350 2950
$Comp
L pspice:DIODE D5
U 1 1 5EB7E30F
P 7950 4650
F 0 "D5" V 7903 4781 50  0000 L CNN
F 1 "1N4007" V 7996 4781 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 7950 4650 50  0001 C CNN
F 3 "~" H 7950 4650 50  0001 C CNN
	1    7950 4650
	0    1    -1   0   
$EndComp
$Comp
L pspice:DIODE D4
U 1 1 5EB7E315
P 7950 4150
F 0 "D4" V 7903 4281 50  0000 L CNN
F 1 "1N4007" V 7996 4281 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 7950 4150 50  0001 C CNN
F 3 "~" H 7950 4150 50  0001 C CNN
	1    7950 4150
	0    1    -1   0   
$EndComp
$Comp
L pspice:DIODE D9
U 1 1 5EB7E31B
P 9350 4650
F 0 "D9" V 9303 4781 50  0000 L CNN
F 1 "1N4007" V 9396 4781 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 9350 4650 50  0001 C CNN
F 3 "~" H 9350 4650 50  0001 C CNN
	1    9350 4650
	0    1    -1   0   
$EndComp
$Comp
L pspice:DIODE D8
U 1 1 5EB7E321
P 9350 4150
F 0 "D8" V 9303 4281 50  0000 L CNN
F 1 "1N4007" V 9396 4281 50  0000 L CNN
F 2 "Diode_THT:D_A-405_P7.62mm_Horizontal" H 9350 4150 50  0001 C CNN
F 3 "~" H 9350 4150 50  0001 C CNN
	1    9350 4150
	0    1    -1   0   
$EndComp
$Comp
L Connector:Screw_Terminal_01x02 J5
U 1 1 5EB7E327
P 8700 4150
F 0 "J5" V 8665 4230 50  0000 L CNN
F 1 "MOTOR2 OUT" V 8572 4230 50  0000 L CNN
F 2 "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal" H 8700 4150 50  0001 C CNN
F 3 "~" H 8700 4150 50  0001 C CNN
	1    8700 4150
	0    1    -1   0   
$EndComp
$Comp
L Motor:Motor_DC M2
U 1 1 5EB7E32D
P 8700 4550
F 0 "M2" V 8500 4700 50  0000 C CNN
F 1 "Motor_DC" V 8500 4450 50  0000 C CNN
F 2 "" H 8700 4460 50  0001 C CNN
F 3 "~" H 8700 4460 50  0001 C CNN
	1    8700 4550
	0    1    -1   0   
$EndComp
Wire Wire Line
	7950 4450 7950 4350
Wire Wire Line
	7950 4350 8400 4350
Connection ~ 7950 4350
Wire Wire Line
	8700 4350 8900 4350
Wire Wire Line
	9350 4450 9350 4350
Connection ~ 9350 4350
Wire Wire Line
	8400 4550 8400 4350
Connection ~ 8400 4350
Wire Wire Line
	8400 4350 8600 4350
Wire Wire Line
	8900 4550 8900 4350
Connection ~ 8900 4350
Wire Wire Line
	8900 4350 9350 4350
$Comp
L power:GND #PWR013
U 1 1 5EB7E340
P 8650 4850
F 0 "#PWR013" H 8650 4600 50  0001 C CNN
F 1 "GND" H 8655 4673 50  0000 C CNN
F 2 "" H 8650 4850 50  0001 C CNN
F 3 "" H 8650 4850 50  0001 C CNN
	1    8650 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 3950 8650 3950
Wire Wire Line
	7750 5100 9800 5100
Wire Wire Line
	9800 4350 9350 4350
Wire Wire Line
	7400 3750 7400 4550
Wire Wire Line
	7400 4550 7750 4550
$Comp
L power:GND #PWR08
U 1 1 5EB818E9
P 6650 4700
F 0 "#PWR08" H 6650 4450 50  0001 C CNN
F 1 "GND" H 6655 4523 50  0000 C CNN
F 2 "" H 6650 4700 50  0001 C CNN
F 3 "" H 6650 4700 50  0001 C CNN
	1    6650 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6650 4250 6650 4700
$Comp
L Connector_Generic:Conn_01x03 J3
U 1 1 5EB861D1
P 5450 3550
F 0 "J3" H 5450 3900 50  0000 C CNN
F 1 "MOTOR2 IN" H 5450 3800 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 5450 3550 50  0001 C CNN
F 3 "~" H 5450 3550 50  0001 C CNN
	1    5450 3550
	-1   0    0    1   
$EndComp
Wire Wire Line
	7350 2150 6600 2150
$Comp
L power:GND #PWR07
U 1 1 5EB9A6D2
P 6600 2150
F 0 "#PWR07" H 6600 1900 50  0001 C CNN
F 1 "GND" H 6605 1973 50  0000 C CNN
F 2 "" H 6600 2150 50  0001 C CNN
F 3 "" H 6600 2150 50  0001 C CNN
	1    6600 2150
	1    0    0    -1  
$EndComp
Connection ~ 6600 2150
Wire Wire Line
	6600 2150 5850 2150
Wire Wire Line
	1850 3500 1850 3400
Wire Wire Line
	1850 3400 2150 3400
Wire Wire Line
	2150 3400 2600 3400
Connection ~ 2150 3400
Wire Wire Line
	1850 3600 1850 3700
Wire Wire Line
	1850 3700 2150 3700
Wire Wire Line
	2150 3700 2900 3700
Connection ~ 2150 3700
Wire Wire Line
	2900 3700 3700 3700
Connection ~ 2900 3700
Wire Wire Line
	3200 3400 3700 3400
$Comp
L Device:LED D1
U 1 1 5EBB37EA
P 4800 3700
F 0 "D1" V 4839 3580 50  0000 R CNN
F 1 "LED" V 4746 3580 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm" H 4800 3700 50  0001 C CNN
F 3 "~" H 4800 3700 50  0001 C CNN
	1    4800 3700
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5EBB7A92
P 4800 3300
F 0 "R1" H 4870 3347 50  0000 L CNN
F 1 "R" H 4870 3254 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 4730 3300 50  0001 C CNN
F 3 "~" H 4800 3300 50  0001 C CNN
	1    4800 3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 5EBB7B5E
P 4150 3700
F 0 "#PWR03" H 4150 3450 50  0001 C CNN
F 1 "GND" H 4155 3523 50  0000 C CNN
F 2 "" H 4150 3700 50  0001 C CNN
F 3 "" H 4150 3700 50  0001 C CNN
	1    4150 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3400 4150 3400
Connection ~ 3700 3400
Wire Wire Line
	3700 3700 4150 3700
Connection ~ 3700 3700
$Comp
L power:+5V #PWR04
U 1 1 5EBBC55C
P 4800 3050
F 0 "#PWR04" H 4800 2900 50  0001 C CNN
F 1 "+5V" H 4815 3227 50  0000 C CNN
F 2 "" H 4800 3050 50  0001 C CNN
F 3 "" H 4800 3050 50  0001 C CNN
	1    4800 3050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5EBBC59C
P 4800 3950
F 0 "#PWR05" H 4800 3700 50  0001 C CNN
F 1 "GND" H 4805 3773 50  0000 C CNN
F 2 "" H 4800 3950 50  0001 C CNN
F 3 "" H 4800 3950 50  0001 C CNN
	1    4800 3950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 5EBBC5D5
P 4150 3400
F 0 "#PWR02" H 4150 3250 50  0001 C CNN
F 1 "+5V" H 4165 3577 50  0000 C CNN
F 2 "" H 4150 3400 50  0001 C CNN
F 3 "" H 4150 3400 50  0001 C CNN
	1    4150 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3050 4800 3150
Wire Wire Line
	4800 3450 4800 3550
Wire Wire Line
	4800 3850 4800 3950
$Comp
L power:+5V #PWR06
U 1 1 5EBC5990
P 5450 2450
F 0 "#PWR06" H 5450 2300 50  0001 C CNN
F 1 "+5V" H 5465 2627 50  0000 C CNN
F 2 "" H 5450 2450 50  0001 C CNN
F 3 "" H 5450 2450 50  0001 C CNN
	1    5450 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2450 5850 2450
Connection ~ 5850 2450
$Comp
L power:+BATT #PWR01
U 1 1 5EBC800E
P 2150 3400
F 0 "#PWR01" H 2150 3250 50  0001 C CNN
F 1 "+BATT" H 2165 3577 50  0000 C CNN
F 2 "" H 2150 3400 50  0001 C CNN
F 3 "" H 2150 3400 50  0001 C CNN
	1    2150 3400
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR09
U 1 1 5EBC80C1
P 7800 2450
F 0 "#PWR09" H 7800 2300 50  0001 C CNN
F 1 "+BATT" H 7815 2627 50  0000 C CNN
F 2 "" H 7800 2450 50  0001 C CNN
F 3 "" H 7800 2450 50  0001 C CNN
	1    7800 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 2450 7800 2450
Connection ~ 7350 2450
$Comp
L power:+BATT #PWR010
U 1 1 5EBCFE16
P 8650 2550
F 0 "#PWR010" H 8650 2400 50  0001 C CNN
F 1 "+BATT" H 8665 2727 50  0000 C CNN
F 2 "" H 8650 2550 50  0001 C CNN
F 3 "" H 8650 2550 50  0001 C CNN
	1    8650 2550
	1    0    0    -1  
$EndComp
Connection ~ 8650 2550
Wire Wire Line
	8650 2550 9350 2550
$Comp
L power:+BATT #PWR012
U 1 1 5EBCFE7F
P 8650 3950
F 0 "#PWR012" H 8650 3800 50  0001 C CNN
F 1 "+BATT" H 8665 4127 50  0000 C CNN
F 2 "" H 8650 3950 50  0001 C CNN
F 3 "" H 8650 3950 50  0001 C CNN
	1    8650 3950
	1    0    0    -1  
$EndComp
Connection ~ 8650 3950
Wire Wire Line
	8650 3950 9350 3950
$Comp
L power:GND #PWR011
U 1 1 5EBD002A
P 8650 3450
F 0 "#PWR011" H 8650 3200 50  0001 C CNN
F 1 "GND" H 8655 3273 50  0000 C CNN
F 2 "" H 8650 3450 50  0001 C CNN
F 3 "" H 8650 3450 50  0001 C CNN
	1    8650 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	7750 3700 9800 3700
Wire Wire Line
	9800 2950 9800 3700
Wire Wire Line
	7950 4850 8650 4850
Connection ~ 8650 4850
Wire Wire Line
	8650 4850 9350 4850
Wire Wire Line
	9800 4350 9800 5100
Wire Wire Line
	7750 4550 7750 5100
$Comp
L Driver_Motor:L298HN U2
U 1 1 5EB8507F
P 6650 3550
F 0 "U2" H 6650 3800 50  0000 C CNN
F 1 "L298HN" H 6650 3700 50  0000 C CNN
F 2 "Package_TO_SOT_THT:TO-220-15_P2.54x2.54mm_StaggerOdd_Lead4.58mm_Vertical" H 6700 2900 50  0001 L CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00000240.pdf" H 6800 3800 50  0001 C CNN
	1    6650 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7250 3750 7400 3750
Wire Wire Line
	6650 2850 6650 2450
Wire Wire Line
	6750 2850 6750 2450
Wire Wire Line
	6750 2450 7350 2450
Wire Wire Line
	5850 2450 6650 2450
Wire Wire Line
	7250 3650 7550 3650
Wire Wire Line
	7550 3650 7550 4350
Wire Wire Line
	7550 4350 7950 4350
Wire Wire Line
	7250 3450 7750 3450
Wire Wire Line
	7750 3450 7750 3700
Wire Wire Line
	7250 3350 7750 3350
Wire Wire Line
	7750 3350 7750 2950
Wire Wire Line
	7750 2950 7950 2950
Wire Wire Line
	5650 3050 6050 3050
Wire Wire Line
	6050 3150 5650 3150
Wire Wire Line
	5650 3250 6050 3250
Wire Wire Line
	6050 3650 5650 3650
Wire Wire Line
	5650 3550 6050 3550
Wire Wire Line
	6050 3450 5650 3450
$Comp
L Connector_Generic:Conn_01x02 J6
U 1 1 5EB9FDB2
P 6450 4600
F 0 "J6" V 6350 4250 50  0000 L CNN
F 1 "SENSE" V 6450 4150 50  0000 L CNN
F 2 "" H 6450 4600 50  0001 C CNN
F 3 "~" H 6450 4600 50  0001 C CNN
	1    6450 4600
	0    1    1    0   
$EndComp
Wire Wire Line
	6450 4400 6450 4250
Wire Wire Line
	6350 4400 6350 4250
$EndSCHEMATC
