EESchema Schematic File Version 4
LIBS:parallax_servos-cache
EELAYER 26 0
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
L ECE3400:+5V #PWR?
U 1 1 5D81353E
P 4800 3250
F 0 "#PWR?" H 4800 3100 50  0001 C CNN
F 1 "+5V" H 4815 3423 50  0000 C CNN
F 2 "" H 4800 3250 50  0001 C CNN
F 3 "" H 4800 3250 50  0001 C CNN
	1    4800 3250
	1    0    0    -1  
$EndComp
$Comp
L ECE3400:GND #PWR?
U 1 1 5D81356E
P 4800 3550
F 0 "#PWR?" H 4800 3300 50  0001 C CNN
F 1 "GND" H 4805 3377 50  0000 C CNN
F 2 "" H 4800 3550 50  0001 C CNN
F 3 "" H 4800 3550 50  0001 C CNN
	1    4800 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 3250 4800 3300
Wire Wire Line
	4800 3300 5250 3300
Wire Wire Line
	4800 3550 4800 3500
Wire Wire Line
	4800 3500 5250 3500
Wire Wire Line
	4600 3400 5250 3400
Text Label 5250 3300 2    50   ~ 0
Servo_Vin
Text Label 5250 3400 2    50   ~ 0
Servo_Vctrl
Text Label 5250 3500 2    50   ~ 0
Servo_GND
Text Label 4600 3400 0    50   ~ 0
D5
Text Notes 7350 7500 0    50   ~ 0
Parallax Servos
Text Notes 7050 6950 0    100  ~ 0
Team 21
Text Notes 8150 7650 0    50   ~ 0
September 2019
Text Notes 10600 7650 0    50   ~ 0
-
$EndSCHEMATC
