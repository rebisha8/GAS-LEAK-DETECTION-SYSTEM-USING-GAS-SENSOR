# GAS-LEAK-DETECTION-SYSTEM-USING-GAS-SENSOR

## Aim:
	To measure the air quality using Gas Sensor  MQ-2 with Arduino UNO Board/ESP-32 using Tinker CAD.

## Hardware / Software Tools required:
	PC/ Laptop with Internet connection
  Tinker CAD tool (Online)
	Arduino UNO Board/ESP-32
  Gas sensor (MQ-2)
	
## Circuit Diagram:

<img width="1036" height="597" alt="image" src="https://github.com/user-attachments/assets/2d6cca6e-b829-4701-8cd2-149756b3f652" />

## Theory :
 The Arduino Uno is powered by the ATmega328P, an 8-bit microcontroller that runs at 16 MHz. It has 32 KB of flash memory, 2 KB of SRAM, and 1 KB of EEPROM. The board 
has 14 digital I/O pins (of which 6 can be used as PWM outputs) and 6 analog input pins. These pins allow the board to interface with various sensors, actuators, and other devices.
The Arduino Uno can be powered via a USB connection or an external power supply. The board has a built-in voltage regulator to manage power from 7 to 12 volts.
The board is programmable using the Arduino IDE (Integrated Development Environment), which supports a simplified version of C/C++. The code, known as a "sketch," is uploaded to the board via a USB connection. The Uno has a USB-B port, which is used for communication with a computer. The USB connection also powers the board when connected. The board includes a reset button that restarts the microcontroller, useful during programming and troubleshooting. The In-Circuit Serial Programming (ICSP) header allows for low-level programming of the microcontroller or firmware updates. The Uno has a built-in LED on pin 13, commonly used for simple tests and debugging.
Procedure:
Step 1: Set Up the Tinkercad Environment
3.	Log in to Tinkercad: Open Tinkercad in your web browser and log in to your account.
4.	Create a New Circuit: In the Tinkercad dashboard, click on "Circuits" and then select "Create New Circuit."
Step 2: Add Components to the Circuit
6.	Arduino Uno: Drag an Arduino Uno board from the components panel onto the workspace.
7.	Gas Sensor: Search for the TMP36 sensor in the components panel and drag it into the workspace.
8.	Breadboard: Drag a small breadboard to the workspace to help with wiring connections.
9.	Resistor (Optional): A resistor may not be necessary for this simple setup, but you can include it for more accurate readings.
10.	Wires: Use wires to connect the components.
Step 3: Connect the MQ-2 Gas Sensor to Arduino:
•	MQ-2 Pins:
o	VCC: Connect this pin to the Arduino 5V pin.
o	GND: Connect this pin to the Arduino GND pin.
o	Analog Output (A0): Connect this pin to an analog input pin on the Arduino (e.g., A0).
o	Digital Output (D0): (Optional) Connect this pin to a digital input pin on the Arduino if you want to use the sensor’s digital threshold output.
Breadboard Wiring:
•	MQ-2 VCC to Arduino 5V: Use a wire to connect the VCC pin of the MQ-2 sensor to the 5V rail on the breadboard (which is connected to Arduino 5V).
•	MQ-2 GND to Arduino GND: Connect the GND pin of the MQ-2 sensor to the ground rail on the breadboard (connected to Arduino GND).
•	MQ-2 Analog Output (A0) to Arduino Analog Pin (A0): Connect the analog output pin of the sensor to Arduino’s A0 pin.
•	(Optional) MQ-2 Digital Output (D0) to Arduino Digital Pin (e.g., D2): Connect the digital output pin if you want to detect gas concentration threshold digitally.
Step 4: Write the Arduino Code
•	Code Editor: Click on the "Code" button at the top of the Tinkercad workspace to open the code editor.
•	Set the Coding Mode: Ensure the editor is in "Text" mode to write your code in C/C++.
•	Enter the Code: Write the following code from the MQ-2  sensor
Step 5: Simulate the Circuit
•	Start Simulation: Click the "Start Simulation" button at the top of the workspace to run the circuit and code.
•	Monitor Output: Open the serial monitor by clicking the "Serial Monitor" button to view the temperature readings in both Celsius and Fahrenheit.
Step 6: Troubleshoot and Refine
•	Check Connections: Ensure that all connections are made correctly on the breadboard and the Arduino.
•	Adjust Code: If needed, tweak the code to improve accuracy or change the format of the output.
Step 7: Save Your Work
•	Stop Simulation: Click "Stop Simulation" to end the simulation.
•	Save the Circuit: Click "Save" to keep your circuit design and code for future use.

## Program:

#include <LiquidCrystal.h>
LiquidCrystal lcd(5,6,8,9,10,11);
  
int redled = 2;
int greenled = 3;
int buzzer = 4;
int sensor = A0;
int sensorThresh = 400;

void setup()
{
pinMode(redled, OUTPUT);
pinMode(greenled,OUTPUT);
pinMode(buzzer,OUTPUT);
pinMode(sensor,INPUT);
Serial.begin(9600);
lcd.begin(16,2);

}

void loop()
{
   
  int analogValue = analogRead(sensor);
  Serial.print(analogValue);
  if(analogValue>sensorThresh)
  {
    digitalWrite(redled,HIGH);
    digitalWrite(greenled,LOW);
    tone(buzzer,1000,10000);
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("ALERT");
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("RUN REBISHA RUN");
    delay(1000);
  }
  else
  {
    digitalWrite(greenled,HIGH);
    digitalWrite(redled,LOW);
    noTone(buzzer);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  Supervised By");
    lcd.setCursor(0,1);
    lcd.print(" Rebisha");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Hello");
    delay(1000);
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print("Rebisha");
    delay(1000);
  }  
     
}
## Output:

https://github.com/user-attachments/assets/ab5a22ba-8352-4471-b964-4e5ec0c4c94b

## Result:
GAS-LEAK-DETECTION-SYSTEM-USING-GAS-SENSOR using tinkercad is executed successfully.
