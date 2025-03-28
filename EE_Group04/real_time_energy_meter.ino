#include <LiquidCrystal_I2C.h>

// LCD Configuration (I2C address 0x20, 16 columns, 2 rows)
LiquidCrystal_I2C lcd(0x20, 16, 2);

// Pin Definitions
int motor1 = 9;      // Motor control pin 1
int motor2 = 11;     // Motor control pin 2
const int buttonPin = 2;  // Button pin for toggling motor

// Motor and Button States
bool motorState = false;          // Tracks if motor is ON/OFF
bool lastButtonState = LOW;        // Previous button state (for debouncing)
bool currentButtonState;           // Current button state
unsigned long lastDebounceTime = 0; // Debounce timer
const int debounceDelay = 50;      // Debounce delay (ms)

// Energy Calculation Variables
unsigned long previousMillis = 0;  // Time tracking for energy calculation
float totalEnergy = 0.0;           // Cumulative energy in Watt-seconds (Ws)

void setup() {
    // Initialize pins
    pinMode(buttonPin, INPUT);
    pinMode(motor1, OUTPUT);
    pinMode(motor2, OUTPUT);
    pinMode(A1, INPUT);  // Voltage measurement pin
  	pinMode(A0, INPUT);
    
    Serial.begin(9600);  // Start serial communication
    previousMillis = millis();  // Record initial time
    
    // LCD Setup
    lcd.init();         // Initialize LCD
    lcd.clear();        // Clear display
    lcd.backlight();    // Turn on backlight
}

void loop() {
    // Read button state with debouncing
    currentButtonState = digitalRead(buttonPin);
    
    // Toggle motor on button press (with debounce)
    if (currentButtonState == HIGH && lastButtonState == LOW) {
        delay(debounceDelay);
        if (digitalRead(buttonPin) == HIGH) {
            motorState = !motorState;  // Toggle motor state
            if (motorState) {
                digitalWrite(motor1, HIGH);  // Turn motor ON
                digitalWrite(motor2, LOW);
            } else {
                digitalWrite(motor1, LOW);   // Turn motor OFF
                digitalWrite(motor2, LOW);
            }
        }
    }
    lastButtonState = currentButtonState;
    
    // Calculate elapsed time (in seconds)
    unsigned long currentMillis = millis();
    float elapsedTime = (currentMillis - previousMillis) / 1000.0;
    previousMillis = currentMillis;
    
    // --- Voltage Measurement ---
    // Scale analog reading (0-1023) to 0-30V range
    float voltage = analogRead(A1) * (30.0 / 1023.0);
    
    // --- Current Measurement ---
    // Read voltages across shunt resistor (simulated in Tinkercad)
    int v1 = analogRead(A0);  // "Shunt" voltage (A0)
    int v2 = analogRead(A1);  // Reference voltage (A1)
    float voltage1 = v1 * (5.0 / 1023.0);  // Convert to actual voltage
    float voltage2 = v2 * (5.0 / 1023.0);
    float shuntVoltage = voltage2 - voltage1;  // Voltage drop across shunt
    float shuntResistance = 10.0;              // Shunt resistor value (ohms)
    float motorCurrent = shuntVoltage / shuntResistance;  // I = V/R
    
    // --- Power and Energy Calculations ---
    float motorPower = voltage * motorCurrent;          // P = V*I
    totalEnergy += motorPower * elapsedTime;          // Energy = P * t
    
    // Serial Monitor Output (for debugging)
    Serial.print("Measured Voltage: "); Serial.print(voltage, 4); Serial.println(" V");
    Serial.print("Estimated motor Current: "); Serial.print(motorCurrent * 1000); Serial.println(" mA");
    Serial.print("Power consumed by motor: "); Serial.print(motorPower, 4); Serial.println(" W");
    Serial.print("Total Energy Consumed: "); Serial.print(totalEnergy, 4); Serial.println(" Ws");
    
    // LCD Output (shows only total energy)
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("E=");
    lcd.print(totalEnergy, 2);  // Display energy with 2 decimal places
    lcd.print(" Ws");
    
    delay(500);  // Short delay to stabilize readings
}