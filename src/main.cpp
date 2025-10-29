#include <Arduino.h>

// Pin definitions for EK-TM4C1294XL LaunchPad
#define LED_RED     PN_1    // Red LED (D1)
#define LED_GREEN   PN_0    // Green LED (D2)
#define LED_BLUE    PF_0    // Blue LED (D3)
#define BUTTON_SW1  PJ_0    // SW1 button

// State variables
bool ledState = false;
bool lastButtonState = HIGH;
unsigned long lastHeartbeat = 0;
bool heartbeatState = false;

// Serial communication protocol
#define CMD_BUTTON_PRESSED  "BTN_PRESSED"
#define CMD_LED_TOGGLE      "LED_TOGGLE"
#define CMD_LED_ACK         "LED_OK"

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n========================================");
    Serial.println("   Flashlight UI - Distributed System");
    Serial.println("   Tiva (Button & LED Control)");
    Serial.println("========================================\n");

    // Initialize GPIO
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(BUTTON_SW1, INPUT_PULLUP);

    // Turn all LEDs off
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);

    // Test LEDs on startup
    Serial.println("[Setup] Testing LEDs...");
    digitalWrite(LED_RED, HIGH);
    delay(300);
    digitalWrite(LED_RED, LOW);

    digitalWrite(LED_GREEN, HIGH);
    delay(300);
    digitalWrite(LED_GREEN, LOW);

    digitalWrite(LED_BLUE, HIGH);
    delay(300);
    digitalWrite(LED_BLUE, LOW);

    Serial.println("\n[Setup] Complete!");
    Serial.println("Blue LED (D3) = Heartbeat (blinks every second)");
    Serial.println("SW1 Button = Send event to Pi4");
    Serial.println("Pi4 Commands: LED_TOGGLE to toggle Red LED (D1)\n");
}

void loop() {
    // Heartbeat - blink blue LED every 1 second
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeat >= 1000) {
        lastHeartbeat = currentTime;
        heartbeatState = !heartbeatState;
        digitalWrite(LED_BLUE, heartbeatState ? HIGH : LOW);
    }

    // Read button state
    bool currentButtonState = digitalRead(BUTTON_SW1);

    // Detect button press (active low) - send to Pi4
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        Serial.println(CMD_BUTTON_PRESSED);
        Serial.flush();  // Ensure data is sent
        delay(300);  // Debounce
    }

    // Check for incoming commands from Pi4
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command == CMD_LED_TOGGLE) {
            ledState = !ledState;
            digitalWrite(LED_RED, ledState ? HIGH : LOW);

            Serial.print("[Tiva] LED toggled. State: ");
            Serial.println(ledState ? "ON" : "OFF");
            Serial.println(CMD_LED_ACK);
            Serial.flush();
        }
    }

    lastButtonState = currentButtonState;
    delay(10);
}