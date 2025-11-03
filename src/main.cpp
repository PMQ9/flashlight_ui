#include <Arduino.h>

// Pin definitions for EK-TM4C1294XL LaunchPad
#define LED_RED     PN_1    // Red LED (D1) - Built-in
#define LED_GREEN   PN_0    // Green LED (D2) - Built-in
#define LED_BLUE    PF_0    // Blue LED (D3) - Built-in
#define LED_EXTERNAL PE_4   // External LED (user-connected)
#define BUTTON_SW1  PJ_0    // SW1 button

// Brightness levels (5 levels: 25%, 50%, 75%, 100%, then cycle back)
#define BRIGHTNESS_LEVELS 5
int brightnessLevels[BRIGHTNESS_LEVELS] = {64, 127, 191, 255};  // 25%, 50%, 75%, 100%
int currentLevel = 0;  // Current brightness level (0-3)

// State variables
bool ledState = false;  // Is LED on or off?
unsigned long buttonPressTime = 0;  // When button was pressed
bool buttonHeld = false;  // Is button being held?
unsigned long lastHeartbeat = 0;
bool lastButtonState = HIGH;

// Serial communication protocol
#define CMD_BUTTON_PRESSED  "BTN_PRESSED"
#define CMD_LED_TOGGLE      "LED_TOGGLE"
#define CMD_LED_ACK         "LED_OK"
#define CMD_BRIGHTNESS      "BRIGHTNESS="  // Format: BRIGHTNESS=0-255

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
    pinMode(LED_EXTERNAL, OUTPUT);
    pinMode(BUTTON_SW1, INPUT_PULLUP);

    // Setup PWM for red LED on PN_1 and external LED on PE_4 (both support PWM)
    // Note: TM4C1294 supports PWM on most GPIO pins
    analogWrite(LED_RED, 0);  // Start off
    analogWrite(LED_EXTERNAL, 0);  // Start off

    // Turn all LEDs off
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_EXTERNAL, LOW);

    // Double flash blue LED to show startup
    Serial.println("[Setup] Heartbeat double-flash (startup indicator)...");
    digitalWrite(LED_BLUE, HIGH);
    delay(200);
    digitalWrite(LED_BLUE, LOW);
    delay(150);
    digitalWrite(LED_BLUE, HIGH);
    delay(200);
    digitalWrite(LED_BLUE, LOW);

    Serial.println("\n[Setup] Complete!");
    Serial.println("Blue LED (D3) = Heartbeat (double-flash every 10 seconds)");
    Serial.println("Red LED (D1) = Flashlight with 4 brightness levels (built-in)");
    Serial.println("External LED (PE_4) = Mirrors red LED brightness (user-connected)");
    Serial.println("\nButton Controls:");
    Serial.println("  OFF state:     1 click = Turn ON at Level 1 (25%)");
    Serial.println("  ON state:      1 click = Turn OFF");
    Serial.println("  Any ON state:  Hold button (500ms+) = Cycle through brightness");
    Serial.println("                 Slow cycling (500ms per level)");
    Serial.println("                 Wraps from Level 4 back to Level 1");
    Serial.println("                 Release button = Stop at current brightness");
    Serial.println("\nBrightness Levels:");
    Serial.println("  Level 1: 25%  (64/255)");
    Serial.println("  Level 2: 50%  (127/255)");
    Serial.println("  Level 3: 75%  (191/255)");
    Serial.println("  Level 4: 100% (255/255)\n");
}

void loop() {
    // Heartbeat - double flash blue LED every 10 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastHeartbeat >= 10000) {  // 10 seconds
        lastHeartbeat = currentTime;
        // Double flash pattern
        digitalWrite(LED_BLUE, HIGH);
        delay(150);
        digitalWrite(LED_BLUE, LOW);
        delay(100);
        digitalWrite(LED_BLUE, HIGH);
        delay(150);
        digitalWrite(LED_BLUE, LOW);
    }

    // Read button state
    bool currentButtonState = digitalRead(BUTTON_SW1);

    // Button press detected (active low, transition from HIGH to LOW)
    if (currentButtonState == LOW && lastButtonState == HIGH) {
        buttonPressTime = millis();
        buttonHeld = false;
        Serial.println("[Button] Pressed");
    }

    // Button hold detection (pressed for 500ms or more)
    if (currentButtonState == LOW && !buttonHeld && (millis() - buttonPressTime >= 500)) {
        buttonHeld = true;
        Serial.println("[Button] Hold detected - cycling brightness");
    }

    // Button held - cycle through brightness levels slowly
    if (currentButtonState == LOW && buttonHeld && ledState) {
        // Cycle through brightness levels slowly while held (500ms per level)
        currentLevel = (currentLevel + 1) % BRIGHTNESS_LEVELS;
        analogWrite(LED_RED, brightnessLevels[currentLevel]);
        analogWrite(LED_EXTERNAL, brightnessLevels[currentLevel]);  // Mirror brightness to external LED
        Serial.print("[Button] Hold: Level ");
        Serial.print(currentLevel + 1);
        Serial.print(" - ");
        Serial.print((currentLevel + 1) * 25);
        Serial.println("%");
        delay(500);  // Slow cycling delay (500ms per level)
    }

    // Button release detected (transition from LOW to HIGH)
    if (currentButtonState == HIGH && lastButtonState == LOW) {
        unsigned long pressDuration = millis() - buttonPressTime;

        if (buttonHeld) {
            // Was held - brightness already set, just release
            Serial.print("[Button] Released at Level ");
            Serial.print(currentLevel + 1);
            Serial.print(" - ");
            Serial.print((currentLevel + 1) * 25);
            Serial.println("%");
            buttonHeld = false;
        }
        else if (pressDuration < 500) {
            // Quick click (< 500ms)
            if (!ledState) {
                // LED is OFF - turn ON at Level 1 (25%)
                ledState = true;
                currentLevel = 0;  // Reset to level 1
                analogWrite(LED_RED, brightnessLevels[currentLevel]);
                analogWrite(LED_EXTERNAL, brightnessLevels[currentLevel]);  // Mirror brightness to external LED
                Serial.println("[Button] Click: LED ON at Level 1 (25%)");
            }
            else {
                // LED is ON - turn OFF
                ledState = false;
                analogWrite(LED_RED, 0);
                analogWrite(LED_EXTERNAL, 0);  // Turn off external LED
                Serial.println("[Button] Click: LED OFF");
            }
        }

        delay(300);  // Debounce
    }

    lastButtonState = currentButtonState;
    delay(10);
}