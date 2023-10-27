/**
 * This example turns the ESP32 into a Bluetooth LE keyboard that writes the words, presses Enter, presses a media key and then Ctrl+Alt+Delete
 */
#include <Arduino.h>
int mode = 0;
boolean notPressed = true;
void setup()
{
    Serial.begin(115200);
    pinMode(22, INPUT_PULLUP);
    pinMode(23, INPUT_PULLUP);
    pinMode(26, INPUT_PULLUP);
    pinMode(25, INPUT_PULLUP);
}

void loop()
{
    if (mode != 1 && digitalRead(22) == LOW) { // MODE1
        Serial.print("1");
        mode = 1;
    }
    if (mode != 2 && digitalRead(26) == LOW) { // MODE2
        Serial.print("2");
        mode = 2;
    }
    if (mode != 3 && digitalRead(25) == LOW) { // MODE3
        Serial.print("3");
        mode = 3;
    }
    if (mode != 4 && digitalRead(23) == LOW) { // MODE4
        Serial.print("4");
        mode = 4;
    }

    if (mode == 3) {
        int diff = analogRead(32) - analogRead(33);
        if (diff > 100) {
            if (notPressed) {
                Serial.print("j");
                delay(10);
                notPressed = false;
            }
        } else if (diff < -100) {
            if (notPressed) {
                Serial.print("l");
                delay(10);
                notPressed = false;
            }
        } else {
            if (notPressed == false) {
                Serial.print("k");
                notPressed = true;
            }
            delay(10);
        }
    }
}
