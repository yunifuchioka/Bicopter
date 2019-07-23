
#include "Constants.h"
#include "Motors.h"

Motors motors(LEFT_SERVO_PIN, RIGHT_SERVO_PIN, LEFT_BLDC_PIN, RIGHT_BLDC_PIN, LEFT_ENCODER_PIN, RIGHT_ENCODER_PIN);

void setup() {
    Serial.begin(9600);

    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

    motors.attachMotors();
}

void loop() {

    bool leftButton = !digitalRead(3);
    bool rightButton = !digitalRead(2);

    int leftPot = analogRead(A3);
    int rightPot = analogRead(A2);

    int u1 = (leftButton)? 135 : ((rightButton)? -45 : 90);

    int u3 = map(leftPot, 0, 1023, 0, 1000);

    motors.writeMotors(u1, u1, u3, u3);
    
    Serial.print(u1);
    Serial.print('\t');
    Serial.print(u3);
    Serial.print('\t');
    Serial.print(motors.get_y1());
    Serial.print('\t');
    Serial.print(motors.get_y2());
    Serial.print('\t');
    Serial.print(millis());
    Serial.print('\n');
}