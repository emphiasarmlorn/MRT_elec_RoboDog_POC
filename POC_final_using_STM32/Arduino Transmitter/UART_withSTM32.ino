#include <SoftwareSerial.h>

SoftwareSerial Serial1(2, 3);  // RX = 2, TX = 3

// #include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>

//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo pulse limits (tune if needed)
// #define SERVOMIN  102   // ≈ 500 µs
// #define SERVOMAX  512   // ≈ 2500 µs

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600); 

  // Wire.begin();
  // pwm.begin();
  // pwm.setPWMFreq(50);   // 50 Hz for servos
}

void loop() {

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();                 // remove CR if present

    Serial.println(data);        // debug to PC
    Serial1.println(data);       // send full line to STM32
  }

  // if (Serial.available()) {
  //   String data = Serial.readStringUntil('\n');
  //   data.trim();

  //   float values[4];
  //   int idx = 0;
  //   int last = 0;

  //   // Parse CSV values (0–1)
  //   for (int i = 0; i < data.length(); i++) {
  //     if (data[i] == ',' || i == data.length() - 1) {
  //       String sub = data.substring(last, i == data.length() - 1 ? i + 1 : i);
  //       values[idx++] = sub.toFloat();
  //       last = i + 1;
  //       if (idx == 4) break;
  //     }
  //   }

    // Convert to servo angles and drive PCA9685
    // for (int i = 0; i < 4; i++) {
    //   float normalized = constrain(values[i], 0.0, 1.0);

    //   // 0–1 → 0–180 degrees
    //   float angle = normalized * 180.0;

    //   // Angle → PCA9685 pulse
    //   int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    //   pwm.setPWM(i, 0, pulse);
    // }

    //Serial.println("OK");
  //}
}