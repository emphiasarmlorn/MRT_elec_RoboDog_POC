#include <SoftwareSerial.h>

SoftwareSerial stm32Serial(2, 3); // Nano's RX, TX respectively

void setup() {
  Serial.begin(115200);        // USB <-> PC
  stm32Serial.begin(115200);  // Nano <-> STM32

}

void loop() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    stm32Serial.println(data);

  }
}
