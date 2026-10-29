// /*
//  * PCA9685 WebSerial Control Firmware (ESP32 / Arduino)
//  * Receives serial commands from Web GUI (e.g. "s 0 90\n")
//  * Library Required: Adafruit PWM Servo Driver Library
//  */

// #include <Wire.h>
// #include <Adafruit_PWMServoDriver.h>

// // Called with default I2C address 0x40
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// #define SERVOMIN  125 // pulse count (out of 4096) at 0 degrees
// #define SERVOMAX  540 // pulse count (out of 4096) at 180 degrees
// #define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// // Allowed angle range per PCA9685 pin (from your servo config)
// const int LIM_MIN[16] = {30, 70, 40, 30, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
// const int LIM_MAX[16] = {130, 150, 140, 120, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180};

// void setup() {
//   Serial.begin(115200);
//   Serial.println("PCA9685 WebSerial Servo Controller Starting...");

//   pwm.begin();
//   pwm.setOscillatorFrequency(27000000);
//   pwm.setPWMFreq(SERVO_FREQ);

//   delay(10);
// }

// // Convert angle (0 to 180 degrees) to PCA9685 tick value (0..4095)
// int angleToTicks(int angle) {
//   return map(angle, 0, 180, SERVOMIN, SERVOMAX);
// }

// void loop() {
//   if (Serial.available() > 0) {
//     String input = Serial.readStringUntil('\n');
//     input.trim();
    
//     // Command format: "s <channel> <angle>"
//     if (input.startsWith("s ")) {
//       int firstSpace = input.indexOf(' ');
//       int secondSpace = input.indexOf(' ', firstSpace + 1);
      
//       if (firstSpace != -1 && secondSpace != -1) {
//         int channel = input.substring(firstSpace + 1, secondSpace).toInt();
//         int angle = input.substring(secondSpace + 1).toInt();
        
//         if (channel >= 0 && channel <= 15 && angle >= 0 && angle <= 180) {
//           angle = constrain(angle, LIM_MIN[channel], LIM_MAX[channel]);
//           int ticks = angleToTicks(angle);
//           pwm.setPWM(channel, 0, ticks);
//           Serial.print("OK: Servo ");
//           Serial.print(channel);
//           Serial.print(" set to ");
//           Serial.println(angle);
//         }
//       }
//     }
//   }
// }