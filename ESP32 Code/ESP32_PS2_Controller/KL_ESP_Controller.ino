/*
TODO:
- increase a wider range for stop 127-129 for reliability
*/

#include <PS2X_lib.h> //for v1.6
#include <HardwareSerial.h>

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/

//  ESP32 pin
// https://github.com/espressif/arduino-esp32/blob/master/docs/esp32_pinmap.png

#define PS2_DAT 19 // MISO  19
#define PS2_CMD 23 // MOSI  23
#define PS2_SEL 5  // SS     5
#define PS2_CLK 18 // SLK   18

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures false
#define rumble false

/**
 * UART between ESP32 and KL25
 */
#define RXD0 3
#define TXD0 1

// Create a UART object
HardwareSerial SerialKL25(0); // UART0 on the ESP32

PS2X ps2x; // create PS2 Controller Class

// right now, the library does NOT support hot pluggable controllers, meaning
// you must always either restart your Arduino after you connect the controller,
// or call config_gamepad(pins) again after connecting the controller.

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;
bool isArcadeDrive = false;  // Start in tank drive mode by default


void setup()
{
    Serial.begin(9600);
    SerialKL25.begin(9600, SERIAL_8N1, RXD0, TXD0); // Baud rate should match the KL25 configuration

    // added delay to give wireless ps2 module some time to startup, before configuring it
    // CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

    while (error != 0)
    {
        delay(1000); // 1 second wait
        // setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
        error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
        Serial.print("#try config ");
        Serial.println(tryNum);
        tryNum++;
    }

    Serial.println(ps2x.Analog(1), HEX);
}

/*Helper functions */

void printBinary(byte val){
  for (int i = 7; i>=0; i--){
    SerialKL25.print(bitRead(val,i));
  }
}

void loop()
{
    ps2x.read_gamepad(false, vibrate); // read controller and set large motor to spin at 'vibrate' speed

    /*Init Arrays & Bytes */    
    uint8_t payload[4];         // Create a buffer to hold the payload
    uint8_t leftDataPacket = 0; // Init data packet for left drive train
    uint8_t rightDataPacket = 0; // Init data packet for right drive train
    uint8_t startByte = 0b00000011; // Start byte with LSB '11'
    uint8_t endByte = 0b01111111;   // End byte with LSB '11'

    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      isArcadeDrive = !isArcadeDrive;  // Toggle between arcade and tank drive by pressing circle button
    }

    if (isArcadeDrive) {
      /* Arcade Drive */
      Serial.print("Arcade Drive: ");
      int leftJoystickY = ps2x.Analog(PSS_LY);
      int rightJoystickX = ps2x.Analog(PSS_RX);
      Serial.print(leftJoystickY);
      Serial.print(",");
      Serial.println(rightJoystickX);

      int forwardSpeed = leftJoystickY - 128;  // This will give us a range from -128 to 127.
      int turningValue = rightJoystickX - 128; // Same range, negative for left turn, positive for right.

      int leftMotorSpeed, rightMotorSpeed;

      if (turningValue < 0) { // Turning left
          leftMotorSpeed = constrain(forwardSpeed + abs(turningValue), -127, 127);  // Decrease left motor speed
          rightMotorSpeed = constrain(forwardSpeed - abs(turningValue), -127, 127); // Increase right motor speed
      } else if(turningValue > 0) { // Turning right
          leftMotorSpeed = constrain(forwardSpeed - turningValue, -127, 127); 
          rightMotorSpeed = constrain(forwardSpeed + turningValue, -127, 127); 
      } else{ // not moving
          leftMotorSpeed = constrain(forwardSpeed, -127, 127); 
          rightMotorSpeed = constrain(forwardSpeed, -127, 127); 
      }

      // Process the left motor speed
      if (leftMotorSpeed == 0) {
        leftDataPacket |= 0b00 << 0;
      } else if (leftMotorSpeed < 0) {
        // go forward
        leftDataPacket |= 0b01 << 0;
        leftMotorSpeed = abs(leftMotorSpeed);  // Convert to positive for leftDataPacket.
      } else {
      // go backward
        leftDataPacket |= 0b10 << 0;
      }
      leftDataPacket |= (uint8_t)((leftMotorSpeed / 127.0) * 63) << 2;

      // Process the right motor speed
      if (rightMotorSpeed == 0) {
        rightDataPacket |= 0b00 << 0;
      } else if (rightMotorSpeed < 0) {
        // go forward
        rightDataPacket |= 0b01 << 0;
        rightMotorSpeed = abs(rightMotorSpeed);  // Convert to positive for rightDataPacket.
      } else {
        // go backward
        rightDataPacket |= 0b10 << 0;
      }
      rightDataPacket |= (uint8_t)((rightMotorSpeed / 127.0) * 63) << 2;

      /* Drift */
      if(ps2x.Button(PSB_L2)){ 
        Serial.println("L2 pressed, drifting left");
        // left half speed of right
        leftDataPacket = 0b00011101;
        rightDataPacket = 0b11111101;
      }
      else if(ps2x.Button(PSB_R2)){
        Serial.println("R2 pressed, drifting right");
        // right half speed of left
        leftDataPacket = 0b11111101;
        rightDataPacket = 0b00011101;
      }

      /* Cap speed */
      if (ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1)){
        Serial.println("L1 & R1 pressed, capping at 50% speed");
        // bitshift right by 2 for index 2-7
        // Create a bit mask to isolate bits 2 to 7
        unsigned char mask = 0b11111100;

        // Extract the bits within the range (bits 2 to 7)
        unsigned char extractedLeftBits = (leftDataPacket & mask) >> 2;
        unsigned char extractedRightBits = (rightDataPacket & mask) >> 2;

        // Perform the right bit shift operation
        unsigned char shiftedLeftBits = extractedLeftBits >> 1;
        unsigned char shiftedRightBits = extractedRightBits >> 1;

        // Clear the original bits 2 to 7 in the data packet
        leftDataPacket &= ~mask;
        rightDataPacket &= ~mask;

        // Combine the shifted bits with the original data packet
        leftDataPacket |= (shiftedLeftBits << 2);
        rightDataPacket |= (shiftedRightBits << 2);
      }
    }
    else{
      /* Tank Drive */
      int leftJoystickY = ps2x.Analog(PSS_LY);
      int rightJoystickY = ps2x.Analog(PSS_RY);
      Serial.print("Tank Drive: ");
      Serial.print(leftJoystickY);
      Serial.print(",");
      Serial.println(rightJoystickY);

      // Process the left joystick Y value
      if (leftJoystickY == 128){
        leftDataPacket |= 0b00 << 0; // Set bits 0 and 1 to '00'
      }else if (leftJoystickY >= 0 && leftJoystickY <= 127){
        leftDataPacket |= 0b01 << 0; // Set bits 0 and 1 to '01'
      }else if (leftJoystickY >= 129 && leftJoystickY <= 255){
        leftDataPacket |= 0b10 << 0; // Set bits 0 and 1 to '10'
      }

      uint8_t leftScaledValue = 0;
      if (leftJoystickY >= 0 && leftJoystickY <= 127){
        leftScaledValue = (uint8_t)(((128.0 - leftJoystickY) / 128.0) * 63);
      } else if (leftJoystickY >= 129 && leftJoystickY <= 255){
        leftScaledValue = (uint8_t)(((leftJoystickY - 128) / 127.0) * 63);
      }
      
      if (ps2x.Button(PSB_L2)){ // Drift
        Serial.println("L2 pressed, drifting left");
        // left 1/4 speed
        leftScaledValue = leftScaledValue / 4;
      }
      else if (ps2x.Button(PSB_L1)){ // Cap speed
        leftScaledValue = leftScaledValue / 3;
      }
      leftDataPacket |= leftScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the leftDataPacket byte.

      // Process the right joystick Y value (similar to left joystick)
      if (rightJoystickY == 128){
        rightDataPacket |= 0b00 << 0;
      } else if (rightJoystickY >= 0 && rightJoystickY <= 127){
        rightDataPacket |= 0b01 << 0;
      } else if (rightJoystickY >= 129 && rightJoystickY <= 255){
        rightDataPacket |= 0b10 << 0;
      }

      uint8_t rightScaledValue = 0;
      if (rightJoystickY >= 0 && rightJoystickY <= 127){
        rightScaledValue = (uint8_t)(((128.0 - rightJoystickY) / 128.0) * 63);
      } else if (rightJoystickY >= 129 && rightJoystickY <= 255) {
        rightScaledValue = (uint8_t)(((rightJoystickY - 128) / 127.0) * 63);
      }

      if (ps2x.Button(PSB_R2)){ // Drift
        Serial.println("R2 pressed, drifting right");
        // right 1/2 speed
        rightScaledValue = rightScaledValue / 2;
      }
      else if (ps2x.Button(PSB_R1)){ // Cap speed
        rightScaledValue = rightScaledValue / 3;
      }
      rightDataPacket |= rightScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the rightDataPacket byte.
    }

    
    /*Ending Music -Activate when Square pressed OR released */
    if(ps2x.NewButtonState(PSB_SQUARE))      {
      Serial.println("Square just released");     
      leftDataPacket = 0b00101011;
      rightDataPacket = 0b00101011;
    }       

    // Copy the bytes into the payload
    payload[0] = startByte; 
    payload[1] = leftDataPacket;
    payload[2] = rightDataPacket;
    payload[3] = endByte;

    // Send the payload over UART to KL25
    // for (int i = 0; i < sizeof(payload); i++) {
    //     SerialKL25.write(payload[i]);
    // }
    SerialKL25.write(payload, sizeof(payload));
   
    // Print the payload (for testing)
    SerialKL25.print("Payload: ");
    for (int i = 0; i < sizeof(payload); i++)
    {
        // Serial.print(payload[i], BIN);
        printBinary(payload[i]);
        SerialKL25.print(" ");
    }
    SerialKL25.println();

    // Add a delay or adjust timing based on your application requirements
    // delay(500);
}
