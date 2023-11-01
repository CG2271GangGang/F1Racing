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

/**
 * DEBUG mode
 * Uncomment to enable debug mode
 */
#define DEBUG

#define TURNSCALE 0.5
#define PIVOTSCALE 4
#define OFFSETSPEED 0

// Create a UART object
HardwareSerial SerialKL25(0); // UART0 on the ESP32

bool inDeadZone(int val);
void handleTankDrive(uint8_t &leftDataPacket, uint8_t &rightDataPacket);
void handleArcadeDrive(uint8_t &leftDataPacket, uint8_t &rightDataPacket);
void sendPayload(const uint8_t leftDataPacket, const uint8_t rightDataPacket);


PS2X ps2x; // create PS2 Controller Class

// right now, the library does NOT support hot pluggable controllers, meaning
// you must always either restart your Arduino after you connect the controller,
// or call config_gamepad(pins) again after connecting the controller.

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;
bool isArcadeDrive = false;  // Start in tank drive mode by default

void setupController() {
    // int error;
    do {
        delay(500);
        error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble); 
        Serial.print("#try config ");
        Serial.println(tryNum++);
    } while (error != 0);
}

void setup() {
    Serial.begin(9600);
    SerialKL25.begin(9600, SERIAL_8N1, RXD0, TXD0);
    setupController();
}


void printBinary(byte val){
  for (int i = 7; i>=0; i--){
    SerialKL25.print(bitRead(val,i));
  }
}

void loop()
{
    uint8_t leftDataPacket = 0;
    uint8_t rightDataPacket = 0;

    // if controller disconnected, send a stop command
    if(error == 1){
        Serial.print("Controller Disconnected");
        return;
    }
    else{
        ps2x.read_gamepad(false, 0);
        /* Switch between arcade and tank drive by pressing circle button */
        if (ps2x.ButtonPressed(PSB_CIRCLE)) {
            isArcadeDrive = !isArcadeDrive;
        }

        if (isArcadeDrive) {
            handleArcadeDrive(leftDataPacket, rightDataPacket);
        } else {
            handleTankDrive(leftDataPacket, rightDataPacket);
        }

        /* Play Ending Music */
        if (ps2x.NewButtonState(PSB_SQUARE)) {
            leftDataPacket = 0b00101011;
            rightDataPacket = 0b00101011;
        }

        /* Play Main Melody*/
        if (ps2x.NewButtonState(PSB_TRIANGLE)){
            leftDataPacket = 0x8b;
            rightDataPacket = 0x8b;
        }
        }
        sendPayload(leftDataPacket, rightDataPacket);
        // delay(100);
    }

bool inDeadZone(int val){
    return abs(val)<=5;
}

void sendPayload(const uint8_t leftDataPacket, const uint8_t rightDataPacket) {
    uint8_t payload[4];
    payload[0] = 0b00000011;
    payload[1] = leftDataPacket;
    payload[2] = rightDataPacket;
    payload[3] = 0b01111111;

    SerialKL25.write(payload, sizeof(payload));

    #ifdef DEBUG
    Serial.print("Payload: ");
    for (int i = 0; i < sizeof(payload); i++) {
        for (int j = 7; j >= 0; j--) {
            Serial.print(bitRead(payload[i], j));
        }
        Serial.print(" ");
    }
    Serial.println();
    #endif
}


void handleArcadeDrive(uint8_t &leftDataPacket, uint8_t &rightDataPacket)
{
    int leftJoystickY = ps2x.Analog(PSS_LY);
    int rightJoystickX = ps2x.Analog(PSS_RX);

    #ifdef DEBUG
        Serial.print("Arcade Drive: ");
        Serial.print(leftJoystickY);
        Serial.print(",");
        Serial.println(rightJoystickX);
    #endif

    int forwardSpeed = 128 - ps2x.Analog(PSS_LY); // Forward/backward value
    int turningValue = ps2x.Analog(PSS_RX) - 127; // Turning value

    int leftMotorSpeed, rightMotorSpeed;

    // Handle the forward and backward motion
    // if (inDeadZone(turningValue))
    if(turningValue == 0)
    {
        leftMotorSpeed = forwardSpeed;
        rightMotorSpeed = forwardSpeed;
    }
    // else if (turningValue < -5)
    else if(turningValue < 0)
    { // Turning or pivoting left
        // if (inDeadZone(forwardSpeed))
        if(forwardSpeed == 0)
        {                 // Rotating left
            leftMotorSpeed = constrain(turningValue * TURNSCALE, -127, 127);   // Negative value
            rightMotorSpeed = constrain(-turningValue * TURNSCALE, -127, 127); // Positive value
        }
        else
        { // Pivoting left
            // pivot forward, turning value -ve, forward +ve
            if (forwardSpeed > 0){
                if(turningValue == -127){ 
                    leftMotorSpeed = OFFSETSPEED; //pivot when joystick pushed to the max
                }
                else{
                    leftMotorSpeed = constrain(turningValue + 127, -127, 127); // 127 tend towards 0
                }
                rightMotorSpeed = constrain(forwardSpeed - turningValue * PIVOTSCALE, -127, 127);
            }
            // pivot backward, turning value -ve, forward -ve
            else if(forwardSpeed < 0){
                if (turningValue == -127)
                {
                    leftMotorSpeed = -OFFSETSPEED; // pivot when joystick pushed to the max
                }
                else
                {
                    leftMotorSpeed = constrain(-(turningValue + 127), -127, 127); // -127 tend towards 0
                }
                rightMotorSpeed = constrain(forwardSpeed + turningValue * PIVOTSCALE, -127, 127);
            }
        }
    }
    else
    { // Turning or pivoting right
        // if (inDeadZone(forwardSpeed))
        if(forwardSpeed == 0)
        {                                   // Rotating right
            leftMotorSpeed = constrain(turningValue * TURNSCALE, -127, 127); // Positive value
            rightMotorSpeed = constrain(-turningValue * TURNSCALE, -127, 127); // Negative value
           
        }
        else
        { // Pivoting right
            // pivot forward, turning value +ve, forward +ve
            if (forwardSpeed > 0){
                if(turningValue == 127){
                    rightMotorSpeed = OFFSETSPEED; // pivot when joystick pushed to the max
                }
                else{
                    rightMotorSpeed = constrain(127 - turningValue, -127, 127); // 127 tend towards 0
                }
                leftMotorSpeed = constrain(forwardSpeed + turningValue * PIVOTSCALE, -127, 127);
            }
            // pivot backward, turning value +ve, forward -ve
            else if (forwardSpeed < 0){
                if(turningValue == 127){
                    rightMotorSpeed = -OFFSETSPEED;
                }
                else{
                    rightMotorSpeed = constrain(turningValue - 127, -127, 127); // -127 tend towards 0
                }
                leftMotorSpeed = constrain(forwardSpeed - turningValue * PIVOTSCALE, -127, 127);
            }
        }
    }

    /*Rotating */
    if (ps2x.Button(PSB_L2)){ // rotate left at half speed
        leftMotorSpeed = -127 * TURNSCALE;
        rightMotorSpeed = 127 * TURNSCALE;
      }  

    if (ps2x.Button(PSB_R2)){ // rotate left at half speed
        leftMotorSpeed = 127 * TURNSCALE;
        rightMotorSpeed = -127 * TURNSCALE;
    }  
    /*Speed Cap Functions*/
    if (ps2x.Button(PSB_R1))
    {
        #ifdef DEBUG
        Serial.println("R1 pressed, capping left & right to half speed");
        #endif
        leftMotorSpeed = leftMotorSpeed / 3;
        rightMotorSpeed = rightMotorSpeed / 3;
    }
    if (ps2x.Button(PSB_L1))
    {
        #ifdef DEBUG
        Serial.println("L1 pressed, capping left & right to speed 1");
        #endif
        leftMotorSpeed = 1;
        rightMotorSpeed = 1;
    }
    
    /* Process the left motor speed */
    // if (inDeadZone(leftMotorSpeed))
    // {
    //     leftDataPacket |= 0b10 << 0;
    //     leftMotorSpeed = 1;
    // }
    if (leftMotorSpeed > 0)
    {
        // go forward
        leftDataPacket |= 0b01 << 0;
    }
    else if(leftMotorSpeed < 0)
    {
        // go backward
        leftDataPacket |= 0b10 << 0;
        leftMotorSpeed = abs(leftMotorSpeed); // Convert to positive for leftDataPacket.
    }
    leftDataPacket |= (uint8_t)((leftMotorSpeed / 127.0) * 63) << 2;

    /* Process the right motor speed */
    // if (inDeadZone(rightMotorSpeed))
    // {
    //     rightDataPacket |= 0b10 << 0;
    //     rightMotorSpeed = 1;
    // }
    if (rightMotorSpeed > 0)
    {
        // go forward
        rightDataPacket |= 0b01 << 0;
    }
    else if(rightMotorSpeed < 0)
    {
        // go backward
        rightDataPacket |= 0b10 << 0;
        rightMotorSpeed = abs(rightMotorSpeed); // Convert to positive for rightDataPacket.
    }
    rightDataPacket |= (uint8_t)((rightMotorSpeed / 127.0) * 63) << 2;
}

void handleTankDrive(uint8_t &leftDataPacket, uint8_t &rightDataPacket) { 
    int leftJoystickY = ps2x.Analog(PSS_LY);
    int rightJoystickY = ps2x.Analog(PSS_RY);

    #ifdef DEBUG
    Serial.print("Tank Drive: ");
    Serial.print(leftJoystickY);
    Serial.print(",");
    Serial.println(rightJoystickY);
    #endif

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


    if (ps2x.Button(PSB_L1)){ // Cap speed to half
        leftScaledValue = leftScaledValue / 3;
    }

    if (ps2x.Button(PSB_R1)){ // Cap speed to half
        rightScaledValue = rightScaledValue / 3;
    }
    leftDataPacket |= leftScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the leftDataPacket byte.
    rightDataPacket |= rightScaledValue << 2; // 6-bit scaled value is stored in bits 2 to 7 of the rightDataPacket byte.

    // rotating at half the speed
    if (ps2x.Button(PSB_L2)){ // rotate left
        leftDataPacket = 0b01010110; // left go back
        rightDataPacket = 0b01010101; // right go forward
    } 
    if (ps2x.Button(PSB_R2)){ // rotate right
        Serial.print("Rotating right");
        leftDataPacket = 0b01010101; // left go forward
        rightDataPacket = 0b01010110; // right go back
    } 
}
