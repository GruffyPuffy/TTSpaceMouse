// This code is bases of Teaching Tech's project (which is also based on varios people, see below)
// https://www.printables.com/model/864950-open-source-spacemouse-space-mushroom-remix

#include "HID.h"

// Debugging
// 0: Debugging off. Set to this once everything is working.
// 1: Output raw joystick values. 0-1023 raw ADC 10-bit values
// 2: Output centered joystick values. Values should be approx -500 to +500, jitter around 0 at idle.
// 3: Output centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle.
// 4: Output translation and rotation values. Approx -800 to 800 depending on the parameter.
// 5: Output debug 4 and 5 side by side for direct cause and effect reference.
int debug = 6;

// Direction
// Modify the direction of translation/rotation depending on preference. This can also be done per application in the 3DConnexion software.
// Switch between true/false as desired.
bool invX = false;  // pan left/right
bool invY = false;  // pan up/down
bool invZ = true;   // zoom in/out
bool invRX = true;  // Rotate around X axis (tilt front/back)
bool invRY = false; // Rotate around Y axis (tilt left/right)
bool invRZ = true;  // Rotate around Z axis (twist left/right)

// Speed - Had to be removed when the V2 shorter range of motion. The logic reduced sensitivity on small movements. Use 3DConnexion slider instead for V2.
// Modify to change sensitibity/speed. Default and maximum 100. Works like a percentage ie. 50 is half as fast as default. This can also be done per application in the 3DConnexion software.
// int16_t speed = 100;

// Default Assembly when looking from above:
//    C           Y+
//    |           .
// B--+--D   X-...Z+...X+
//    |           .
//    A           Y-
//
// Wiring. Matches the first eight analogue pins of the Arduino Pro Micro (atmega32u4)
int PINLIST[8] = {
    // The positions of the reads
    A1, // X-axis A
    A0, // Y-axis A
    A3, // X-axis B
    A2, // Y-axis B
    A7, // X-axis C
    A6, // Y-axis C
    A9, // X-axis D
    A8  // Y-axis D
};
// Deadzone to filter out unintended movements. Increase if the mouse has small movements when it should be idle or the mouse is too senstive to subtle movements.
int DEADZONE = 3; // Recommended to have this as small as possible for V2 to allow smaller knob range of motion.

// This portion sets up the communication with the 3DConnexion software. The communication protocol is created here.
// hidReportDescriptor webpage can be found here: https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/
static const uint8_t _hidReportDescriptor[] PROGMEM = {
    0x05, 0x01,       //  Usage Page (Generic Desktop)
    0x09, 0x08,       //  0x08: Usage (Multi-Axis)
    0xa1, 0x01,       //  Collection (Application)
    0xa1, 0x00,       // Collection (Physical)
    0x85, 0x01,       //  Report ID
    0x16, 0x00, 0x80, // logical minimum (-500)
    0x26, 0xff, 0x7f, // logical maximum (500)
    0x36, 0x00, 0x80, // Physical Minimum (-32768)
    0x46, 0xff, 0x7f, // Physical Maximum (32767)
    0x09, 0x30,       //    Usage (X)
    0x09, 0x31,       //    Usage (Y)
    0x09, 0x32,       //    Usage (Z)
    0x75, 0x10,       //    Report Size (16)
    0x95, 0x03,       //    Report Count (3)
    0x81, 0x02,       //    Input (variable,absolute)
    0xC0,             //  End Collection
    0xa1, 0x00,       // Collection (Physical)
    0x85, 0x02,       //  Report ID
    0x16, 0x00, 0x80, // logical minimum (-500)
    0x26, 0xff, 0x7f, // logical maximum (500)
    0x36, 0x00, 0x80, // Physical Minimum (-32768)
    0x46, 0xff, 0x7f, // Physical Maximum (32767)
    0x09, 0x33,       //    Usage (RX)
    0x09, 0x34,       //    Usage (RY)
    0x09, 0x35,       //    Usage (RZ)
    0x75, 0x10,       //    Report Size (16)
    0x95, 0x03,       //    Report Count (3)
    0x81, 0x02,       //    Input (variable,absolute)
    0xC0,             //  End Collection

    0xa1, 0x00, // Collection (Physical)
    0x85, 0x03, //  Report ID
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //    Logical Maximum (1)
    0x75, 0x01, //    Report Size (1)
    0x95, 32,   //    Report Count (24)
    0x05, 0x09, //    Usage Page (Button)
    0x19, 1,    //    Usage Minimum (Button #1)
    0x29, 32,   //    Usage Maximum (Button #24)
    0x81, 0x02, //    Input (variable,absolute)
    0xC0,
    0xC0};

// Axes are matched to pin order.
#define AX 0
#define AY 1
#define BX 2
#define BY 3
#define CX 4
#define CY 5
#define DX 6
#define DY 7

// Centerpoint variable to be populated during setup routine.
int centerPoints[8];

int maxPoints[8] = {512, 512, 512, 512, 512, 512, 512, 512};
int minPoints[8] = {512, 512, 512, 512, 512, 512, 512, 512};

// Function to read and store analogue voltages for each joystick axis.
void readAllFromJoystick(int *rawReads)
{
    for (int i = 0; i < 8; i++)
    {
        rawReads[i] = analogRead(PINLIST[i]);

        // Auto max/min check
        if (rawReads[i] > maxPoints[i])
        {
            maxPoints[i] = rawReads[i];
        }

        if (rawReads[i] < minPoints[i])
        {
            minPoints[i] = rawReads[i];
        }
    }
}

void setup()
{
    // HID protocol is set.
    static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
    HID().AppendDescriptor(&node);
    // Begin Seral for debugging
    Serial.begin(250000);
    delay(100);
    
    
    // Read idle/centre positions for joysticks.
    readAllFromJoystick(centerPoints);
    for (int p=0; p<100; p++)
    {
        delay(3);

        int newCenterPoints[8];
        readAllFromJoystick(newCenterPoints);

        // Filter in new center
        for (int i = 0; i < 8; i++)
        {
            centerPoints[i] = (centerPoints[i] + newCenterPoints[i]) / 2;
        }
    }
}

// Function to send translation and rotation data to the 3DConnexion software using the HID protocol outlined earlier. Two sets of data are sent: translation and then rotation.
// For each, a 16bit integer is split into two using bit shifting. The first is mangitude and the second is direction.
void send_command(int16_t rx, int16_t ry, int16_t rz, int16_t x, int16_t y, int16_t z)
{
    uint8_t trans[6];
    trans[0] = (uint8_t)(x & 0xFF);
    trans[1] = (uint8_t)(x >> 8);
    trans[2] = (uint8_t)(y & 0xFF);
    trans[3] = (uint8_t)(y >> 8);
    trans[4] = (uint8_t)(z & 0xFF);
    trans[5] = (uint8_t)(z >> 8);
    HID().SendReport(1, trans, 6);

    uint8_t rot[6];
    rot[0] = (uint8_t)(rx & 0xFF);
    rot[1] = (uint8_t)(rx >> 8);
    rot[2] = (uint8_t)(ry & 0xFF);
    rot[3] = (uint8_t)(ry >> 8);
    rot[4] = (uint8_t)(rz & 0xFF);
    rot[5] = (uint8_t)(rz >> 8);
    HID().SendReport(2, rot, 6);
}

void old_loop()
{
    int rawReads[8], centered[8];
    // Joystick values are read. 0-1023
    readAllFromJoystick(rawReads);

    // Report back 0-1023 raw ADC 10-bit values if enabled
    if (debug == 1)
    {
        Serial.print("AX:");
        Serial.print(rawReads[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(rawReads[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(rawReads[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(rawReads[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(rawReads[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(rawReads[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(rawReads[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.println(rawReads[7]);
    }

    // Subtract centre position from measured position to determine movement.
    for (int i = 0; i < 8; i++)
        centered[i] = rawReads[i] - centerPoints[i]; //

    // Report centered joystick values if enabled. Values should be approx -500 to +500, jitter around 0 at idle.
    if (debug == 2)
    {
        Serial.print("AX:");
        Serial.print(centered[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(centered[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(centered[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(centered[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(centered[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(centered[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(centered[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.println(centered[7]);
    }

    // Filter movement values. Set to zero if movement is below deadzone threshold.
    for (int i = 0; i < 8; i++)
    {
        if (centered[i] < DEADZONE && centered[i] > -DEADZONE)
            centered[i] = 0;
    }

    // Report centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle
    if (debug == 3)
    {
        Serial.print("AX:");
        Serial.print(centered[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(centered[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(centered[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(centered[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(centered[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(centered[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(centered[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.println(centered[7]);
    }

    // Doing all through arithmetic contribution by fdmakara
    // Integer has been changed to 16 bit int16_t to match what the HID protocol expects.
    int16_t transX, transY, transZ, rotX, rotY, rotZ; // Declare movement variables at 16 bit integers
    // Original fdmakara calculations
    // transX = (-centered[AX] +centered[CX])/1;
    // transY = (-centered[BX] +centered[DX])/1;
    // transZ = (-centered[AY] -centered[BY] -centered[CY] -centered[DY])/2;
    // rotX = (-centered[AY] +centered[CY])/2;
    // rotY = (+centered[BY] -centered[DY])/2;
    // rotZ = (+centered[AX] +centered[BX] +centered[CX] +centered[DX])/4;
    // My altered calculations based on debug output. Final divisor can be changed to alter sensitivity for each axis.

    transX = -(-centered[CY] + centered[AY]) / 1;
    transY = (-centered[BY] + centered[DY]) / 1;
    if ((abs(centered[AX]) > DEADZONE) && (abs(centered[BX]) > DEADZONE) && (abs(centered[CX]) > DEADZONE) && (abs(centered[DX]) > DEADZONE))
    {
        transZ = (-centered[AX] - centered[BX] - centered[CX] - centered[DX]) / 2;
        transX = 0;
        transY = 0;
    }
    else
    {
        transZ = 0;
    }
    rotX = (-centered[AX] + centered[CX]) / 1;
    rotY = (+centered[BX] - centered[DX]) / 1;
    if ((abs(centered[AY]) > DEADZONE) && (abs(centered[BY]) > DEADZONE) && (abs(centered[CY]) > DEADZONE) && (abs(centered[DY]) > DEADZONE))
    {
        rotZ = (+centered[AY] + centered[BY] + centered[CY] + centered[DY]) / 2;
        rotX = 0;
        rotY = 0;
    }
    else
    {
        rotZ = 0;
    }

    // Alter speed to suit user preference - Use 3DConnexion slider instead for V2.
    // transX = transX/100*speed;
    // transY = transY/100*speed;
    // transZ = transZ/100*speed;
    // rotX = rotX/100*speed;
    // rotY = rotY/100*speed;
    // rotZ = rotZ/100*speed;
    // Invert directions if needed
    if (invX == true)
    {
        transX = transX * -1;
    };
    if (invY == true)
    {
        transY = transY * -1;
    };
    if (invZ == true)
    {
        transZ = transZ * -1;
    };
    if (invRX == true)
    {
        rotX = rotX * -1;
    };
    if (invRY == true)
    {
        rotY = rotY * -1;
    };
    if (invRZ == true)
    {
        rotZ = rotZ * -1;
    };


    // Special case... 
    // if (abs(rotZ) > 400)
    // {
    //     transX = 0;
    //     transY = 0;
    //     transZ = 0;
    //     rotY = 0;
    //     rotX = 0;
    // }

    if ( (abs(transX) < DEADZONE) && (abs(transY) < DEADZONE) && (abs(transZ) < DEADZONE) )
    {
        rotX = map(rotX, -1024, 1024, -32768, 32767);
        rotZ = map(rotZ, -1024, 1024, -32768, 32767);
        rotY = map(rotY, -1024, 1024, -32768, 32767);
    }

    transX = map(transX, -1024, 1024, -32768, 32767);
    transZ = map(transZ, -1024, 1024, -32768, 32767);
    transY = map(transY, -1024, 1024, -32768, 32767);

    float transScale = 0.25f;
    float rotScale = 0.25f;

    transX = transScale*transX;
    transY = transScale*transY;
    transZ = transScale*transZ;

    rotX = rotScale*rotX;
    rotY = rotScale*rotY;
    rotZ = rotScale*rotZ;

    // Report translation and rotation values if enabled. Approx -800 to 800 depending on the parameter.
    if (debug == 4)
    {
        Serial.print("TX:");
        Serial.print(transX);
        Serial.print(",");
        Serial.print("TY:");
        Serial.print(transY);
        Serial.print(",");
        Serial.print("TZ:");
        Serial.print(transZ);
        Serial.print(",");
        Serial.print("RX:");
        Serial.print(rotX);
        Serial.print(",");
        Serial.print("RY:");
        Serial.print(rotY);
        Serial.print(",");
        Serial.print("RZ:");
        Serial.println(rotZ);
    }
    // Report debug 4 and 5 info side by side for direct reference if enabled. Very useful if you need to alter which inputs are used in the arithmatic above.
    if (debug == 5)
    {
        Serial.print("AX:");
        Serial.print(centered[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(centered[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(centered[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(centered[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(centered[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(centered[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(centered[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.print(centered[7]);
        Serial.print("||");
        Serial.print("TX:");
        Serial.print(transX);
        Serial.print(",");
        Serial.print("TY:");
        Serial.print(transY);
        Serial.print(",");
        Serial.print("TZ:");
        Serial.print(transZ);
        Serial.print(",");
        Serial.print("RX:");
        Serial.print(rotX);
        Serial.print(",");
        Serial.print("RY:");
        Serial.print(rotY);
        Serial.print(",");
        Serial.print("RZ:");
        Serial.println(rotZ);
    }


    // Send data to the 3DConnexion software.
    // The correct order for me was determined after trial and error
    send_command(rotX, rotZ, rotY, transX, transZ, transY);
    //delay(1);
    send_command(rotX-1, rotZ-1, rotY-1, transX-1, transZ-1, transY-1);
    //delay(1);
}




void loop()
{
    const float deadZone = 0.05;

    const float f_invX = 1.0f;  // pan left/right
    const float f_invY = 1.0f;  // pan up/down
    const float f_invZ = -1.0f;   // zoom in/out
    const float f_invRX = -1.0f;  // Rotate around X axis (tilt front/back)
    const float f_invRY = 1.0f; // Rotate around Y axis (tilt left/right)
    const float f_invRZ = -1.0f;  // Rotate around Z axis (twist left/right)

    const float boostTX = 1.3f;
    const float boostTY = 1.3f;
    const float boostTZ = 1.3f;

    const float boostRX = 1.5f;
    const float boostRY = 1.5f;
    const float boostRZ = 1.5f;

    // Output
    float transX, transY, transZ, rotX, rotY, rotZ;

    // Scale [0...1023]
    int rawReads[8];

    // Scale [-1...1]
    float centered[8];


    // Joystick values are read. 0-1023
    readAllFromJoystick(rawReads);

    // Subtract centre position from measured position to determine movement.
    // Change scale to [-1...1]
    for (int i = 0; i < 8; i++)
    {
        centered[i] = (rawReads[i] - centerPoints[i]);

        if (centered[i] >= 0)
        {
            centered[i] = centered[i] / (float)(1024.0f - centerPoints[i]);
        }
        else
        {
            centered[i] = centered[i] / (float)(centerPoints[i]);
        }
    }

    if (debug == 2)
    {
        // Report centered joystick values if enabled. Values should be approx -500 to +500, jitter around 0 at idle.
        Serial.print("AX:");
        Serial.print(centered[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(centered[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(centered[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(centered[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(centered[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(centered[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(centered[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.print("           ");
        Serial.print("cAX:");
        Serial.print(centerPoints[0]);
        Serial.print(",");
        Serial.print("cAY:");
        Serial.print(centerPoints[1]);
        Serial.print(",");
        Serial.print("cBX:");
        Serial.print(centerPoints[2]);
        Serial.print(",");
        Serial.print("cBY:");
        Serial.print(centerPoints[3]);
        Serial.print(",");
        Serial.print("cCX:");
        Serial.print(centerPoints[4]);
        Serial.print(",");
        Serial.print("cCY:");
        Serial.print(centerPoints[5]);
        Serial.print(",");
        Serial.print("cDX:");
        Serial.print(centerPoints[6]);
        Serial.print(",");
        Serial.print("cDY:");
        Serial.println(centerPoints[7]);
    }

    // Apply deadzone
    for (int i = 0; i < 8; i++)
    {
        if (fabs(centered[i]) < deadZone)
        {
            centered[i] = 0.0f;
        }
    }

    if (debug == 3)
    {
        // Report centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle
        Serial.print("AX:");
        Serial.print(centered[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(centered[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(centered[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(centered[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(centered[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(centered[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(centered[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.println(centered[7]);
    }


    // Calculate translation and rotation based on the 4 joysticks
    transX = -(-centered[CY] + centered[AY]) / 2.0f;
    transY = (-centered[BY] + centered[DY]) / 2.0f;
    transZ = (-centered[AX] - centered[BX] - centered[CX] - centered[DX]) / 4.0f;

    rotX = (-centered[AX] + centered[CX]) / 2.0;
    rotY = (+centered[BX] - centered[DX]) / 2.0f;
    rotZ = (+centered[AY] + centered[BY] + centered[CY] + centered[DY]) / 4.0f;


    // Boost some values as we are not using the full range of the joysticks (some times)
    // Boost and invert any results
    transX = constrain(boostTX*f_invX*transX, -1.0f, 1.0f);
    transY = constrain(boostTY*f_invY*transY, -1.0f, 1.0f);
    transZ = constrain(boostTZ*f_invZ*transZ, -1.0f, 1.0f);

    rotX = constrain(boostRX*f_invRX*rotX, -1.0f, 1.0f);
    rotY = constrain(boostRY*f_invRY*rotY, -1.0f, 1.0f);
    rotZ = constrain(boostRZ*f_invRZ*rotZ, -1.0f, 1.0f);
  

    if (debug == 4)
    {
        Serial.print("TX:");
        Serial.print(transX);
        Serial.print(",");
        Serial.print("TY:");
        Serial.print(transY);
        Serial.print(",");
        Serial.print("TZ:");
        Serial.print(transZ);
        Serial.print(",");
        Serial.print("RX:");
        Serial.print(rotX);
        Serial.print(",");
        Serial.print("RY:");
        Serial.print(rotY);
        Serial.print(",");
        Serial.print("RZ:");
        Serial.println(rotZ);
    }

    if (debug == 5)
    {
        // Report debug 4 and 5 info side by side for direct reference if enabled. Very useful if you need to alter which inputs are used in the arithmatic above.
        Serial.print("AX:");
        Serial.print(centered[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(centered[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(centered[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(centered[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(centered[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(centered[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(centered[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.print(centered[7]);
        Serial.print(" || ");
        Serial.print("TX:");
        Serial.print(transX);
        Serial.print(",");
        Serial.print("TY:");
        Serial.print(transY);
        Serial.print(",");
        Serial.print("TZ:");
        Serial.print(transZ);
        Serial.print(",");
        Serial.print("RX:");
        Serial.print(rotX);
        Serial.print(",");
        Serial.print("RY:");
        Serial.print(rotY);
        Serial.print(",");
        Serial.print("RZ:");
        Serial.print(rotZ);

/*        Serial.print(" || ");
        Serial.print("AX:");
        Serial.print(minPoints[0]);
        Serial.print(",");
        Serial.print(maxPoints[0]);
        Serial.print(" AY:");
        Serial.print(minPoints[1]);
        Serial.print(",");
        Serial.print(maxPoints[1]);

        Serial.print(" BX:");
        Serial.print(minPoints[2]);
        Serial.print(",");
        Serial.print(maxPoints[2]);
        Serial.print(" BY:");
        Serial.print(minPoints[3]);
        Serial.print(",");
        Serial.print(maxPoints[3]);

        Serial.print(" CX:");
        Serial.print(minPoints[4]);
        Serial.print(",");
        Serial.print(maxPoints[4]);
        Serial.print(" CY:");
        Serial.print(minPoints[5]);
        Serial.print(",");
        Serial.print(maxPoints[5]);

        Serial.print(" DX:");
        Serial.print(minPoints[6]);
        Serial.print(",");
        Serial.print(maxPoints[6]);
        Serial.print(" DY:");
        Serial.print(minPoints[7]);
        Serial.print(",");
        Serial.print(maxPoints[7]);
*/
        Serial.println("");

    }


    // Single command mode?
    const float minMove = 0.16f;
    if (fabs(rotZ) > minMove)
    {
        rotY = 0.0f;
        rotX = 0.0f;

        transX = 0.0f;
        transY = 0.0f;
        transZ = 0.0f;
    }
    else
    {
        rotZ = 0.0f;
    }


    if (fabs(transZ) > minMove)
    {
        rotY = 0.0f;
        rotX = 0.0f;
        rotZ = 0.0f;

        transX = 0.0f;
        transY = 0.0f;
    }
    else
    {
        transZ = 0.0f;
    }

/*    else if (fabs(transZ) > minMove)
    {
        rotX = 0.0f;
        rotY = 0.0f;
        rotZ = 0.0f;

        transX = 0.0f;
        transY = 0.0f;
    }
    else if (fabs(transX) > minMove)
    {
        rotX = 0.0f;
        rotY = 0.0f;
        rotZ = 0.0f;

        transY = 0.0f;
        transZ = 0.0f;
    }
    else if (fabs(transY) > minMove)
    {
        rotX = 0.0f;
        rotY = 0.0f;
        rotZ = 0.0f;

        transX = 0.0f;
        transZ = 0.0f;
    }
    else if (fabs(rotY) > minMove)
    {
        rotZ = 0.0f;
        rotX = 0.0f;

        transX = 0.0f;
        transY = 0.0f;
        transZ = 0.0f;
    }
    else if (fabs(rotX) > minMove)
    {
        rotZ = 0.0f;
        rotY = 0.0f;

        transX = 0.0f;
        transY = 0.0f;
        transZ = 0.0f;
    }
*/




    // Last step is to convert values to HID values (-32767...32767)
    int16_t tx = (int16_t)(transX * 32767.0f);
    int16_t ty = (int16_t)(transY * 32767.0f);
    int16_t tz = (int16_t)(transZ * 32767.0f);

    int16_t rx = (int16_t)(rotX * 32767.0f);
    int16_t ry = (int16_t)(rotY * 32767.0f);
    int16_t rz = (int16_t)(rotZ * 32767.0f);

    if (debug == 6)
    {
        Serial.print("TX:");
        Serial.print(tx);
        Serial.print(",");
        Serial.print("TY:");
        Serial.print(ty);
        Serial.print(",");
        Serial.print("TZ:");
        Serial.print(tz);
        Serial.print(",");
        Serial.print("RX:");
        Serial.print(rx);
        Serial.print(",");
        Serial.print("RY:");
        Serial.print(ry);
        Serial.print(",");
        Serial.print("RZ:");
        Serial.print(rz);
        Serial.println("");
    }

    // Send data to the 3DConnexion software.
    send_command(rx, rz, ry, tx, tz, ty);

    // Under Linux, using "spacenavd", it seems like the reports must "differ" for a report to be used,
    // So, always send the rerport, but very close the old value so that an event is tiggered
    send_command(rx-1, rz-1, ry-1, tx-1, tz-1, ty-1);


#if 0


    // Doing all through arithmetic contribution by fdmakara
    // Integer has been changed to 16 bit int16_t to match what the HID protocol expects.
    int16_t transX, transY, transZ, rotX, rotY, rotZ; // Declare movement variables at 16 bit integers
    // Original fdmakara calculations
    // transX = (-centered[AX] +centered[CX])/1;
    // transY = (-centered[BX] +centered[DX])/1;
    // transZ = (-centered[AY] -centered[BY] -centered[CY] -centered[DY])/2;
    // rotX = (-centered[AY] +centered[CY])/2;
    // rotY = (+centered[BY] -centered[DY])/2;
    // rotZ = (+centered[AX] +centered[BX] +centered[CX] +centered[DX])/4;
    // My altered calculations based on debug output. Final divisor can be changed to alter sensitivity for each axis.

    transX = -(-centered[CY] + centered[AY]) / 1;
    transY = (-centered[BY] + centered[DY]) / 1;
    if ((abs(centered[AX]) > DEADZONE) && (abs(centered[BX]) > DEADZONE) && (abs(centered[CX]) > DEADZONE) && (abs(centered[DX]) > DEADZONE))
    {
        transZ = (-centered[AX] - centered[BX] - centered[CX] - centered[DX]) / 2;
        transX = 0;
        transY = 0;
    }
    else
    {
        transZ = 0;
    }
    rotX = (-centered[AX] + centered[CX]) / 1;
    rotY = (+centered[BX] - centered[DX]) / 1;
    if ((abs(centered[AY]) > DEADZONE) && (abs(centered[BY]) > DEADZONE) && (abs(centered[CY]) > DEADZONE) && (abs(centered[DY]) > DEADZONE))
    {
        rotZ = (+centered[AY] + centered[BY] + centered[CY] + centered[DY]) / 2;
        rotX = 0;
        rotY = 0;
    }
    else
    {
        rotZ = 0;
    }

    // Alter speed to suit user preference - Use 3DConnexion slider instead for V2.
    // transX = transX/100*speed;
    // transY = transY/100*speed;
    // transZ = transZ/100*speed;
    // rotX = rotX/100*speed;
    // rotY = rotY/100*speed;
    // rotZ = rotZ/100*speed;
    // Invert directions if needed
    if (invX == true)
    {
        transX = transX * -1;
    };
    if (invY == true)
    {
        transY = transY * -1;
    };
    if (invZ == true)
    {
        transZ = transZ * -1;
    };
    if (invRX == true)
    {
        rotX = rotX * -1;
    };
    if (invRY == true)
    {
        rotY = rotY * -1;
    };
    if (invRZ == true)
    {
        rotZ = rotZ * -1;
    };


    // Special case... 
    // if (abs(rotZ) > 400)
    // {
    //     transX = 0;
    //     transY = 0;
    //     transZ = 0;
    //     rotY = 0;
    //     rotX = 0;
    // }

    if ( (abs(transX) < DEADZONE) && (abs(transY) < DEADZONE) && (abs(transZ) < DEADZONE) )
    {
        rotX = map(rotX, -1024, 1024, -32768, 32767);
        rotZ = map(rotZ, -1024, 1024, -32768, 32767);
        rotY = map(rotY, -1024, 1024, -32768, 32767);
    }

    transX = map(transX, -1024, 1024, -32768, 32767);
    transZ = map(transZ, -1024, 1024, -32768, 32767);
    transY = map(transY, -1024, 1024, -32768, 32767);

    float transScale = 0.25f;
    float rotScale = 0.25f;

    transX = transScale*transX;
    transY = transScale*transY;
    transZ = transScale*transZ;

    rotX = rotScale*rotX;
    rotY = rotScale*rotY;
    rotZ = rotScale*rotZ;

    // Report translation and rotation values if enabled. Approx -800 to 800 depending on the parameter.
    if (debug == 4)
    {
        Serial.print("TX:");
        Serial.print(transX);
        Serial.print(",");
        Serial.print("TY:");
        Serial.print(transY);
        Serial.print(",");
        Serial.print("TZ:");
        Serial.print(transZ);
        Serial.print(",");
        Serial.print("RX:");
        Serial.print(rotX);
        Serial.print(",");
        Serial.print("RY:");
        Serial.print(rotY);
        Serial.print(",");
        Serial.print("RZ:");
        Serial.println(rotZ);
    }
    // Report debug 4 and 5 info side by side for direct reference if enabled. Very useful if you need to alter which inputs are used in the arithmatic above.
    if (debug == 5)
    {
        Serial.print("AX:");
        Serial.print(centered[0]);
        Serial.print(",");
        Serial.print("AY:");
        Serial.print(centered[1]);
        Serial.print(",");
        Serial.print("BX:");
        Serial.print(centered[2]);
        Serial.print(",");
        Serial.print("BY:");
        Serial.print(centered[3]);
        Serial.print(",");
        Serial.print("CX:");
        Serial.print(centered[4]);
        Serial.print(",");
        Serial.print("CY:");
        Serial.print(centered[5]);
        Serial.print(",");
        Serial.print("DX:");
        Serial.print(centered[6]);
        Serial.print(",");
        Serial.print("DY:");
        Serial.print(centered[7]);
        Serial.print("||");
        Serial.print("TX:");
        Serial.print(transX);
        Serial.print(",");
        Serial.print("TY:");
        Serial.print(transY);
        Serial.print(",");
        Serial.print("TZ:");
        Serial.print(transZ);
        Serial.print(",");
        Serial.print("RX:");
        Serial.print(rotX);
        Serial.print(",");
        Serial.print("RY:");
        Serial.print(rotY);
        Serial.print(",");
        Serial.print("RZ:");
        Serial.println(rotZ);
    }


    // Send data to the 3DConnexion software.
    // The correct order for me was determined after trial and error
    send_command(rotX, rotZ, rotY, transX, transZ, transY);
    //delay(1);
    send_command(rotX-1, rotZ-1, rotY-1, transX-1, transZ-1, transY-1);
    //delay(1);

#endif
}