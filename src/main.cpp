// This code is bases of Teaching Tech's project (which is also based on varios people, see below)
// https://www.printables.com/model/864950-open-source-spacemouse-space-mushroom-remix

#include "HID.h"

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
    for (int p = 0; p < 100; p++)
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

// Deadband threshold
#define DEADBAND_THRESHOLD 0.05f

// Low-pass filter constants
#define ALPHA 0.1f // Smoothing factor (0 < ALPHA < 1)

// Function to apply a deadband filter
void apply_deadband(float *x, float *y)
{
    if (fabs(*x) < DEADBAND_THRESHOLD)
        *x = 0;
    if (fabs(*y) < DEADBAND_THRESHOLD)
        *y = 0;
}

// Function to apply a low-pass filter
void apply_low_pass_filter(float *prev, float current)
{
    *prev = ALPHA * current + (1 - ALPHA) * (*prev);
}

// Function to calculate translation and rotation with filtering
void calculate_filtered_translation_rotation(const float joystick_inputs[4][2], float translation[3], float rotation[3])
{
    static float prev_x[4] = {0}; // Previous X-axis values for filtering
    static float prev_y[4] = {0}; // Previous Y-axis values for filtering

    float filtered_inputs[4][2];
    float total_force_x = 0;
    float total_force_y = 0;
    float total_force_z = 0;
    float total_torque[3] = {0};

    // Apply deadband and low-pass filters to joystick inputs
    for (int i = 0; i < 4; ++i)
    {
        // Apply deadband filter
        float x = joystick_inputs[i][0];
        float y = joystick_inputs[i][1];
        apply_deadband(&x, &y);

        // Apply low-pass filter
        apply_low_pass_filter(&prev_x[i], x);
        apply_low_pass_filter(&prev_y[i], y);

        // Use filtered values
        filtered_inputs[i][0] = prev_x[i];
        filtered_inputs[i][1] = prev_y[i];

        // Calculate forces and torques based on filtered inputs
        x = filtered_inputs[i][0];
        y = filtered_inputs[i][1];

        float fx, fy;

        if (i == 0)
        { // 0 degrees (forward/backward)
            fx = -y;
            fy = x;
            total_torque[0] -= x * 12.5f; // Rotate around X-axis (tilting forward/backward)
        }
        else if (i == 1)
        { // 90 degrees (right/left)
            fx = -x;
            fy = -y;
            total_torque[1] += x * 12.5f; // Rotate around Y-axis (tilting left/right)
        }
        else if (i == 2)
        { // 180 degrees (backward/forward)
            fx = y;
            fy = -x;
            total_torque[0] += x * 12.5f; // Rotate around X-axis (tilting backward/forward)
        }
        else
        { // 270 degrees (left/right)
            fx = x;
            fy = y;
            total_torque[1] -= x * 12.5f; // Rotate around Y-axis (tilting right/left)
        }

        total_force_x += fx;
        total_force_y += fy;
        total_force_z += filtered_inputs[i][0]; // X-axis movement contributes to Z translation

        // For rotation around Z-axis (twisting)
        total_torque[2] += y * 12.5f; // Y-axis movement contributes to rotation around Z-axis
    }

    // Set translation and rotation results
    translation[0] = total_force_x;
    translation[1] = total_force_y;
    translation[2] = total_force_z / 4; // Averaging the Z translation

    rotation[0] = total_torque[0];     // Rotation around X-axis
    rotation[1] = total_torque[1];     // Rotation around Y-axis
    rotation[2] = total_torque[2] / 4; // Averaging the rotation around Z-axis
}

void loop()
{
    const float f_invX = -1.0f;  // pan left/right
    const float f_invY = -1.0f;  // pan up/down
    const float f_invZ = -1.0f;  // zoom in/out
    const float f_invRX = 1.0f; // Rotate around X axis (tilt front/back)
    const float f_invRY = -1.0f; // Rotate around Y axis (tilt left/right)
    const float f_invRZ = -1.0f; // Rotate around Z axis (twist left/right)

    const float boostTX = 1.0f;
    const float boostTY = 1.0f;
    const float boostTZ = 1.0f;

    const float boostRX = 0.2f;
    const float boostRY = 0.2f;
    const float boostRZ = 0.2f;

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

    // Default Assembly when looking from above:
    //    C           Y+
    //    |           .
    // B--+--D   X-...Z+...X+
    //    |           .
    //    A           Y-
    //
    // A = 180, B=270, D=90, C=0
    float joystick_inputs[4][2] = {
        {centered[4], centered[5]},
        {centered[6], centered[7]},
        {centered[0], centered[1]},
        {centered[2], centered[3]}};

    float translation[3];
    float rotation[3];

    calculate_filtered_translation_rotation(joystick_inputs, translation, rotation);

    // Boost some values as we are not using the full range of the joysticks (some times)
    // Boost and invert any results
    translation[0] = constrain(boostTX * f_invX * translation[0], -1.0f, 1.0f);
    translation[1] = constrain(boostTY * f_invY * translation[1], -1.0f, 1.0f);
    translation[2] = constrain(boostTZ * f_invZ * translation[2], -1.0f, 1.0f);

    rotation[0] = constrain(boostRX * f_invRX * rotation[0], -1.0f, 1.0f);
    rotation[1] = constrain(boostRY * f_invRY * rotation[1], -1.0f, 1.0f);
    rotation[2] = constrain(boostRZ * f_invRZ * rotation[2], -1.0f, 1.0f);

    // Last step is to convert values to HID values (-32767...32767)
    int16_t tx = (int16_t)(translation[0] * 32767.0f);
    int16_t ty = (int16_t)(translation[1] * 32767.0f);
    int16_t tz = (int16_t)(translation[2] * 32767.0f);

    int16_t rx = (int16_t)(rotation[0] * 32767.0f);
    int16_t ry = (int16_t)(rotation[1] * 32767.0f);
    int16_t rz = (int16_t)(rotation[2] * 32767.0f);

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
        Serial.print(" || ");

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
    send_command(rx - 1, rz - 1, ry - 1, tx - 1, tz - 1, ty - 1);
}
