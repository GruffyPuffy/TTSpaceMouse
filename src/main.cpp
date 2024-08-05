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

// Movement thresholds
const float trans_threshold[3] = {0.1f, 0.1f, 0.02f};
const float rot_threshold[3] = {0.03f, 0.03f, 0.03f};

// Axes inverting etc
const float f_invX = -1.0f;  // pan left/right
const float f_invY = -1.0f;  // pan up/down
const float f_invZ = -1.0f;  // zoom in/out
const float f_invRX = 1.0f;  // Rotate around X axis (tilt front/back)
const float f_invRY = -1.0f; // Rotate around Y axis (tilt left/right)
const float f_invRZ = -1.0f; // Rotate around Z axis (twist left/right)

// Change this value to decrease/increase speed of movment.
const float allBoost = 0.2f;

// Apply different speed factors for all directions/translations
const float boostTX = 0.4f*allBoost;
const float boostTY = 0.4f*allBoost;
const float boostTZ = 0.8f*allBoost;

const float boostRX = 1.8f*allBoost;
const float boostRY = 1.8f*allBoost;
const float boostRZ = 2.8f*allBoost;

// Deadband threshold
#define DEADBAND_THRESHOLD 0.00f

// Low-pass filter constants
#define ALPHA 0.4f // Smoothing factor (0 < ALPHA < 1)

// Movement state for tracking previous movements and rotations
typedef struct
{
    float translation_detected[3];
    float rotation_detected[3];

} MovementState;



// Centerpoint variable to be populated during setup routine.
int centerPoints[8];

// Function to read and store analogue voltages for each joystick axis.
void readAllFromJoystick(int *rawReads)
{
    for (int i = 0; i < 8; i++)
    {
        rawReads[i] = analogRead(PINLIST[i]);
    }
}

void setup()
{
    // HID protocol is set.
    static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
    HID().AppendDescriptor(&node);

    Serial.begin(115200);

    // Calibration of center position (3*100 => 300ms)
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

// Send data on the USB endpoint
// Pack int16_t into uint8_t bytes.
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

// Function to apply a deadband filter
void apply_deadband(float *x, float *y)
{
    if (fabs(*x) < DEADBAND_THRESHOLD)
    {
        *x = 0;
    }

    if (fabs(*y) < DEADBAND_THRESHOLD)
    {
        *y = 0;
    }
}

// Function to apply a low-pass filter
void apply_low_pass_filter(float *prev, float current)
{
    *prev = ALPHA * current + (1 - ALPHA) * (*prev);
}

// Function to calculate translation and rotation with filtering
void calculate_translation_rotation(const float joystick_inputs[4][2], float translation[3], float rotation[3])
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

        // Distance from rotation point. In reality this is just boosting rotations, set to 1.0f for now.
        const float distance = 1.0f;

        if (i == 0)
        { 
            // 0 degrees (forward/backward)
            fx = -y;
            fy = x;
            total_torque[0] -= x * distance; // Rotate around X-axis (tilting forward/backward)
        }
        else if (i == 1)
        { 
            // 90 degrees (right/left)
            fx = -x;
            fy = -y;
            total_torque[1] += x * distance; // Rotate around Y-axis (tilting left/right)
        }
        else if (i == 2)
        { 
            // 180 degrees (backward/forward)
            fx = y;
            fy = -x;
            total_torque[0] += x * distance; // Rotate around X-axis (tilting backward/forward)
        }
        else
        { 
            // 270 degrees (left/right)
            fx = x;
            fy = y;
            total_torque[1] -= x * distance; // Rotate around Y-axis (tilting right/left)
        }

        total_force_x += fx;
        total_force_y += fy;
        total_force_z += filtered_inputs[i][0]; // X-axis movement contributes to Z translation

        // For rotation around Z-axis (twisting)
        total_torque[2] += y * distance; // Y-axis movement contributes to rotation around Z-axis
    }

    // Set translation and rotation results
    translation[0] = total_force_x;
    translation[1] = total_force_y;
    translation[2] = total_force_z / 4; // Averaging the Z translation

    rotation[0] = total_torque[0];     // Rotation around X-axis
    rotation[1] = total_torque[1];     // Rotation around Y-axis
    rotation[2] = total_torque[2] / 4; // Averaging the rotation around Z-axis
}

void detect_type_of_move(const float translation[3], const float rotation[3], MovementState *state)
{
    const float trans_detect[3] = {0.05f, 0.05f, 0.25f};
    const float rot_detect[3] = {0.05f, 0.05f, 0.10f};

    for (int t=0; t<3; t++)
    {
        if (fabs(translation[t]) > trans_detect[t])
        {
            state->translation_detected[t] = 1.0;
        }
        else
        {
            state->translation_detected[t] -= 0.01;
        }

        state->translation_detected[t] = constrain(state->translation_detected[t], 0.0f, 1.0f);

    }

    for (int r=0; r<3; r++)
    {
        if (fabs(rotation[r]) > rot_detect[r])
        {
            state->rotation_detected[r] = 1.0;
        }
        else
        {
            state->rotation_detected[r] -= 0.01;
        }

        state->rotation_detected[r] = constrain(state->rotation_detected[r], 0.0f, 1.0f);
    }

}

void loop()
{
    // Static to keep between loop() calls
    static MovementState state;

    // Internals
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
    // A=180, B=270, D=90, C=0
    float joystick_inputs[4][2] = {
        {centered[4], centered[5]},
        {centered[6], centered[7]},
        {centered[0], centered[1]},
        {centered[2], centered[3]}};

    float translation[3];
    float rotation[3];

    // Convert from Joystick to translation/rotation
    calculate_translation_rotation(joystick_inputs, translation, rotation);

    const float trans_threshold[3] = {0.75f, 0.75f, 0.75f};
    const float rot_threshold[3] = {0.75f, 0.75f, 0.75f};

    detect_type_of_move(translation, rotation, &state);

    Serial.print("Moving: ");

    //if (state.translation_detected[0] > trans_threshold[0])
    {
        Serial.print(" TX:");
        //Serial.print(state.last_translation[0]);
        Serial.print(translation[0]);
    }
    //if (state.translation_detected[1] > trans_threshold[1])
    {
        Serial.print(" TY:");
        //Serial.print(state.last_translation[1]);
        Serial.print(translation[1]);
    }
    //if (state.translation_detected[2] > trans_threshold[2])
    {
        Serial.print(" TZ:");
        //Serial.print(state.last_translation[2]);
        Serial.print(translation[2]);
    }


    //if (state.rotation_detected[0] > rot_threshold[0])
    {
        Serial.print(" RX:");
        //Serial.print(state.last_rotation[0]);
        Serial.print(rotation[0]);
    }
    //if (state.rotation_detected[1] > rot_threshold[1])
    {
        Serial.print(" RY:");
        //Serial.print(state.last_rotation[1]);
        Serial.print(rotation[1]);
    }
    //if (state.rotation_detected[2] > rot_threshold[2])
    {
        Serial.print(" RZ:");
        //Serial.print(state.last_rotation[2]);
        Serial.print(rotation[2]);
    }

    // Apply priority 
    if (state.translation_detected[2] > trans_threshold[2])
    {
        float damp = 0.0f;

        translation[0] = damp * translation[0];
        translation[1] = damp * translation[1];

        rotation[0] = damp * rotation[0];
        rotation[1] = damp * rotation[1];
        rotation[2] = damp * rotation[2];
        Serial.print(" ---> PUSH/PULL");
    }
    else if (state.rotation_detected[2] > rot_threshold[2])
    {
        float damp = 0.02f;

        translation[0] = damp * translation[0];
        translation[1] = damp * translation[1];
        translation[2] = damp * translation[2];

        rotation[0] = damp * rotation[0];
        rotation[1] = damp * rotation[1];
        Serial.print(" ---> TWIST");
    }
    else if ((state.rotation_detected[0] > rot_threshold[0]) || (state.rotation_detected[1] > rot_threshold[1]))
    {
        float damp = 0.20f;

        translation[0] = damp * translation[0];
        translation[1] = damp * translation[1];
        translation[2] = damp * translation[2];

        rotation[2] = damp * rotation[2];
        Serial.print(" ---> ROTATION");
    }
    else if ((state.translation_detected[0] > trans_threshold[0]) || (state.translation_detected[1] > trans_threshold[1]))
    {
        float damp = 0.20f;

        translation[2] = damp * translation[2];

        rotation[0] = damp * rotation[0];
        rotation[1] = damp * rotation[1];
        rotation[2] = damp * rotation[2];

        Serial.print(" ---> TRANSLATION");
    }
    else
    {
        Serial.print(" ---> IDLE");
    }

    Serial.println();

    // Boost some values as we are not using the full range of the joysticks (some times)
    // Boost and invert any results
    translation[0] = constrain(boostTX * f_invX * translation[0], -1.0f, 1.0f);
    translation[1] = constrain(boostTY * f_invY * translation[1], -1.0f, 1.0f);
    translation[2] = constrain(boostTZ * f_invZ * translation[2], -1.0f, 1.0f);

    rotation[0] = constrain(boostRX * f_invRX * rotation[0], -1.0f, 1.0f);
    rotation[1] = constrain(boostRY * f_invRY * rotation[1], -1.0f, 1.0f);
    rotation[2] = constrain(boostRZ * f_invRZ * rotation[2], -1.0f, 1.0f);


    const bool forLinuxDebug = true;

    if (forLinuxDebug)
    {
        // Last step is to convert values 
        int16_t tx = (int16_t)(translation[0] * 20000.0f);
        int16_t ty = (int16_t)(translation[1] * 20000.0f);
        int16_t tz = (int16_t)(translation[2] * 20000.0f);

        int16_t rx = (int16_t)(rotation[0] * 20000.0f);
        int16_t ry = (int16_t)(rotation[1] * 20000.0f);
        int16_t rz = (int16_t)(rotation[2] * 20000.0f);

        // Send data to the 3DConnexion software.
        // SWAP order to match 

        send_command(rx, rz, ry, tx, tz, ty);

        delay(10);

        // This part is added when running on
        // Linux with spacenavd. For some reason,
        // not known to me, this does not work
        // unless you "change" the values. You need
        // to keep sending different values for it to
        // trigger. This is not needed on Windows, 
        // and normal joystick users... 
        // Make sure to disable when using outside
        // spacenavd environment.
        send_command(rx+1, rz+1, ry+1, tx+1, tz+1, ty+1);

    }
    else
    {
        int16_t tx = (int16_t)(translation[0] * 1023.0f);
        int16_t ty = (int16_t)(translation[1] * 1023.0f);
        int16_t tz = (int16_t)(translation[2] * 1023.0f);

        int16_t rx = (int16_t)(rotation[0] * 1023.0f);
        int16_t ry = (int16_t)(rotation[1] * 1023.0f);
        int16_t rz = (int16_t)(rotation[2] * 1023.0f);

        // Send data to the 3DConnexion software.
        // SWAP order to match 

        send_command(rx, rz, ry, tx, tz, ty);

        delay(10);
    }

}
