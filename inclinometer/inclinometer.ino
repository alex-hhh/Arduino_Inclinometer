/*
 * Arduino Inclinometer Project, main sketch
 *
 * Copyright (C) 2017 Alex Harsanyi <alexharsanyi@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_ADXL345_U.h>
#include <EEPROM.h>


// .................................................... User Parameters ....

/** Pitch angle (in degrees) above which the inclinometer will beep and blink
 * on screen the pitch value.
 */
#define PITCH_WARN (35)

/** Roll angle (in degrees) above which the inclinometer will beep and blink
 * on screen the roll value.
 */
#define ROLL_WARN (35)

/** Blink (also beep) interval in microseconds.  This is the interval by which
 * the inclinometer will beep and blink when the pitch or roll angles exceed
 * their "warn" values defined above.
 */
#define BLINK_INTERVAL (300000)

/** Amount of time, in microseconds, needed to hold a button (reset or
 * calibrate) in order to perform the action.  This prevents starting
 * calibration or resetting the max values accidentally.
 */
#define HOLD_INTERVAL (1000000)

/** Low pass filter RC constant for filtering acceleration values, time in
 * microseconds.  Larger values will make the display look more stable, but it
 * will take longer to display the true roll and pitch.
 */
#define LPF_ALPHA (500000.0)

// Calibration parameters.  These will need to be updated for each unit, see
// the Calibration section in README.md

#define X_SLOPE 0.003984064
#define X_INTERCEPT -0.047808765
#define Y_SLOPE 0.003937008
#define Y_INTERCEPT 0.07480315
#define Z_SLOPE 0.003944773
#define Z_INTERCEPT 0.065088757


// ............................................. Hardware Configuration ....

// #define WANT_DEBUG_OUTPUT

#define BUZZER_PIN 8
#define RESET_PIN 11
#define CALIBRATE_PIN 12

#define OLED_SCL   6
#define OLED_SDA   5
#define OLED_RES   4
#define OLED_DC    3
#define OLED_CS    12                   // not connected

/** Place (address) in EEPROM where we store the calibration data
 */
#define EEPROM_CALIBRATION_ADDRESS (0)


// ......................................... Program Data and Functions ....

/** When set, the max pitch and roll values are not updated and they will also
 * blink
 */
#define HOLD_MAX_FLAG 0x10

/** When set, the PITCH has exceeded the warning threshold -- the pitch angle
 * will be displayed blinking and the warning tone will sound. 
 */
#define PITCH_WARN_FLAG 0x20

/** When set, the ROLL has exceeded the warning threshold -- the roll angle
 * will be displayed blinking and the warning tone will sound. 
 */
#define ROLL_WARN_FLAG  0x40

/** Internal flag to control blinking.  Toggles on and off automatically at
 * predefined intervals. 
 */
#define BLINK_ON_FLAG   0x80

/** State values for the accelerometer application.  See main loop() function.
 */
#define STATE_ACQUIRE_DOWN_DIRECTION 0
#define STATE_ACQUIRE_FORWARD_DIRECTION 1
#define STATE_RUNNING 2

#define STATE_MASK 0x0F

uint8_t status_flags = 0;

#define SET_FLAG(flag) do { status_flags |= (flag); } while(0)
#define CLR_FLAG(flag) do { status_flags &= ~(flag); } while (0)
#define TOGGLE_FLAG(flag) do { status_flags ^= flag; } while (0)
#define IS_FLAG_SET(flag) ((status_flags & (flag)) == (flag))
#define IS_FLAG_SET_ALL(flags) ((status_flags & (flags)) == (flags))
#define IS_FLAG_SET_ANY(flags) ((status_flags & (flags)) != 0)
#define SET_STATE(s) do { status_flags = (status_flags & ~STATE_MASK) | s; } while(0)

/** Timestamp (value of micros()) when the last time loop() was called.  Used
 * by `update_timer` to measure delta times and keep track of the beep time.
 */
uint32_t last_loop_start = 0;

/** Time interval in microseconds since the of the last loop() call. This is
 * used to calculate accurate alpha values for low pass filters.  Note that
 * this is a 16bit value, allowing for a maximum of approx 65 milliseconds
 * between subsequent loop() calls.
 */
uint16_t delta_time;

/** Accumulated time in the current beep and blink period, see
 * `update_timer()` for how this is used.
 */
uint32_t blink_accum_time = 0;

/** Accumulated time the reset button is held down for.
 */
int32_t reset_hold_time = 0;

/** Accumulated time the calibrate button is held down for.
 */
int32_t calibrate_hold_time = 0;

/** X, Y and Z axis for the installed accelerometer position.  These are used
 * to determine the roll and pitch regardless of the position in which the
 * unit is installed.  They are set during the calibration process, see
 * on_acquire_down_direction() and on_acquire_forward_direction()
 *
 * This data is also saved to EEPROM and restored when the unit starts up.
 */
struct orientation_t {
    float xaxis[3];
    float yaxis[3];
    float zaxis[3];
} orientation;

/** Maximum pitch and roll angles the unit has recorded.  These are
 * continuously updated UNLESS the HOLD_MAX_FLAG is set.  They can be reset if
 * the user presses the reset button for a HOLD_INTERVAL microseconds.  See
 * handle_reset_button().
 */
float max_roll = 0;
float max_pitch = 0;

/** The OLED display access class
 */
Adafruit_SSD1306 oled(OLED_SDA, OLED_SCL, OLED_DC, OLED_RES, OLED_CS);

/** The ADXL345 accelerometer access class
 */
Adafruit_ADXL345_Unified adxl345;


// ........................................................ vector math ....

float vlen_squared(float vec[3])
{
    return vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2];
}

float vlen(float vec[3])
{
    return sqrt(vlen_squared(vec));
}

void vzero(float vec[3])
{
    vec[0] = vec[1] = vec[2] = 0.0;
}

float* vnormalize(float vec[3], float vec_out[3])
{
    float m = vlen(vec);
    vec_out[0] = vec[0] / m;
    vec_out[1] = vec[1] / m;
    vec_out[2] = vec[2] / m;
    return vec_out;
}

float vdot(float vec_a[3], float vec_b[3])
{
    return vec_a[0] * vec_b[0] + vec_a[1] * vec_b[1] + vec_a[2] * vec_b[2];
}

float* vcross(float vec_a[3], float vec_b[3], float vec_out[3])
{
    float x = vec_a[1] * vec_b[2] - vec_a[2] * vec_b[1];
    float y = vec_a[2] * vec_b[0] - vec_a[0] * vec_b[2];
    float z = vec_a[0] * vec_b[1] - vec_a[1] * vec_b[0];
    vec_out[0] = x;
    vec_out[1] = y;
    vec_out[2] = z;
    return vec_out;
}

#ifdef WANT_DEBUG_OUTPUT
void vprint(float vec[3], const char *name = nullptr, bool nl = false)
{
    if (name) {
        Serial.print(name);
        Serial.print(": ");
    }
    Serial.print("[");
    Serial.print(vec[0]);
    Serial.print(", ");
    Serial.print(vec[1]);
    Serial.print(", ");
    Serial.print(vec[2]);
    Serial.print("]");
    if (nl) {
        Serial.print("\n");
    }
}
#endif


// ...................................................  string constants ....

// https://www.arduino.cc/reference/en/language/variables/utilities/progmem/

const char string_0[] PROGMEM = "Roll";
const char string_1[] PROGMEM = "Pitch";
const char string_2[] PROGMEM = "Rolled";
const char string_3[] PROGMEM = "Over";
const char string_4[] PROGMEM = "Down";
const char string_5[] PROGMEM = "Direction";
const char string_6[] PROGMEM = "Move";
const char string_7[] PROGMEM = "Forward!";
const char string_8[] PROGMEM = "Left";
const char string_9[] PROGMEM = "Right";
const char string_10[] PROGMEM = "Up";
const char string_11[] PROGMEM = "Max";
const char string_12[] PROGMEM = "Hold to reset";
const char string_13[] PROGMEM = "Hold to calibrate";

const char* const string_table[] PROGMEM = {
    string_0, string_1, string_2, string_3,
    string_4, string_5, string_6, string_7,
    string_8, string_9, string_10, string_11,
    string_12, string_13
};

#define ROLL_STR 0
#define PITCH_STR 1
#define ROLLED_STR 2
#define OVER_STR 3
#define DOWN_STR 4
#define DIRECTION_STR 5
#define MOVE_STR 6
#define FORWARD_STR 7
#define LEFT_STR 8
#define RIGHT_STR 9
#define UP_STR 10
#define MAX_STR 11
#define HOLD_TO_RESET_STR 12
#define HOLD_TO_CALIBRATE_STR 13

/**  Display a string stored in 'string_table' at position 'x', 'y' 
 */
void oled_print_from_progmem(int text_size, int x, int y, int string_index)
{
    char buffer[20];
    oled.setTextSize(text_size);
    oled.setCursor(x, y);
    strcpy_P(buffer, (char*)pgm_read_word(&(string_table[string_index])));
    oled.print(buffer);
}


// ............................................................. eeprom ....

const uint16_t crc_table[16] PROGMEM = {
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
};

uint16_t crc_table_ref(int index)
{
    // NOTE: crc_table + index is different from crc_table[index] since
    // crc_table is in PROGMEM!
    return pgm_read_word_near(crc_table + index);
}

/** Calculate a 16 bit CRC checksum of the 'data' buffer containing 'len'
 * bytes.
 */
uint16_t calculate_crc(const uint8_t *data, int len)
{
    uint16_t crc = 0;
    while (len-- > 0) {
        uint8_t byte = *data++;
        uint16_t tmp = crc_table_ref(crc & 0x0F);
        crc = (crc >> 4) & 0x0FFF;
        crc = crc ^ tmp ^ crc_table_ref(byte & 0x0F);
        tmp = crc_table_ref(crc & 0x0F);
        crc = (crc >> 4) & 0x0FFF;
        crc = crc ^ tmp ^ crc_table_ref((byte >> 4) & 0x0F);
    }
    return crc;
}

/** Save the calibration values to EEPROM along with a CRC checksum.  Next
 * time the unit boots up it will reuse these values, avoiding the need for a
 * calibration.
 */
void save_calibration_to_eeprom()
{
    uint16_t crc = calculate_crc(
        reinterpret_cast<uint8_t*>(&orientation),
        sizeof(orientation));
    EEPROM.put(EEPROM_CALIBRATION_ADDRESS, orientation);
    EEPROM.put(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), crc);
}

/** Read calibration data from EEPROM, if the CRC of the data is valid, go
 * straight to running state, otherwise go to calibration mode.
 */
void restore_calibration_from_eeprom()
{
    uint16_t stored_crc;
    EEPROM.get(EEPROM_CALIBRATION_ADDRESS, orientation);
    EEPROM.get(EEPROM_CALIBRATION_ADDRESS + sizeof(orientation), stored_crc);
    uint16_t crc = calculate_crc(
        reinterpret_cast<uint8_t*>(&orientation),
        sizeof(orientation));

    if (crc == stored_crc) {
        SET_STATE(STATE_RUNNING);
    } else {
        // reset the Z-Axis so it is acquired again
        vzero(orientation.zaxis);
        SET_STATE(STATE_ACQUIRE_DOWN_DIRECTION);
    }
}


// .......................................................... utilities ....

/** Convert 'angle' from radians to degrees.
 */
float rad2deg(float angle)
{
    return 180.0 * (angle / 3.1415926);
}

/** Read the accelerometer X, Y and Z values and store them in 'output'.  The
 * output values are calibrated and filtered.
 */
void read_accelerometer(float output[3])
{
    // The filtered accelerometer values.  This is 'static', so it remembers
    // its values between `read_accelerometer' calls.
    static float filter[3];

    // Step 1: read the raw values from the accelerometer
    output[0] = adxl345.getX();
    output[1] = adxl345.getY();
    output[2] = adxl345.getZ();

    // Step 2: calibrate the values, see the readme.md file
    output[0] = output[0] * X_SLOPE + X_INTERCEPT;
    output[1] = output[1] * Y_SLOPE + Y_INTERCEPT;
    output[2] = output[2] * Z_SLOPE + Z_INTERCEPT;

    // Step 3: calculate the filter alpha value and update the filter.
    float alpha = float(delta_time) / (LPF_ALPHA + float(delta_time));

    filter[0] = filter[0] * (1 - alpha) + output[0] * alpha;
    filter[1] = filter[1] * (1 - alpha) + output[1] * alpha;
    filter[2] = filter[2] * (1 - alpha) + output[2] * alpha;

    // Step 4: produce the final calibrated and filtered values.
    output[0] = filter[0];
    output[1] = filter[1];
    output[2] = filter[2];
}

/** Return the pitch angle in degrees (forward - backward inclination) of the
 * vehicle based on the current "down" direction stored in 'cal'.  'cal' is
 * converted to local coordinates (accounts for the installation orientation
 * of the inclinometer itself).
 *
 * A positive pitch angle indicates that the vehicle is pointing up, a
 * negative angle indicates that the vehicle is pointing down.
 */
float calculate_pitch(float cal[3])
{
    float down[3] = {0, 0, 1};
    float pitch_dir[3] = { cal[0], 0, cal[2] };
    vnormalize(pitch_dir, pitch_dir);
    float pitch = vdot(down, pitch_dir);
    float angle = rad2deg(acos(pitch));
    if (pitch_dir[0] > 0)
        return angle;                   // up
    else
        return -angle;                  // down
}

/** Return the roll angle in degrees (left - right inclination) of the vehicle
 * based on the current "down" direction stored in 'cal'.  'cal' is converted
 * to local coordinates (accounts for the installation orientation of the
 * inclinometer itself).
 *
 * A positive roll angle indicates that the vehicle is rolling to the right, a
 * negative angle indicates that it is rolling to the left.
 */
float calculate_roll(float cal[3])
{
    float down[3] = {0, 0, 1};
    float roll_dir[3] = { 0, cal[1], cal[2] };
    vnormalize(roll_dir, roll_dir);
    float roll = vdot(down, roll_dir);
    float angle = rad2deg(acos(roll));
    if (roll_dir[1] > 0)
        return -angle;                  // right
    else
        return angle;                   // left
}


// ................................................... main application ....

/** Update blink and buzzer timers and sound the buzzer if the warn conditions
 * are set.  This function should be the first one called inside `loop()`.
 */
void update_timer()
{
    uint32_t now = micros();

    // Calculate `delta_time`, taking care of roll overs which happen
    // approximately every 70 minutes
    if (now < last_loop_start) { // roll over
        delta_time = (0xFFFFFFFF - last_loop_start) + now;
    } else {
        delta_time = now - last_loop_start;
    }

    last_loop_start = now;

    blink_accum_time += delta_time;
    if (blink_accum_time > BLINK_INTERVAL) {

        if (IS_FLAG_SET(BLINK_ON_FLAG)) {
            CLR_FLAG(BLINK_ON_FLAG);
            // If we clear BLINK_ON_FLAG, turn off the buzzer unconditionally.
            noTone(BUZZER_PIN);
        } else {
            SET_FLAG(BLINK_ON_FLAG);
            // If we set BLINK_ON_FLAG, check for any warn conditions being
            // active and turn on the buzzer if they are.
            if (IS_FLAG_SET_ANY(ROLL_WARN_FLAG | PITCH_WARN_FLAG)) {
                tone(BUZZER_PIN, 15);
            }
        }

        // We subtract BLINK_INTERVAL from blink_accum_time instead of setting
        // it to 0, so any residual time left is counted in the next interval.
        blink_accum_time -= BLINK_INTERVAL;
    }
}

/** Sound a 'times' number of short beeps.  This will pause the program
 * execution for approx 200 * times milliseconds.
 */
void multi_beep(int times)
{
    while(times--) {
        tone(BUZZER_PIN, 15);
        delay(100);
        noTone(BUZZER_PIN);
        delay(100);
    }
}

/** Check the status of the reset button: a short press will toggle the "hold
 * max" mode where the max values are not updated.  A long press will reset
 * the max values.
 */
void handle_reset_button()
{
    // NOTE: calibrate and reset pins are active low -- when the button is
    // pressed, it will read a 0 when it is released it will read a 1.

    if (digitalRead(RESET_PIN) == 1) {
        // The reset button is released.  If reset_hold_time is not 0, the
        // button was just released.  We check to see if it was pressed a
        // short amount of time, and if it was, we toggle the HOLD_MAX_FLAG
        if (reset_hold_time > (HOLD_INTERVAL / 10) && reset_hold_time < HOLD_INTERVAL) {
            TOGGLE_FLAG(HOLD_MAX_FLAG);
            tone(BUZZER_PIN, 15);
            delay(100);
            noTone(BUZZER_PIN);
        }
        reset_hold_time = 0;
    } else {
        reset_hold_time += delta_time;
    }

    // If the reset button has been held down for the required hold interval,
    // reset the max pitch and max roll.
    if (reset_hold_time > HOLD_INTERVAL) {
        max_pitch = max_roll = 0;
        reset_hold_time = -HOLD_INTERVAL; // time out it!
        // Clear any hold mode if we reset the max values
        CLR_FLAG(HOLD_MAX_FLAG);
        multi_beep(2);
    }
}

/** Check the status of the calibration button.  A long press will cause the
 * unit to switch to calibration mode.
 */
void handle_calibrate_button()
{
    if (digitalRead(CALIBRATE_PIN) == 1) {
        calibrate_hold_time = 0;
    } else {
        calibrate_hold_time += delta_time;
    }

    if (calibrate_hold_time > HOLD_INTERVAL) {
        SET_STATE(STATE_ACQUIRE_DOWN_DIRECTION);
        // reset the Z-Axis so it is acquired again
        vzero(orientation.zaxis);
        // Clear all state flags
        CLR_FLAG(HOLD_MAX_FLAG);
        CLR_FLAG(PITCH_WARN_FLAG);
        CLR_FLAG(ROLL_WARN_FLAG);
        multi_beep(3);
    }
}


void display_pitch_roll(float pitch, float roll, float gforce)
{
    oled.clearDisplay();
    oled.setTextColor(WHITE);

    // Display the "hold..." messages, if the buttons are held down, but only
    // after a small amount of time has passed.
    if (reset_hold_time > (HOLD_INTERVAL / 4)) {
        oled_print_from_progmem(1, 10, 5, HOLD_TO_RESET_STR);
    } else if (calibrate_hold_time > (HOLD_INTERVAL / 4)) {
        oled_print_from_progmem(1, 10, 5, HOLD_TO_CALIBRATE_STR);
    }

    if (abs(roll) > 80 || abs(pitch) > 80) {
        oled_print_from_progmem(2, 20, 27, ROLLED_STR);
        oled_print_from_progmem(2, 20, 50, OVER_STR);
    } else {
        // NOTE: We don't show the roll or pitch angles when the warning
        // condition are set and the BLINK_ON_FLAG is cleared -- this creates
        // the blinking effect of the value.

        if (! IS_FLAG_SET(ROLL_WARN_FLAG) || IS_FLAG_SET(BLINK_ON_FLAG)) {
            oled.setTextSize(2);
            oled.setCursor(10, 22);
            oled.print(abs(roll), 0);
            oled.print("\t");                // \t displays the degree sign
        }

        oled_print_from_progmem(1, 55, 22, ROLL_STR);
        if (abs(roll) >= 1) {
            oled_print_from_progmem(1, 55, 32, roll > 0 ? RIGHT_STR : LEFT_STR);
        }

        oled_print_from_progmem(1, 100, 22, MAX_STR);

        if (! IS_FLAG_SET(HOLD_MAX_FLAG) || IS_FLAG_SET(BLINK_ON_FLAG)) {
            oled.setCursor(100, 32);
            oled.print(max_roll, 0);
            oled.print("\t");                // \t displays the degree sign
        }

        if (! IS_FLAG_SET(PITCH_WARN_FLAG) || IS_FLAG_SET(BLINK_ON_FLAG)) {
            oled.setTextSize(2);
            oled.setCursor(10, 45);
            oled.print(abs(pitch), 0);
            oled.print("\t");
        }

        oled_print_from_progmem(1, 55, 45, PITCH_STR);
        if (abs(pitch) >= 1) {
            oled_print_from_progmem(1, 55, 55, pitch > 0 ? DOWN_STR : UP_STR);
        }

        oled_print_from_progmem(1, 100, 45, MAX_STR);

        if (! IS_FLAG_SET(HOLD_MAX_FLAG) || IS_FLAG_SET(BLINK_ON_FLAG)) {
            oled.setCursor(100, 55);
            oled.print(max_pitch, 0);
            oled.print("\t");                // \t displays the degree sign
        }
    }

    oled.display();
}

/** Determine which way is down.  We assume that the vehicle is on level
 * ground.
 *
 * We wait for the accelerometer to stabilize than the reading cal[] reading
 * becomes the 'zaxis'.
 */
void on_acquire_down_direction(float cal[3])
{
    float dot = vdot(cal, orientation.zaxis);
    if (dot > 0.99 && dot < 1.01) {
        vnormalize(orientation.zaxis, orientation.zaxis);

        // Got our down direction, we can now determine the forward direction.
        SET_STATE(STATE_ACQUIRE_FORWARD_DIRECTION);
    }
    else {
        orientation.zaxis[0] = (orientation.zaxis[0] + cal[0]) * 0.5;
        orientation.zaxis[1] = (orientation.zaxis[1] + cal[1]) * 0.5;
        orientation.zaxis[2] = (orientation.zaxis[2] + cal[2]) * 0.5;

        // Show a message on the screen that we are acquiring the down
        // direction.
        oled.clearDisplay();

        oled.setTextSize(2);
        oled.setTextColor(WHITE);

        oled.setCursor(5, 5);
        oled.print(dot, 2);
        oled_print_from_progmem(2, 5, 27, DOWN_STR);
        oled_print_from_progmem(2, 5, 50, DIRECTION_STR);
        oled.display();
    }
}

/** Determine which way is forward ('xaxis'), and which way is left ('yaxis').
 *
 * We do this by asking the user to drive forward or break.  This will give us
 * a vector of non-unit magnitude which points down and slightly backwards or
 * forwards (but not directly backwards or forwards!).  This vector, together
 * with the `zaxis' determines he forward yaw plane and the cross product of
 * these vectors is the left direction ('yaxis').  Once we have he 'yaxis' and
 * the 'zaxis' we can determine which way is forward using the cross product
 * between the 'yaxis' and the 'zaxis'.
 */
void on_acquire_forward_direction(float cal[3])
{
    float gforce = vlen(cal);

    if (gforce > 1.02) {
        // 0.02 of acceleration indicates that the vehicle is moving and we
        // can determine the forward direction.  Note that on Earth, the
        // vehicle will always be subjected to at leas 1g due to gravity
        // (unless the vehicle is in free fall).
        float left[3];
        vcross(orientation.zaxis, cal, left);
        vcross(left, orientation.zaxis, orientation.xaxis);
        vcross(orientation.zaxis, orientation.xaxis, orientation.yaxis);
        vnormalize(orientation.xaxis, orientation.xaxis);
        vnormalize(orientation.yaxis, orientation.yaxis);
        save_calibration_to_eeprom();
        SET_STATE(STATE_RUNNING);
    } else {
        // The vehicle is not accelerating fast enough yet.  Show a message on
        // the screen instructing the user to move the vehicle forward.
        oled.clearDisplay();

        oled.setTextSize(2);
        oled.setTextColor(WHITE);

        oled.setCursor(10, 5);
        oled.print(gforce, 2);
        oled.setCursor(65, 5);
        oled.print("G");

        oled_print_from_progmem(2, 10, 27, MOVE_STR);
        oled_print_from_progmem(2, 10, 50, FORWARD_STR);

        oled.display();
    }
}

/** Display vehicle roll and pitch based on the current accelerometer reading
 * in 'cal'.
 */
void on_running(float cal[3])
{
    // 'cal' is in world coordinates, transform it to local coordinates, to
    // calculate the calibrated roll and pitch.  Note that the vdot() calls
    // together make a matrix -- vector multiplication.
    float ncal[3];
    ncal[0] = vdot(orientation.xaxis, cal);
    ncal[1] = vdot(orientation.yaxis, cal);
    ncal[2] = vdot(orientation.zaxis, cal);

    float pitch = calculate_pitch(ncal);
    float roll = calculate_roll(ncal);
    float gforce = vlen(cal);

    // Store max pitch an roll that we have seen.  Unlike the pitch and roll
    // which can be negative (for left roll or for down pitch), the max_pitch
    // and max_roll are always positive (i.e. we store the max angle we have
    // seen, regardless of direction).  We don't update them when the vehicle
    // is accelerating, as this gives unrealistic angles.

    if (gforce < 1.01 && ! IS_FLAG_SET(HOLD_MAX_FLAG)) {
        if (abs(pitch) > max_pitch) {
            max_pitch = abs(pitch);
        }
        if (abs(roll) > max_roll) {
            max_roll = abs(roll);
        }
    }

    // Check for WARN conditions being met and set or clear appropriate flags.
    // We set the warn flag when an angle exceeds the warn value, but only
    // clear it when it drops two degrees below that value.  This ensures that
    // there is no annoying quick on-off buzzer when the inclinometer hovers
    // around the warn value.

    if (abs(roll) > ROLL_WARN) {
        SET_FLAG(ROLL_WARN_FLAG);
    } else if (abs(roll) < (ROLL_WARN - 2)) { // note the -2, hysteresis !
        CLR_FLAG(ROLL_WARN_FLAG);
    }

    if (abs(pitch) > PITCH_WARN) {
        SET_FLAG(PITCH_WARN_FLAG);
    } else if (abs(pitch) < (PITCH_WARN - 2)) { // note the -2, hysteresis !
        CLR_FLAG(PITCH_WARN_FLAG);
    }

    display_pitch_roll(pitch, roll, gforce);
}

void setup()
{
#ifdef WANT_DEBUG_OUTPUT
    Serial.begin(38400);
#endif
    pinMode(BUZZER_PIN, OUTPUT);
    // Sound the buzzer during initialization, this acts as a check that the
    // buzzer is working.
    tone(BUZZER_PIN, 15);

    pinMode(RESET_PIN, INPUT);
    pinMode(CALIBRATE_PIN, INPUT);
    // put your setup code here, to run once:
    adxl345.begin();
    adxl345.setRange(ADXL345_RANGE_4_G);
    adxl345.setDataRate(ADXL345_DATARATE_50_HZ);
    // by default, we'll generate the high voltage from the 3.3v line
    // internally! (neat!)
    oled.begin(SSD1306_SWITCHCAPVCC);
    oled.clearDisplay();
    oled.display();
    last_loop_start = micros();

    restore_calibration_from_eeprom();
    noTone(BUZZER_PIN);
}

void loop()
{
    update_timer();

    float acceleration[3];
    read_accelerometer(acceleration);

    switch (status_flags & STATE_MASK) {
    case STATE_ACQUIRE_DOWN_DIRECTION:
        on_acquire_down_direction(acceleration);
        break;
    case STATE_ACQUIRE_FORWARD_DIRECTION:
        on_acquire_forward_direction(acceleration);
        break;
    case STATE_RUNNING:
        on_running(acceleration);
        handle_reset_button();
        handle_calibrate_button();
        break;
    }
}

/*
  Local Variables:
  mode: c++
  End:
*/
