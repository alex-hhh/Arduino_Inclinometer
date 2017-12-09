/*
 * Arduino Inclinometer Project, main sketch
 *
 * Copyright (C) 2017 Alex Harsanyi (alexharsanyi@gmail.com)
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


// .................................................... User Parameters ....

/** Pitch angle (in degrees) above which the inclinometer will beep and blink
 * on screen the pitch value.
 */
#define PITCH_WARN 35

/** Roll angle (in degrees) above which the inclinometer will beep and blink
 * on screen the roll value.
 */
#define ROLL_WARN 35

/** Blink (also beep) interval in microseconds.  This is the interval by which
 * the inclinometer will beep and blink when the pitch or roll angles exceed
 * their "warn" values defined above.
 */
#define BLINK_INTERVAL (200000)

/** Low pass filter RC constant for filtering acceleration values, time in
 * microseconds.  Larger values will make the display look more stable, but it
 * will take longer to display the true roll and pitch.
 */
#define LPF_ALPHA (500000.0)

// Calibration parameters.  These will need to be updated for each unit, see
// the Calibration section in README.md

#define X_SLOPE 0.003976143
#define X_INTERCEPT -0.02584493
#define Y_SLOPE 0.003944773
#define Y_INTERCEPT 0.084812623
#define Z_SLOPE 0.003960396
#define Z_INTERCEPT 0.089108911


// ............................................. Hardware Configuration ....

// #define WANT_DEBUG_OUTPUT

#define BUZZER_PIN 8

#define OLED_SCL   6
#define OLED_SDA   5
#define OLED_RES   4
#define OLED_DC    3
#define OLED_CS    12                   // not connected


// ......................................... Program Data and Functions ....

#define PITCH_WARN_FLAG 0x20
#define ROLL_WARN_FLAG  0x40
#define BLINK_ON_FLAG   0x80

#define STATE_ACQUIRE_DOWN_DIRECTION 0
#define STATE_ACQUIRE_FORWARD_DIRECTION 1
#define STATE_RUNNING 2

#define STATE_MASK 0x0F

uint8_t status_flags = 0;

#define SET_FLAG(flag) do { status_flags |= (flag); } while(0)
#define CLR_FLAG(flag) do { status_flags &= ~(flag); } while (0)
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

/** X, Y and Z axis for the installed accelerometer position.  These are used
 * to determine the roll and pitch regardless of the position in which the
 * unit is installed.  They are set during the calibration process, see
 * on_acquire_down_direction() and on_acquire_forward_direction()
 */
float xaxis[3];
float yaxis[3];
float zaxis[3];

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


// .......................................................... utilities ....

/*** Convert 'angle' from radians to degrees. 
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
 */
float calculate_pitch(float cal[3])
{
    float down[3] = {0, 0, 1};
    float pitch_dir[3] = { cal[0], 0, cal[2] };
    vnormalize(pitch_dir, pitch_dir);
    float pitch = vdot(down, pitch_dir);
    return rad2deg(acos(pitch));
}

/** Return the roll angle in degrees (left - right inclination) of the vehicle
 * based on the current "down" direction stored in 'cal'.  'cal' is converted
 * to local coordinates (accounts for the installation orientation of the
 * inclinometer itself).
 */
float calculate_roll(float cal[3])
{
    float down[3] = {0, 0, 1};
    float roll_dir[3] = { 0, cal[1], cal[2] };
    vnormalize(roll_dir, roll_dir);
    float roll = vdot(down, roll_dir);
    return rad2deg(acos(roll));
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

void display_pitch_roll(float pitch, float roll, float gforce)
{
    oled.clearDisplay();

    oled.setTextSize(2);
    oled.setTextColor(WHITE);

    oled.setCursor(10, 5);
    oled.print(gforce, 2);
    oled.setCursor(65,5);
    oled.print("G");

    if (roll > 80 || pitch > 80) {
        oled.setCursor(20, 27);
        oled.print("Rolled");
        oled.setCursor(20, 50);
        oled.print("Over");
    } else {
        // NOTE: We don't show the roll or pitch angles when the warning
        // condition are set and the BLINK_ON_FLAG is cleared -- this creates
        // the blinking effect of the value.

        if (! IS_FLAG_SET(ROLL_WARN_FLAG) || IS_FLAG_SET(BLINK_ON_FLAG)) {
            oled.setCursor(10, 27);
            oled.print(roll, 0);
            oled.print("\t");                // \t displays the degree sign
        }

        oled.setCursor(65, 27);
        oled.print("Roll");

        if (! IS_FLAG_SET(PITCH_WARN_FLAG) || IS_FLAG_SET(BLINK_ON_FLAG)) {
            oled.setCursor(10, 50);
            oled.print(pitch, 0);
            oled.print("\t");
        }

        oled.setCursor(65, 50);
        oled.print("Pitch");
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
    float dot = vdot(cal, zaxis);
    if (dot > 0.99 && dot < 1.01) {
        vnormalize(zaxis, zaxis);

        // Got our down direction, we can now determine the forward direction.
        SET_STATE(STATE_ACQUIRE_FORWARD_DIRECTION);
    }
    else {
        zaxis[0] = (zaxis[0] + cal[0]) * 0.5;
        zaxis[1] = (zaxis[1] + cal[1]) * 0.5;
        zaxis[2] = (zaxis[2] + cal[2]) * 0.5;

        // Show a message on the screen that we are acquiring the down
        // direction.
        oled.clearDisplay();

        oled.setTextSize(2);
        oled.setTextColor(WHITE);

        oled.setCursor(5, 5);
        oled.print(dot, 2);
        oled.setCursor(5, 27);
        oled.print("Down");
        oled.setCursor(5, 50);
        oled.print("Direction");

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
        vcross(zaxis, cal, left);
        vcross(left, zaxis, xaxis);
        vcross(zaxis, xaxis, yaxis);
        vnormalize(xaxis, xaxis);
        vnormalize(yaxis, yaxis);
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

        oled.setCursor(10, 27);
        oled.print("Move");

        oled.setCursor(10, 50);
        oled.print("Forward!");

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
    ncal[0] = vdot(xaxis, cal);
    ncal[1] = vdot(yaxis, cal);
    ncal[2] = vdot(zaxis, cal);

    float pitch = calculate_pitch(ncal);
    float roll = calculate_roll(ncal);
    float gforce = vlen(cal);

    // Check for WARN conditions being met and set or clear appropriate flags.
    // We set the warn flag when an angle exceeds the warn value, but only
    // clear it when it drops two degrees below that value.  This ensures that
    // there is no annoying quick on-off buzzer when the inclinometer hovers
    // around the warn value.

    if (roll > ROLL_WARN) {
        SET_FLAG(ROLL_WARN_FLAG);
    } else if (roll < (ROLL_WARN - 2)) { // note the -2, hysteresis !
        CLR_FLAG(ROLL_WARN_FLAG);
    }

    if (pitch > PITCH_WARN) {
        SET_FLAG(PITCH_WARN_FLAG);
    } else if (pitch < (PITCH_WARN - 2)) { // note the -2, hysteresis !
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
        break;
    }
}

/*
  Local Variables:
  mode: c++
  End:
*/
