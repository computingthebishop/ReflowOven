#ifndef CONFIG_H
#define CONFIG_H

//#define ALWAYS_FIRST_RUN // force resett to factory settings at startup
//#define FAKE_HW 1
//#define PIDTUNE 1 // autotune wouldn't fit in the 28k available on my arduino pro micro.
#define WITH_BEEPER // Enables Beeper
//#define WITH_FAN // Enables Lid opening Servo (not yet implemented)
//#define WITH_SERVO // Enables Lid opening Servo (not yet implemented)
//#define SERIAL_VERBOSE


// ----------------------------------------------------------------------------

#define WITH_SPLASH 1


static const uint8_t PIN_LCD_CS   = 4;
static const uint8_t PIN_LCD_DC   = 6;
static const uint8_t PIN_LCD_RST  = 7;
static const uint8_t LCD_ROTATION = 3; // 0/2-> portrait, 1/3-> landscape

static const uint8_t PIN_TC_CS   = 9;
static const uint8_t PIN_TC_DO   = 8;
static const uint8_t PIN_TC_CLK  = 10;
static const uint8_t PIN_HEATER = 3; //
static const uint8_t PIN_FAN    = 5; //
static const uint8_t PIN_BEEPER = A7; // Beeper Out
// --- encoder
static const uint8_t PIN_ENC_A           = A0; //
static const uint8_t PIN_ENC_B           = A1; //
static const uint8_t PIN_ENC_BTN         = A2; //
static const uint8_t ENC_STEPS_PER_NOTCH = 2;
static const boolean IS_ENC_ACTIVE       = false; // encoder module actively fed with VCC ( seems to works bad if set to true )

static const uint16_t BEEP_FREQ = 1976; // B6 note

static const uint8_t PIN_ZX = 2; // pin for zero crossing detector
static const uint8_t INT_ZX = digitalPinToInterrupt(PIN_ZX); // interrupt for zero crossing detector

static const uint8_t NUM_TEMP_READINGS  = 5;
static const uint8_t TC_ERROR_TOLERANCE = 5; // allow for n consecutive errors due to noisy power supply before bailing out
static const float   TEMP_COMPENSATION  = 1.0; // correction factor to match temperature measured with other device



// see: https://www.compuphase.com/electronics/reflowsolderprofiles.htm
static const uint8_t DEFAULT_SOAK_TEPM_A      = 110;
static const uint8_t DEFAULT_SOAK_TEPM_B      = 160;
static const uint8_t DEFAULT_SOAK_DURATION    = 180;
static const uint8_t DEFAULT_PEAK_TEPM        = 200;
static const uint8_t DEFAULT_PEAK_DURATION    = 15;
static const float DEFAULT_RAMP_UP_RATE       = 1.2; // degrees / second (keep it about 1/2 of maximum to prevent PID overshooting)
static const float DEFAULT_RAMP_DOWN_RATE     = 2.0; // degrees / second
static const uint8_t FACTORY_FAN_ASSIST_SPEED = 33;


/*
Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional) (double >=0)
Ki: Determines how aggressively the PID reacts to error over time (Integral) (double>=0)
Kd: Determines how aggressively the PID reacts to the change in error (Derivative) (double>=0)

Experimental method to tune PID:

> Set all gains to 0.
> Increase Kd until the system oscillates.
> Reduce Kd by a factor of 2-4.
> Set Kp to about 1% of Kd.
> Increase Kp until oscillations start.
> Decrease Kp by a factor of 2-4.
> Set Ki to about 1% of Kp.
> Increase Ki until oscillations start.
> Decrease Ki by a factor of 2-4.

*/
#define PID_SAMPLE_TIME 200
#define FACTORY_KP  1.75 // 1.75 //4.0
#define FACTORY_KI 0.03 // 0.03 // 0.05
#define FACTORY_KD 3.0 //3.0//2.0





#endif // CONFIG_H
