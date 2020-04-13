/********************************************************
 * DiffDrive Arduino Uno & Dual MC33926 Motor Controller Example
 * See defines for pin wirings.
 ********************************************************/

#include <DiffDrive.h>

// Motor Interfaces
#define PWM_LEFT     (6)
#define DIR1_LEFT    (5)
#define DIR2_LEFT    (4)
#define ENCA_LEFT    (2)
#define ENCB_LEFT    (3)
#define STATUS_LEFT  (7)
#define CSENSE_LEFT  (A0)

#define PWM_RIGHT    (11)
#define DIR1_RIGHT   (13)
#define DIR2_RIGHT   (12)
#define ENCA_RIGHT   (A4)
#define ENCB_RIGHT   (A5)
#define STATUS_RIGHT (A2)
#define CSENSE_RIGHT (A1)

#define ENABLE       (8)
#define SLEW         (9)
#define INVERT       (10)

// Robot Params
#define CNTS_PER_REV (3591.84) // https://www.pololu.com/product/4866
#define R            (0.045)
#define D            (0.1665)
#define L            (0.1)

// PID Constants
#define KP_CONS      (110.0f)
#define KI_CONS      (100.0f)
#define KD_CONS      (0.0f)
#define KP_AGG       (110.0f)
#define KI_AGG       (100.0f)
#define KD_AGG       (0.0f)

DiffDrive<PWM_LEFT, DIR1_LEFT, DIR2_LEFT, ENCA_LEFT, ENCB_LEFT, STATUS_LEFT, CSENSE_LEFT,
          PWM_RIGHT, DIR1_RIGHT, DIR2_RIGHT, ENCA_RIGHT, ENCB_RIGHT, STATUS_RIGHT, CSENSE_RIGHT,
          ENABLE, SLEW, INVERT> robot(CNTS_PER_REV, R, D, L,
                                      KP_CONS, KI_CONS, KD_CONS,
                                      KP_AGG, KI_AGG, KD_AGG);

ISR(PCINT1_vect) {
    DiffDrive<PWM_LEFT, DIR1_LEFT, DIR2_LEFT, ENCA_LEFT, ENCB_LEFT, STATUS_LEFT, CSENSE_LEFT,
              PWM_RIGHT, DIR1_RIGHT, DIR2_RIGHT, ENCA_RIGHT, ENCB_RIGHT, STATUS_RIGHT, CSENSE_RIGHT,
              ENABLE, SLEW, INVERT>::encoder_isr_right(PINC, A0);
}

ISR(PCINT2_vect) {
    DiffDrive<PWM_LEFT, DIR1_LEFT, DIR2_LEFT, ENCA_LEFT, ENCB_LEFT, STATUS_LEFT, CSENSE_LEFT,
              PWM_RIGHT, DIR1_RIGHT, DIR2_RIGHT, ENCA_RIGHT, ENCB_RIGHT, STATUS_RIGHT, CSENSE_RIGHT,
              ENABLE, SLEW, INVERT>::encoder_isr_left(PIND, 0);
}

void setup() {
    robot.begin(115200);
}

void loop() {
    robot.loop();
}
