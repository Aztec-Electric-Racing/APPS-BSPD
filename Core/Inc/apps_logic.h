//
// Created by kami on 11/18/2026.
//

#ifndef APPS_BSPD_APPS_LOGIC_H
#define APPS_BSPD_APPS_LOGIC_H
#include <stdint.h>

uint16_t GetSensorPermille(uint16_t sensor, uint32_t lower, uint16_t range);
uint32_t ClampDac(int32_t value);
uint16_t GetThrottlePermille(uint16_t sensor1, uint16_t sensor2,
                             uint32_t lower1, uint16_t range1,
                             uint32_t lower2, uint16_t range2);

uint8_t AreSensorsValid(uint16_t s1, uint16_t s2,
                        uint32_t lower1, uint32_t upper1,
                        uint32_t lower2, uint32_t upper2);

uint8_t CalibrationIsValid(uint32_t lower1, uint32_t upper1,
                           uint32_t lower2, uint32_t upper2);

/* ---- Thresholds shared by the firmware and the unit tests ---- */
#define APPS_MISMATCH_PERMILLE  100U  /* 10.0% allowed APPS1/APPS2 disagreement */
#define APPS_MISMATCH_TIME_MS   100U  /* disagreement must last longer than this to fault */
#define APPS_TRIP_PERMILLE      250U  /* 25.0% calibrated travel */
#define APPS_CLEAR_PERMILLE     50U   /* 5.0% calibrated travel */

/* State the mismatch timer remembers between loop passes. Start it zeroed. */
typedef struct
{
    uint8_t  timing;      /* 1 while a mismatch is being timed */
    uint32_t startTick;   /* time (ms) the current mismatch began */
} MismatchTimer_t;

/* Forget any mismatch in progress (e.g. when the sensors are invalid). */
void MismatchTimerReset(MismatchTimer_t *timer);

/* Returns 1 (fault) once the two sensors have disagreed by more than
 * APPS_MISMATCH_PERMILLE for longer than APPS_MISMATCH_TIME_MS. nowMs is the
 * current time, so the function never touches the hardware timer itself. */
uint8_t UpdateMismatchFault(MismatchTimer_t *timer,
                            uint16_t permille1, uint16_t permille2,
                            uint32_t nowMs);

/* Decides the next state of the throttle/brake latch (1 = latched/inhibited). */
uint8_t NextThrottleBrakeLatch(uint8_t currentlyLatched,
                               uint8_t sensorsValid,
                               uint8_t appsFault,
                               uint8_t brakesPressed,
                               uint16_t throttlePermille);




#endif //APPS_BSPD_APPS_LOGIC_H
