#ifndef BMI270_H
#define BMI270_H

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the BMI270 driver. Returns 1 on success, 0 on failure.
int bmi270_init(void);

// Read sensor-derived angles.
// Returns 1 if a fresh/valid reading is returned, 0 otherwise.
// out_roll_deg: tilt left/right in degrees (positive = tilt right).
// out_yaw_deg: integrated yaw angle in degrees (relative zero updated by rezero function).
int bmi270_read_angles(float *out_roll_deg, float *out_yaw_deg);

// Re-zero the internal yaw/roll offsets (call when user presses FN in gyro mode).
void bmi270_rezero(void);

#ifdef __cplusplus
}
#endif

#endif // BMI270_H
