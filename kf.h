#ifndef _KF_H_
#define _KF_H_

extern float angle_estimate;
extern float gyro_bias;
extern float covar[4];
/*
extern float state[2];
extern const float Q[4];// process noise
extern const float R;// measurement noise
extern float K[2];
extern const float dt;
*/
void kf_prior(u8 input_flag);
void kf_measure();

#endif
