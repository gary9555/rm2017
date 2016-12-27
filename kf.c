#include "yaw_gyro.h"

// kalman filter
float state[2];
float covar[4]={0.1,0, 0,0.1};
float angle_estimate;
float gyro_bias = 0;
const float Q[4] = {0.1, 0, 0, 0.1};// process noise
const float R = 0.1;// measurement noise
float K[2];
const float dt = 0.007;
u8 first_time_kf_flag = 1;
/* A = [1 0;0 1]; u = [omega*dt; 0]; */

// kalman filter prior update 
void kf_prior(u8 input_flag){
	
	if(first_time_kf_flag){
		angle_estimate = real_angle;
		first_time_kf_flag = 0;
	}else{
		int i;		
		if(input_flag){
			angle_estimate += yaw_pid_output_angle*dt;
			if(angle_estimate>3600){
				angle_estimate -= 3600;
			}else if(angle_estimate < 0){
				angle_estimate += 3600;
			}
		}
		// covariance update
		for(i=0;i<4;i++){
			covar[i] += Q[i];
		}
	}
}	

// kalman filter measurement update
void kf_measure(){
	
	int i;
	float temp1 = 1/(covar[0]+covar[1]+covar[2]+covar[3]+R);
	K[0] = (covar[0]+covar[1])*temp1;
	K[1] = (covar[2]+covar[3])*temp1;
	
	float yaw_measure = real_angle; //////////////////////////////////////////////  need to find the output from the gyro
	float tmp=0;
	if(yaw_measure - angle_estimate > 3000){
		tmp = 3600;
	} else if(yaw_measure - angle_estimate < -3000){
		tmp = -3600;
	}
	float temp2 = (yaw_measure - tmp - angle_estimate - gyro_bias);
	angle_estimate += K[0]*temp2;
	gyro_bias += K[1]*temp2;
	
	if(angle_estimate>3600){
		angle_estimate -= 3600;
	}else if(angle_estimate < 0){
		angle_estimate += 3600;
	}
	// covariance update
	float temp3[4];
	temp3[0] = (1-K[0]) * covar[0] - K[0]*covar[2];
	temp3[1] = (1-K[0]) * covar[1] - K[0]*covar[3];
	temp3[2] = -K[1]*covar[0] + (1-K[1]) * covar[2];
	temp3[3] = -K[1]*covar[1] + (1-K[1]) * covar[3];
	for(i=0;i<4;i++){
		covar[i] = temp3[i];
	}
}
