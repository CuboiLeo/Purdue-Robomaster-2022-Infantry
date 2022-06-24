#include "Kalman.h"
#include "Higher_Class.h"

Kalman1 Vision_kalman_x;
Kalman1 Vision_kalman_x1;
Kalman1 Vision_kalman_x2;
Kalman1 Vision_kalman_y;
Kalman1 Vision_kalman2_x;
Kalman1 Vision_kalman2_x1;

float kalman1_p_now = 0;
float kalman1_K = 0;
float kalman1_x_now = 0;
float kalman1_x_next = 0;

void kalmanCreate(void)
{
	Vision_kalman_y.A = 1;
	Vision_kalman_y.Q = 3;
	Vision_kalman_y.R = 1;

	Vision_kalman_x.A = 1;
	Vision_kalman_x.Q = 1;
	Vision_kalman_x.R = 2000;
	
	Vision_kalman_x1.A = 1;
	Vision_kalman_x1.Q = 0.05;
	Vision_kalman_x1.R = 1000;	
	
	Vision_kalman_x2.A = 1;
	Vision_kalman_x2.Q = 0.001f;
	Vision_kalman_x2.R = 1;//100;
	
	Vision_kalman2_x.Q_gyro = 0;
	Vision_kalman2_x.Q = 1;
	Vision_kalman2_x.R = 100;//1200;
	
	Vision_kalman2_x1.Q_gyro = 0;
	Vision_kalman2_x1.Q = 1;
	Vision_kalman2_x1.R = 1;//20;
}


float KalmanFilter(Kalman1 *kalman1, float real)//, float speed, float run_time_clock, float fps)
{
	  kalman1->x_now = kalman1->x_last;
    kalman1->p_now = kalman1->p_last + kalman1->Q;  
    kalman1->K = kalman1->p_now / (kalman1->p_now + kalman1->R);    //¿¨¶ûÂüÔöÒæ   
    kalman1->x_next = kalman1->x_now + kalman1->K * (real - kalman1->x_now);    //Ô¤¹À
    kalman1->p_next = (1 - kalman1->K) * kalman1->p_now;     
   	kalman1->x_last = kalman1->x_next;  
    kalman1->p_last = kalman1->p_next;
	  return kalman1->x_last;
}


float KalmanFilter2(Kalman1 *kalman2, float real, float speed)//, float run_time_clock, float fps)
{
	float tim = vision_run_fps * 0.001f;// + vision_run_time;
	kalman2->x_next = kalman2->x_next + speed * tim;//- kalman2->Q_bias
	//alman2->x_next = real + speed * tim;  
	kalman2->P0 = kalman2->P0 + kalman2->Q - (kalman2->P1 - kalman2->P2) * tim + kalman2->P3 * tim * tim;
	kalman2->P1 = kalman2->P1 - kalman2->P3 * tim;
	kalman2->P2 = kalman2->P2 - kalman2->P3 * tim;
	kalman2->P3 = kalman2->P3 + kalman2->Q_gyro;
	kalman2->K0 = kalman2->P0 / (kalman2->P0 + kalman2->R);
	kalman2->K1 = kalman2->P2 / (kalman2->P0 + kalman2->R);
	kalman2->x_next = kalman2->x_next + kalman2->K0 * (real - kalman2->x_next);
	//kalman2->Q_bias = kalman2->Q_bias + kalman2->K1 * (real - kalman2->x_next);
	kalman2->P0 = kalman2->P0 - kalman2->K0 * kalman2->P0;
	kalman2->P1 = kalman2->P1 - kalman2->K0 * kalman2->P1;
	kalman2->P2 = kalman2->P2 - kalman2->K1 * kalman2->P0;
	kalman2->P3 = kalman2->P3 - kalman2->K1 * kalman2->P1;
	kalman2->x_now = kalman2->x_next;
	return kalman2->x_next;
}


void Kalman_Reset(Kalman1 *kalman1)
{
	kalman1->x_now = 1;
	kalman1->p_now = 1;
	kalman1->x_next = 1;
	kalman1->p_next = 1;
	kalman1->x_last = 1;
	kalman1->p_last = 1;
}


void Kalman_Reset2(Kalman1 *kalman2)
{
	kalman2->x_now = 1;
	kalman2->x_next = 1;
	kalman2->P0 = 1;
	kalman2->P1 = 1;
	kalman2->P2 = 1;
	kalman2->P3 = 1;
}

