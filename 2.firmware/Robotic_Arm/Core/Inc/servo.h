#ifndef   ROBOTIC_ARM_SERVO_H_
#define   ROBOTIC_ARM_SERVO_H_

#define PI 3.1415926535898f

void pwm_start(void);

void translate_angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4);

void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4);

void servo_angle_calculate(float target_x, float target_y, float target_z);

void move_target(char axis, double delta);

void preset_target(int mode);


#endif