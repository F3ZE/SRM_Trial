#include "servo.h"
#include "main.h"
#include "math.h"
#include "stdio.h"
#include "stm32f1xx_hal_tim.h"

extern double pulse_1;
extern double pulse_2;
extern double pulse_3;
extern double pulse_4;

extern double now_angle_1;
extern double now_angle_2;
extern double now_angle_3;
extern double now_angle_4;

extern double target_angle_1;
extern double target_angle_2;
extern double target_angle_3;
extern double target_angle_4;

extern double R;
extern double length_1;
extern double length_2;
extern double length_3;
extern double length_4;

extern double j_all;

extern double l0;
extern double l1;
extern double l2;

extern float pi;

extern uint8_t center_x ,center_y ,color_type ;

extern double center_x_cm,center_y_cm;

double a1;
double a2;
double a3;
double a4;

double abs_angle_error_1;
double abs_angle_error_2;
double abs_angle_error_3;
double abs_angle_error_4;

extern TIM_HandleTypeDef htim2;

void pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PWM_1
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PWM_2
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//PWM_3
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);//PWM_4
}

void translate_angle_to_pulse(double angle_1,double angle_2,double angle_3,double angle_4)
{
    pulse_1 = (((angle_1 + 70) / 90 ) + 0.5)*(20000/20);
    pulse_2 = (((angle_2 +90) / 90 ) + 0.5)*(20000/20);
    pulse_3 = (((angle_3) / 90 ) + 0.5)*(20000/20);
    pulse_4 = ((((-angle_4 + 90) / 270 * 180) / 90) + 0.5)*(20000/20);
}

void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4)
{
    translate_angle_to_pulse(angle_1,angle_2,angle_3,angle_4);

    if(pulse_1 >= 0 && pulse_1 <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse_1);
    }
    if(pulse_2 >= 0 && pulse_2 <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse_2);
    }
    if(pulse_3 >= 0 && pulse_3 <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pulse_3);
    }
    if(pulse_4 >= 0 && pulse_4 <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse_4);
    }
}

void servo_angle_calculate(float target_x, float target_y, float target_z)
{
    if (target_y >= 18)
        target_y = 18;
    else if(target_y <= 3)
        target_y = 3;
    float len_1, len_2, len_3, len_4;
    float j1,j2,j3,j4 ;
    float L, H, bottom_r;
    float j_sum;
    float len, high;
    float cos_j3, sin_j3;
    float cos_j2, sin_j2;
    float k1, k2;
    int i;
    float n, m;
    n = 0;
    m = 0;

    bottom_r = 14;
    len_1 = 12;
    len_2 = 12;
    len_3 = 12;
    len_4 = 12;

    if (target_x == 0)
        j1 = 90;
    else
        j1 = 90 - atan(target_x / (target_y + bottom_r)) * (57.3);


    for (i = 0; i <= 180; i ++)
    {
        j_sum = 3.1415927 * i / 180;

        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
        high = target_z;

        L = len - len_4 * sin(j_sum);
        H = high - len_4 * cos(j_sum) - len_1;

        cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
        sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

        j3 = atan((sin_j3) / (cos_j3)) * (57.3);

        k2 = len_3 * sin(j3 / 57.3);
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

        j2 = atan((sin_j2) / (cos_j2)) * 57.3;
        j4 = j_sum * 57.3 - j2 - j3;

        if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
        {
            n ++;
        }
    }

    for (i = 0; i <= 180; i ++)
    {
        j_sum = 3.1415927 * i / 180;

        len = sqrt((target_y + bottom_r) * (target_y + bottom_r) + target_x * target_x);
        high = target_z;

        L = len - len_4 * sin(j_sum);
        H = high - len_4 * cos(j_sum) - len_1;

        cos_j3 = ((L * L) + (H * H) - ((len_2) * (len_2)) - ((len_3) * (len_3))) / (2 * (len_2) * (len_3));
        sin_j3 = (sqrt(1 - (cos_j3) * (cos_j3)));

        j3 = atan((sin_j3) / (cos_j3)) * (57.3);

        k2 = len_3 * sin(j3 / 57.3);
        k1 = len_2 + len_3 * cos(j3 / 57.3);

        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        sin_j2 = (sqrt(1 - (cos_j2) * (cos_j2)));

        j2 = atan((sin_j2) / (cos_j2)) * 57.3;
        j4 = j_sum * 57.3 - j2 - j3;

        if (j2 >= 0 && j3 >= 0 && j4 >= -90 && j2 <= 180 && j3 <= 180 && j4 <= 90)
        {
            m ++;
            if (m == n / 2 || m == (n + 1) / 2)
                break;
        }
    }
    target_angle_1 = j1;
    target_angle_2 = j2;
    target_angle_3 = j3;
    target_angle_4 = j4;

    printf("center_x: %f\r\n center_y: %f\r\n taret_angle_1: %f\r\n taret_angle_2: %f\r\n taret_angle_3: %f\r\n taret_angle_4: %f\r\n",target_x,target_y,j1,j2,j3,j4);
}

void servo_control(double temp_target_angle_1,double temp_target_angle_2,double temp_target_angle_3,double temp_target_angle_4,double temp_target_angle_5,double temp_target_angle_6)
{

//	 abs_angle_error_1 = fabs(temp_target_angle_1 - now_angle_1);   //������ֵ
    abs_angle_error_2 = fabs(temp_target_angle_2 - now_angle_2);   //������ֵ
    abs_angle_error_3 = fabs(temp_target_angle_3 - now_angle_3);   //������ֵ
    abs_angle_error_4 = fabs(temp_target_angle_4 - now_angle_4);   //������ֵ
    abs_angle_error_5 = fabs(temp_target_angle_5 - now_angle_5);   //������ֵ
    abs_angle_error_6 = fabs(temp_target_angle_6 - now_angle_6);   //������ֵ

//   a1 = abs_angle_error_1;
    a2 = abs_angle_error_2;
    a3 = abs_angle_error_3;
    a4 = abs_angle_error_4;
    a5 = abs_angle_error_5;
    a6 = abs_angle_error_6;

    if(temp_target_angle_2 != NULL)
    {
        for(;abs_angle_error_2 >= 3;abs_angle_error_2 --)
        {

            double pwm_angle_2 = (temp_target_angle_2 > now_angle_2 ? temp_target_angle_2 - abs_angle_error_2 : temp_target_angle_2 + abs_angle_error_2);

            pwm_out(temp_target_angle_1,pwm_angle_2,now_angle_3,now_angle_4,now_angle_5,now_angle_6);
            HAL_Delay(20);
            now_angle_2 = pwm_angle_2;
        }
        now_angle_2 = temp_target_angle_2;
        pwm_out(temp_target_angle_1,temp_target_angle_2,now_angle_3,now_angle_4,now_angle_5,now_angle_6);
    }

    if(temp_target_angle_3 != NULL)
    {
        for(;abs_angle_error_3 >= 3;abs_angle_error_3 --)
        {

            double pwm_angle_3 = (temp_target_angle_3 > now_angle_3 ? temp_target_angle_3 - abs_angle_error_3 : temp_target_angle_3 + abs_angle_error_3);
            pwm_out(temp_target_angle_1,temp_target_angle_2,pwm_angle_3,now_angle_4,now_angle_5,now_angle_6);
            HAL_Delay(20);
            now_angle_3 = pwm_angle_3;
        }
        now_angle_3 = temp_target_angle_3;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,now_angle_4,now_angle_5,now_angle_6);
    }

    if(temp_target_angle_4 != NULL)
    {
        for(;abs_angle_error_4 >= 3;abs_angle_error_4 --)
        {

            double pwm_angle_4 = (temp_target_angle_4 > now_angle_4 ? temp_target_angle_4 - abs_angle_error_4 : temp_target_angle_4 + abs_angle_error_4);
            pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,pwm_angle_4,now_angle_5,now_angle_6);
            HAL_Delay(20);
            now_angle_4 = pwm_angle_4;
        }
        now_angle_4 = temp_target_angle_4;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,now_angle_5,now_angle_6);
    }

    if(temp_target_angle_6 != NULL)
    {
        for(;abs_angle_error_6 >= 3;abs_angle_error_6 --)
        {

            double pwm_angle_6 = (temp_target_angle_6 > now_angle_6 ? temp_target_angle_6 - abs_angle_error_6 : temp_target_angle_6 + abs_angle_error_6);
            pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,now_angle_5,pwm_angle_6);
            HAL_Delay(20);
            now_angle_6 = pwm_angle_6;
        }
        now_angle_6 = temp_target_angle_6;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,now_angle_5,temp_target_angle_6);
    }

    if(temp_target_angle_5 != NULL)
    {
        for(;abs_angle_error_5 >= 3;abs_angle_error_5 --)
        {

            double pwm_angle_5 = (temp_target_angle_5 > now_angle_5 ? temp_target_angle_5 - abs_angle_error_5 : temp_target_angle_5 + abs_angle_error_5);
            pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,pwm_angle_5,temp_target_angle_6);
            HAL_Delay(20);
            now_angle_5 = pwm_angle_5;
        }
        now_angle_5 = temp_target_angle_5;
        pwm_out(temp_target_angle_1,temp_target_angle_2,temp_target_angle_3,temp_target_angle_4,temp_target_angle_5,temp_target_angle_6);
    }

    HAL_Delay(1000);

    abs_angle_error_1  = a1 ;
    abs_angle_error_2  = a2 ;
    abs_angle_error_3  = a3 ;
    abs_angle_error_4  = a4 ;
    abs_angle_error_5  = a5 ;
    abs_angle_error_6  = a6 ;

}

void servo_reset_begin(void)
{
    pwm_out(0,0,0,0);
    HAL_Delay(1000);
    now_angle_1 = 0;
    now_angle_2 = 0;
    now_angle_3 = 0;
    now_angle_4 = 0;
    HAL_Delay(1000);
}

void servo_reset(void)
{
    double i1 = 0,i2 = 0,i3 = 0;
    now_angle_6 = 90;
    double b5 = now_angle_5;
    double b4 = now_angle_4;
    //double b3 = now_angle_3;
    double b2 = now_angle_2;
    for(i1 = 10 * b5;i1 >= 0;i1 -= b5)
    {
        pwm_out(0,now_angle_2,0,now_angle_4,0.1 * i1,90);
        HAL_Delay(30);
        now_angle_5 = 0.1 *i1;

    }
    pwm_out(0,now_angle_2,0,now_angle_4,0,90);

    for(i2 = 10 * b4;i2 >= 0;i2 -= b4)
    {
        pwm_out(0,now_angle_2,0,0.1 * i2,0,90);
        HAL_Delay(30);
        now_angle_4 = 0.1 * i2;

    }
    pwm_out(0,now_angle_2,0,0,0,90);

    for(i3 = 10 * b2;i3 >= 0;i3 -= b2)
    {
        pwm_out(0,0.1 * i3,0,0,0,90);
        HAL_Delay(30);
        now_angle_2 = 0.1 * i3;
    }
    pwm_out(0,0,0,0,0,90);

    HAL_Delay(1000);
    now_angle_1 = 0;
}

void move_target(char axis, int direction, int delta)
{
    // 假设我们的机械臂有6个舵机，并且有一组逆运动学函数来计算舵机位置
    double target_x = 0, target_y = 0, target_z = 0;
    if (direction)
    {
        direction = 1;
    }
    else
    {
        direction = -1;
    }
    // 根据方向更新目标末端位置
    if (axis == 'X')
    {
        target_x += direction * (delta);
        servo_angle_calculate(target_x, target_y, target_z); // 更新目标角度
        pwm_out(target_angle_1, target_angle_2, target_angle_3, target_angle_4);
        HAL_Delay(20); // 控制更新频率
    }
    if (axis == 'Y')
    {
        target_y += direction * (delta);
        servo_angle_calculate(target_x, target_y, target_z); // 更新目标角度
        pwm_out(target_angle_1, target_angle_2, target_angle_3, target_angle_4);
        HAL_Delay(20); // 控制更新频率
    }
    if (axis == 'Z')
    {
        target_z += direction * (delta);
        servo_angle_calculate(target_x, target_y, target_z); // 更新目标角度
        pwm_out(target_angle_1, target_angle_2, target_angle_3, target_angle_4);
        HAL_Delay(20); // 控制更新频率
    }

//todo: 不是很平滑，可以有加减速控制
/*    // 加速与减速运动控制，同之前的逻辑
    double current_speed = 0.0;
    double max_speed = (double)speed;
    double acceleration = 0.1; // 加速度因子
    double deceleration = 0.1;  // 减速度因子
    // 加速阶段
    while (current_speed < max_speed) {
        current_speed += acceleration;
        if (current_speed > max_speed) {
            current_speed = max_speed;
        }
        // 更新舵机到新目标位置

    }
    // 减速阶段
    while (current_speed > 0) {
        current_speed -= deceleration;
        if (current_speed < 0) {
            current_speed = 0;
        }
        // 可以选择保持在最后的目标位置
        pwm_out(target_angle_1, target_angle_2, target_angle_3, target_angle_4);
        HAL_Delay(20); // 控制更新频率
    }*/
}

void preset_target(int mode)
{
    // 预设的位置，依据不同的模式
    switch (mode) {
        case 1:
            target_angle_1= 0;   // 设置舵机1的角度
            target_angle_2= 0;   // 设置舵机2的角度
            target_angle_3= 0;   // 设置舵机3的角度
            target_angle_4= 0;   // 设置舵机4的角度
            break;
        case 2:
            target_angle_1= 0;   // 设置舵机1的角度
            target_angle_2= 0;   // 设置舵机2的角度
            target_angle_3= 0;   // 设置舵机3的角度
            target_angle_4= 0;   // 设置舵机4的角度
            break;
        case 3:
            target_angle_1= 0;   // 设置舵机1的角度
            target_angle_2= 0;   // 设置舵机2的角度
            target_angle_3= 0;   // 设置舵机3的角度
            target_angle_4= 0;   // 设置舵机4的角度
            break;
        default:
            break;
    }

    // 调用pwm_out更新舵机位置
    pwm_out(target_angle_1, target_angle_2, target_angle_3, target_angle_4);
    HAL_Delay(1000); // 等待舵机移动到新位置
}
