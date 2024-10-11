#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "cmsis_os.h"

#include "math.h"

#include "servo.h"
#include "ble_remote.h"

// 内部变量声明
double base_radius = 5.6;  // 底座半径
double base_height = 6.6;  // 底座高度
double arm_length[3] = {24.6, 12, 15}; // 机械臂长度数组

double now_angle[4] = {0, -25, 115, 90}; // 当前角度数组

double now_x = 0, now_y = 0, now_z = 0; // 目标坐标

int servo_mode_flag = 0; // 1: 末端控制器角度限制模式 0: 自由模式
// 外部变量声明
extern TIM_HandleTypeDef htim2;

void pwm_start(void)
{
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//PWM_1
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//PWM_2
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);//PWM_3
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);//PWM_4
}

void pwm_out(double angle_1, double angle_2, double angle_3, double angle_4)
{
    double pulse[4]; // 脉冲宽度数组

    pulse[3] = (10 * (-(angle_1 + 20) + 90) / 180 + 2.5) / 100 * 20000;
    pulse[2] = (10 * ((angle_2 + 15) + 90) / 180 + 2.5) / 100 * 20000;
    pulse[1] = (10 * (-(angle_3) + 85)  / 180 + 2.5) / 100 * 20000;
    pulse[0] = (10 * ((angle_4 - 5) + 90) / 180 + 2.5) / 100 * 20000;

    if(pulse[0] >= 0 && pulse[0] <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, pulse[0]);
    }
    if(pulse[1] >= 0 && pulse[1] <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, pulse[1]);
    }
    if(pulse[2] >= 0 && pulse[2] <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, pulse[2]);
    }
    if(pulse[3] >= 0 && pulse[3] <= 20000)
    {
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, pulse[3]);
    }
}

void servo_angle_calculate(float target_x, float target_y, float target_z)
{
    double angles[4]; // 四个姿态角
    double valid_angles[180][4]; // 存储有效的角度组合
    int valid_count = 0; // 有效角度计数器

    // 计算底部姿态角
    angles[0] = (target_x == 0) ? 90 : 90 - atan(target_x / (target_y + base_radius)) * (57.3);
    for (int i = servo_mode_flag == 1 ? 180 : 0 ; i <= 180; i++) {
        double j_sum = PI * i / 180;
        double len = sqrt((target_y + base_radius) * (target_y + base_radius) + target_x * target_x);
        double high = target_z;

        double L = len - arm_length[2] * sin(j_sum);
        double H = high - arm_length[2] * cos(j_sum) - base_height;

        double cos_j3 = ((L * L) + (H * H) - (arm_length[0] * arm_length[0]) - (arm_length[1] * arm_length[1])) / (2 * arm_length[0] * arm_length[1]);
        double sin_j3 = sqrt(1 - (cos_j3) * (cos_j3));
        angles[2] = atan(sin_j3 / cos_j3) * (57.3);

        double k2 = arm_length[1] * sin(angles[2] / 57.3);
        double k1 = arm_length[0] + arm_length[1] * cos(angles[2] / 57.3);
        double cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        double sin_j2 = sqrt(1 - (cos_j2) * (cos_j2));
        angles[1] = atan(sin_j2 / cos_j2) * (57.3);
        angles[3] = j_sum * 57.3 - angles[1] - angles[2];

        // 验证姿态角的有效性
        if (angles[1] >= 0 && angles[2] >= 0 && angles[3] >= -90
            && angles[1] <= 180 && angles[2] <= 180 && angles[3] <= 90)
        {
            // 存储有效的姿态角
            valid_angles[valid_count][0] = angles[0];
            valid_angles[valid_count][1] = angles[1];
            valid_angles[valid_count][2] = angles[2];
            valid_angles[valid_count][3] = angles[3];
            valid_count++;
        }
    }
    // 选择中值角度
    if (valid_count > 0) {
        int median_index = valid_count / 2;
        now_angle[0] = valid_angles[median_index][0];
        now_angle[1] = valid_angles[median_index][1];
        now_angle[2] = valid_angles[median_index][2];
        now_angle[3] = valid_angles[median_index][3];
    }
}

void servo_move_delta(double delta_x, double delta_y, double delta_z)
{
    now_x += delta_x;
    now_y += delta_y;
    now_z += delta_z;
    servo_angle_calculate(now_x, now_y, now_z); // 更新目标角度
    pwm_out(now_angle[0], now_angle[1], now_angle[2], now_angle[3]);
    HAL_Delay(10); //控制更新频率
}

void servo_move_to_target(double target_angle_1, double target_angle_2, double target_angle_3, double target_angle_4)
{
    double step_time = 70; // 每个循环的延迟时间（毫秒）
    double max_speed = 3; // 速度因子，越大表示每次转动的角度越大
    double current_angle_1 = now_angle[0];
    double current_angle_2 = now_angle[1];
    double current_angle_3 = now_angle[2];
    double current_angle_4 = now_angle[3];

    // 将目标角度差距转化为步数
    double delta_1 = target_angle_1 - current_angle_1;
    double delta_2 = target_angle_2 - current_angle_2;
    double delta_3 = target_angle_3 - current_angle_3;
    double delta_4 = target_angle_4 - current_angle_4;

    // 计算总的转动次数（以最大速度步数作为依据）
    int steps_1 = (int)(fabs(delta_1) / max_speed) + 1;
    int steps_2 = (int)(fabs(delta_2) / max_speed) + 1;
    int steps_3 = (int)(fabs(delta_3) / max_speed) + 1;
    int steps_4 = (int)(fabs(delta_4) / max_speed) + 1;

    // 执行移动到目标角度
    for (int i = 0; i <= steps_1; i++)
    {
        double proportion = (double)i / steps_1;
        now_angle[0] = current_angle_1 + delta_1 * proportion;
        pwm_out(now_angle[0], now_angle[1], now_angle[2], now_angle[3]);
        HAL_Delay(step_time);
    }

    for (int i = 0; i <= steps_2; i++)
    {
        double proportion = (double)i / steps_2;
        now_angle[1] = current_angle_2 + delta_2 * proportion;
        pwm_out(now_angle[0], now_angle[1], now_angle[2], now_angle[3]);
        HAL_Delay(step_time);
    }

    for (int i = 0; i <= steps_3; i++)
    {
        double proportion = (double)i / steps_3;
        now_angle[2] = current_angle_3 + delta_3 * proportion;
        pwm_out(now_angle[0], now_angle[1], now_angle[2], now_angle[3]);
        HAL_Delay(step_time);
    }

    for (int i = 0; i <= steps_4; i++)
    {
        double proportion = (double)i / steps_4;
        now_angle[3] = current_angle_4 + delta_4 * proportion;
        pwm_out(now_angle[0], now_angle[1], now_angle[2], now_angle[3]);
        HAL_Delay(step_time);
    }

    // 最终位置更新
    now_angle[0] = target_angle_1;
    now_angle[1] = target_angle_2;
    now_angle[2] = target_angle_3;
    now_angle[3] = target_angle_4;
}

void preset_target(int mode)
{
    // 预设的位置，依据不同的模式
    switch (mode) {
        case 1:
            servo_move_to_target(0, -25, 115, 90);
            now_x = 0, now_y = 0, now_z = 0;
            break;
        case 2:
            servo_move_to_target(0, -25, 115, 45);
            now_x = 0, now_y = 0, now_z = 0;
            break;
        case 3:
            servo_move_to_target(0, -25, 115, 0);
            now_x = 0, now_y = 0, now_z = 0;
            break;
        default:
            break;
    }
}

void StartservoTask(void const * argument)
{
    while(1)
    {
        const remote_t *r = get_remote_control_point();
        if (r->Button[0]==0x01)
            preset_target(1);
        if (r->Button[1]==0x01)
            preset_target(2);
        if (r->Button[2]==0x01)
            preset_target(3);
        if (r->Switch[0]==0x01)
        {
            servo_mode_flag = 0;
            servo_move_delta(r->rocker[0].x_position,
                             r->rocker[0].y_position,
                             r->rocker[1].y_position);//-500~500,作为x轴增加量
        }
        else if (r->Switch[1]==0x01&&r->Switch[0]==0x00)
        {
            servo_mode_flag = 1;
            servo_move_delta(r->rocker[0].y_position,
                             r->rocker[0].x_position,
                             r->rocker[1].y_position);//-500~500,作为y轴增加量
        }
        osDelay(1);
    }
}