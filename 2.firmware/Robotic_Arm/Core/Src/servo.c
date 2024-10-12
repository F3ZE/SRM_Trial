#include "main.h"
#include "stm32f1xx_hal_tim.h"
#include "cmsis_os.h"

#include "math.h"

#include "servo.h"
#include "ble_remote.h"

#define ANGLE_1_MIN -110
#define ANGLE_1_MAX 70
#define ANGLE_2_MIN -65
#define ANGLE_2_MAX 115
#define ANGLE_3_MIN -90
#define ANGLE_3_MAX 90
#define ANGLE_4_MIN -85
#define ANGLE_4_MAX 95

// 内部变量声明
double base_height = 15;  // 底座高度
double arm_length[3] = {24.6, 12, 15}; // 机械臂长度数组

double now_angle[4] = {0, -25, 115, 75}; // 当前角度数组
//double now_x = 0, now_y = 5.61, now_z = 19.87; // 目标坐标
double now_x = 0, now_y = 5.61, now_z = 19.87; // 目标坐标

int servo_mode_flag = 1; // 1: 末端控制器角度限制模式 0: 自由模式
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

    pulse[2] = (10 * ((angle_1 + 20) + 90) / 180 + 2.5) / 100 * 20000;// -110~70
    pulse[3] = (10 * ((angle_2 - 25) + 90) / 180 + 2.5) / 100 * 20000; // -65~115
    pulse[0] = (10 * (-(angle_3) + 90)  / 180 + 2.5) / 100 * 20000; // -90~90
    pulse[1] = (10 * ((angle_4 - 5) + 90) / 180 + 2.5) / 100 * 20000;// -85~95

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

    angles[0] = 90 - atan(target_y / (target_x + 1e-9)) * (57.3);

    // 计算底部姿态角
    if (servo_mode_flag == 1)
    {
        double a_sum = PI * 165 / 180; //165度为理想抓取角度
        double len = sqrt((target_y * target_y) + (target_x * target_x));
        double high = target_z;

        double L = len - arm_length[2] * sin(a_sum);
        double H = high - arm_length[2] * cos(a_sum) - base_height;

        double cos_j3 = ((L * L) + (H * H) - (arm_length[0] * arm_length[0]) - (arm_length[1] * arm_length[1])) /
                        (2 * arm_length[0] * arm_length[1]);
        double sin_j3 = sqrt(1 - (cos_j3 * cos_j3));
        angles[2] = atan(sin_j3 / (cos_j3 + 1e-9)) * (57.3);

        double k2 = arm_length[1] * sin(angles[2] / 57.3);
        double k1 = arm_length[0] + arm_length[1] * cos(angles[2] / 57.3);
        double cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        double sin_j2 = sqrt(1 - (cos_j2) * (cos_j2));
        angles[1] = atan(sin_j2 / cos_j2 + 1e-9) * (57.3);
        angles[3] = a_sum * 57.3 - angles[1] - angles[2];

        // 验证姿态角的有效性
        if (angles[0] >= ANGLE_1_MIN && angles[0] <= ANGLE_1_MAX &&
            angles[1] >= ANGLE_2_MIN && angles[1] <= ANGLE_2_MAX &&
            angles[2] >= ANGLE_3_MIN && angles[2] <= ANGLE_3_MAX &&
            angles[3] >= ANGLE_4_MIN && angles[3] <= ANGLE_4_MAX)
        {
            now_angle[0] = angles[0];
            now_angle[1] = angles[1];
            now_angle[2] = angles[2];
            now_angle[3] = angles[3];
        }
    }
    else if (servo_mode_flag == 0)
    {
        int m = 0; // 有效角度计数器
        int n = 0;
        for (int i = 0; i <= 180; i++)
        {
            double a_sum = PI * i / 180;
            double len = sqrt((target_y * target_y) + (target_x * target_x));
            double high = target_z;

            double L = len - arm_length[2] * sin(a_sum);
            double H = high - arm_length[2] * cos(a_sum) - base_height;

            double cos_j3 = ((L * L) + (H * H) - (arm_length[0] * arm_length[0]) - (arm_length[1] * arm_length[1])) /
                            (2 * arm_length[0] * arm_length[1]);
            double sin_j3 = sqrt(1 - (cos_j3 * cos_j3));
            angles[2] = atan(sin_j3 / (cos_j3 + 1e-9)) * (57.3);

            double k2 = arm_length[1] * sin(angles[2] / 57.3);
            double k1 = arm_length[0] + arm_length[1] * cos(angles[2] / 57.3);
            double cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
            double sin_j2 = sqrt(1 - (cos_j2) * (cos_j2));
            angles[1] = atan(sin_j2 / cos_j2 + 1e-9) * (57.3);
            angles[3] = a_sum * 57.3 - angles[1] - angles[2];

            // 验证姿态角的有效性
            if (angles[0] >= ANGLE_1_MIN && angles[0] <= ANGLE_1_MAX &&
                angles[1] >= ANGLE_2_MIN && angles[1] <= ANGLE_2_MAX &&
                angles[2] >= ANGLE_3_MIN && angles[2] <= ANGLE_3_MAX &&
                angles[3] >= ANGLE_4_MIN && angles[3] <= ANGLE_4_MAX)
            {
                // 存储有效的姿态角
                n++;
            }
        }
        for (int i = 0; i <= 180; i++)
        {
            double a_sum = PI * i / 180;
            double len = sqrt((target_y * target_y) + (target_x * target_x));
            double high = target_z;

            double L = len - arm_length[2] * sin(a_sum);
            double H = high - arm_length[2] * cos(a_sum) - base_height;

            double cos_j3 = ((L * L) + (H * H) - (arm_length[0] * arm_length[0]) - (arm_length[1] * arm_length[1])) /
                            (2 * arm_length[0] * arm_length[1]);
            double sin_j3 = sqrt(1 - (cos_j3 * cos_j3));
            angles[2] = atan(sin_j3 / (cos_j3 + 1e-9)) * (57.3);

            double k2 = arm_length[1] * sin(angles[2] / 57.3);
            double k1 = arm_length[0] + arm_length[1] * cos(angles[2] / 57.3);
            double cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
            double sin_j2 = sqrt(1 - (cos_j2) * (cos_j2));
            angles[1] = atan(sin_j2 / cos_j2 + 1e-9) * (57.3);
            angles[3] = a_sum * 57.3 - angles[1] - angles[2];
            // 验证姿态角的有效性
            if (angles[0] >= ANGLE_1_MIN && angles[0] <= ANGLE_1_MAX &&
                angles[1] >= ANGLE_2_MIN && angles[1] <= ANGLE_2_MAX &&
                angles[2] >= ANGLE_3_MIN && angles[2] <= ANGLE_3_MAX &&
                angles[3] >= ANGLE_4_MIN && angles[3] <= ANGLE_4_MAX)
            {
                // 存储有效的姿态角
                m++;
                if (m == n / 2 || m == (n + 1) / 2)
                {
                    now_angle[0] = angles[0];
                    now_angle[1] = angles[1];
                    now_angle[2] = angles[2];
                    now_angle[3] = angles[3];
                    break;
                }
            }
        }
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
    double step_time = 10; // 每个循环的延迟时间（毫秒）
    double max_speed = 0.85; // 速度因子，越大表示每次转动的角度越大
    double current_angle[4] = {now_angle[0], now_angle[1], now_angle[2], now_angle[3]};

    // 将目标角度差距转化为步数
    double delta[4];
    for (int i = 0; i < 4; i++) {
        delta[i] = (i == 0 ? target_angle_1 : (i == 1 ? target_angle_2 : (i == 2 ? target_angle_3 : target_angle_4))) - current_angle[i];
    }

    // 计算总的转动次数（以最大速度步数作为依据）
    int steps[4];
    int max_steps = 0; // 最大步骤数
    for (int i = 0; i < 4; i++) {
        steps[i] = (int)(fabs(delta[i]) / max_speed) + 1;
        if (steps[i] > max_steps) {
            max_steps = steps[i]; // 记录最大步骤数
        }
    }

    // 执行移动到目标角度
    for (int i = 0; i <= max_steps; i++)
    {
        double proportion = (double)i / max_steps;
        for (int j = 0; j < 4; j++) {
            now_angle[j] = current_angle[j] + delta[j] * proportion;
        }
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
            servo_move_to_target(0, -15, 90, 95);
//            now_x = 0, now_y = 5.61, now_z = 19.87;
            break;
        case 2:
            servo_move_to_target(0, 55, 75, 35);
          //  now_x = 0, now_y = 0, now_z = 0;
            break;
        case 3:
            servo_move_to_target(0, -10, -20, 35);
            //todo: 最后移动框
           // now_x = 0, now_y = 0, now_z = 0;
            break;
        case 4:
            servo_move_to_target(0, 0, 0, 0);
          //  now_x = 0, now_y = 0, now_z = 0;
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
        if (r->Button[3]==0x01)
            preset_target(4);

        if (r->Switch[0]==0x01&&r->Switch[1]==0x00)
        {
            servo_mode_flag = 1;
            servo_move_delta(r->rocker[0].y_position,
                             r->rocker[0].x_position,
                             r->rocker[1].y_position);//-500~500,作为y轴增加量
        }

        /*
        if (r->Switch[1]==0x01&&r->Switch[0]==0x00)
        {
            servo_mode_flag = 0;
            servo_move_delta(r->rocker[0].x_position,
                             r->rocker[0].y_position,
                             r->rocker[1].y_position);//-500~500,作为x轴增加量
        }
*/
        osDelay(1);
    }
}