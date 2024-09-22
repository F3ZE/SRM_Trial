#include <servo.h>
#include <stdio.h>

void bluetooth_receive_data(char* command) {
    int dir = 0, mode= 0, delta = 0;

    // 首先根据第一个字符判断命令类型
    switch (command[0]) {
        case 'X':  // 处理 X 轴移动
            sscanf(command, "X%dD%d", &dir, &delta);  // 提取方向和delta值
            if ((dir==0 || dir ==1) && delta >= 0 && delta <=100)
            {
                move_target('X', dir, delta);
            }
            break;

        case 'Y':  // 处理 Y 轴移动
            sscanf(command, "Y%dD%d", &dir, &delta);
            if ((dir==0 || dir ==1) && delta >= 0 && delta <=100)
            {
                move_target('Y', dir, delta);
            }
            break;

        case 'Z':  // 处理 Z 轴移动
            sscanf(command, "Z%dD%d", &dir, &delta);
            if ((dir==0 || dir ==1) && delta >= 0 && delta <=100)
            {
                move_target('Z', dir, delta);
            }
            break;

        case 'M':  // 处理预设目标
            sscanf(command, "M%d", &mode);  // 仅提取预设值
            if (mode<=3 && mode >=1)
            {
                preset_target(mode);
            }
            break;

        default:
            printf("Unknown command: %s\n", command);
            break;
    }
}
