#ifndef VOFA_H
#define VOFA_H

#include "../../control/define.h"
#include "constant.h"

// 定义共用体数据结构
union SendToVofa {
    struct {
        float tz_camera;
        float yaw_camera;
        float roll_camera;
        float vx_camera;
        float bestyaw;
        float CurrentYaw;
        float sumdist;
        float visionYaw;
        float visionPitch;
        float RuiYaw;
        float RuiPitch;
        unsigned char TAIL[4];
    };
    uint8_t DATA[48];
};
union ReceiveFromVofa {
    int intValue;
};

extern SendToVofa toVofa;
extern ReceiveFromVofa rfvofa;

[[noreturn]] void FuncSendToVofa();

#endif