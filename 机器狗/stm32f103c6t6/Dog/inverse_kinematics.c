#include "inverse_kinematics.h"
#include "dog_utils.h"
#include "math.h"

void inverse_funtion(float thigh_len, float shank_len, float tx, float ty, float* thigh_ang, float* shank_ang) {
    tx = -tx;
    *shank_ang = _PI - acos((tx * tx + ty * ty - thigh_len * thigh_len - shank_len * shank_len) / (-2 * thigh_len * shank_len));
    float fai_ang = acos((thigh_len * thigh_len + tx * tx + ty * ty - shank_len * shank_len) / (2 * thigh_len * sqrt(tx * tx + ty * ty)));
    if (tx > 0) {
        *thigh_ang = fabs(atan(ty / tx)) - fai_ang;
    } else if (tx < 0) {
        *thigh_ang = _PI - fabs(atan(ty / tx)) - fai_ang;
    } else {
        *thigh_ang = _PI_2 - fai_ang;
    }
    *shank_ang = 180 * (*shank_ang) / _PI;
    *thigh_ang = 180 * (*thigh_ang) / _PI;
}
