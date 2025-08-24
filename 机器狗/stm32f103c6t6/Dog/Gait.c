#include "Gait.h"

void swing_curve_generate(float t, float Tf, float xt, float zh, Motion_Trajectory* MT) {
    float tdtf = t / Tf;
    float tdtf2 = tdtf * tdtf;
    float tdtf3 = tdtf * tdtf2;
    if (t < Tf * 0.25) {
        MT->swing_xf = 0;
    } else if (t < Tf * 0.75) {
        MT->swing_xf = -16 * xt * tdtf3 + 24 * xt * tdtf2 - 9 * xt * tdtf + xt;
    } else {
        MT->swing_xf = xt;
    }

    if (t < Tf / 2) {
        MT->swing_zf = -16 * zh * tdtf3 + 12 * zh * tdtf2;
    } else {
        MT->swing_zf = -4 * zh * tdtf2 + 4 * zh * tdtf;
    }
}

void support_curve_generate(float t, float Tf, float xt, Motion_Trajectory* MT) {
    MT->support_xf = xt * (1 - t) / (1 - Tf);
    MT->support_zf = 0;
}

void trot(float t, float xt, float zt, int8_t r1, int8_t r4, int8_t r2, int8_t r3, Motion_Trajectory* MT) {
    float Tf = 0.5;
    if (t < Tf) {
        swing_curve_generate(t, Tf, xt, zt, MT);
        support_curve_generate(t + Tf, Tf, xt, MT);

        MT->x1 = MT->swing_xf * r1;
        MT->x2 = MT->support_xf * r2;
        MT->x3 = MT->swing_xf * r3;
        MT->x4 = MT->support_xf * r4;

        MT->z1 = MT->swing_zf;
        MT->z2 = MT->support_zf;
        MT->z3 = MT->swing_zf;
        MT->z4 = MT->support_zf;
    } else {
        swing_curve_generate(t - Tf, Tf, xt, zt, MT);
        support_curve_generate(t, Tf, xt, MT);
        
        MT->x1 = MT->support_xf * r1;
        MT->x2 = MT->swing_xf * r2;
        MT->x3 = MT->support_xf * r3;
        MT->x4 = MT->swing_xf * r4;

        MT->z1 = MT->support_zf;
        MT->z2 = MT->swing_zf;
        MT->z3 = MT->support_zf;
        MT->z4 = MT->swing_zf;
    }
}
