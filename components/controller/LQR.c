#include "LQR.h"

#include "main.h"

#define LimitMax(input, max)       \
    {                              \
        if (input > max) {         \
            input = max;           \
        } else if (input < -max) { \
            input = -max;          \
        }                          \
    }

void LQR_init(Lqr_t *lqr, const fp32 K[3], fp32 max_out, fp32 max_iout, fp32 dt)
{
    if(lqr == NULL || K == NULL)
    {
        return;
    }

    lqr->k1 = K[0];
    lqr->k2 = K[1];
    lqr->ki = K[2];
    lqr->dt = dt;
    lqr->max_out = max_out;
    lqr->max_iout = max_iout;
    lqr->angle_ref = 0.0f;
    lqr->angle_fdb = 0.0f;
    lqr->omega_fdb = 0.0f;
    lqr->e_int = 0.0f;
    lqr->out = 0.0f;

}

void LQR_param_update(Lqr_t *lqr, const fp32 K[3])
{
    if(lqr == NULL || K == NULL)
    {
        return;
    }
    lqr->k1 = K[0];
    lqr->k2 = K[1];
    lqr->ki = K[2];

}

fp32 LQR_calc(Lqr_t *lqr, fp32 err, fp32 omega_fdb)
{
     fp32 e_int_new;
    fp32 out_unsat;

    if (lqr == NULL) {
        return 0.0f;
    }

    /* 离散积分：先试算本次累加后的值 */
    e_int_new = lqr->e_int + err * lqr->dt;
    LimitMax(e_int_new, lqr->max_iout);   

    /* 试算未限幅总输出 */
    out_unsat = -(lqr->k1 * err + lqr->k2 * omega_fdb + lqr->ki * e_int_new);

    /* 条件抗饱和：输出饱和时冻结积分，防止 windup */
    if (out_unsat > -lqr->max_out && out_unsat < lqr->max_out) {
        lqr->e_int = e_int_new;           
    }
    

    /* 用最终 e_int 重算输出并限幅 */
    lqr->out = -(lqr->k1 * err + lqr->k2 * omega_fdb + lqr->ki * lqr->e_int);
    LimitMax(lqr->out, lqr->max_out);

    return lqr->out;
}

void LQR_clear(Lqr_t *lqr)
{
    if(lqr == NULL)
    {
        return;
    }
    lqr->e_int = 0.0f;
    lqr->out = 0.0f;
    lqr->angle_ref = 0.0f;
    lqr->angle_fdb = 0.0f;
    lqr->omega_fdb = 0.0f;
}

