/* ahrs.c — 重构版（可直接替换）
 * 关键改动：
 * 1) Mahony 互补滤波加入加速度门控与积分限幅
 * 2) 倾斜检测加入：方向签名/低通/迟滞与回差，默认阈值对齐 APEX=35°
 * 3) 世界/机体系线性加速度保留；接口基本不变
 */

#include "ahrs.h"
#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846

/*======================= 可调参数（按需修改） =======================*/
/* —— 滤波增益（Mahony 风格） —— */
#ifndef AHRS_KP
#define AHRS_KP          2.0f      /* 比例增益 Kp（姿态误差 → 陀螺修正） */
#endif
#ifndef AHRS_KI
#define AHRS_KI          0.05f     /* 积分增益 Ki（估计去除陀螺零偏） */
#endif
#define AHRS_I_MAX       0.5f      /* 积分限幅（防飘） */

/* —— 加速度门控（只有 |a|≈1g 时信任重力方向） —— */
#define ACC_TRUST_LOW_G  0.85f     /* 低阈：<0.85g 认为线加速度/干扰大 */
#define ACC_TRUST_HIGH_G 1.15f     /* 高阈：>1.15g 同上 */
#define ACC_TRUST_GAIN   1.0f      /* 门控通过后的权重 0..1（1=全权重） */

/* —— 倾斜检测（机体系）参数，默认与 APEX 类似 —— */
#define TILT_TH_ON_DEG   35.0f     /* 倾斜“触发阈值”（度） */
#define TILT_TH_OFF_DEG  30.0f     /* 倾斜“恢复阈值”（回差，度） */
#define TILT_MERGE_DEG    3.0f     /* roll/pitch 接近时的择优阈值 */
#define TILT_ON_FRAMES   16        /* 连续满足 N 帧才判定进入（约 40ms@400Hz） */
#define TILT_OFF_FRAMES  20        /* 连续恢复 N 帧才判定退出 */

/* —— 线性加速度运动判定 —— */
#define MOVE_TH_G        0.12f     /* 平移阈值（g） */
#define MOVE_HYS_G       0.02f     /* 回差（g） */

/* —— 方向签名（用于你板载坐标与数学坐标的差异：必要时把宏改为 -1） —— */

#define ROLL_DIR_SIGN     (-1.0f)


#define PITCH_DIR_SIGN    (-1.0f)


#define FORWARD_DIR_SIGN  (-1.0f)  /* 前(+)/后(-) 方向修正 */


#define LEFT_DIR_SIGN     (-1.0f)  /* 左(+)/右(-) 方向修正 */


/* —— LPF α —— */
#define LPF_ALPHA_BODY    0.20f
#define LPF_ALPHA_WORLD   0.20f

/*======================= 内部状态 =======================*/
static float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f; /* 姿态四元数 */
static float exInt=0.0f, eyInt=0.0f, ezInt=0.0f;


static inline void lpf_init(lpf1_t* f, float a)
{
	f->alpha=fminf(fmaxf(a,0.001f),0.999f);
	f->y=0; f->inited=0;
}
static inline float lpf_apply(lpf1_t* f, float x)
{ 
	if(!f->inited)
	{
			f->y=x;
			f->inited=1;
	} 
	
	f->y += f->alpha*(x-f->y);
	
	return f->y;
}

/* 机体系/世界系水平分量 LPF */
static lpf1_t lpf_body_x, lpf_body_y, lpf_world_x, lpf_world_y;

/* 倾斜检测状态机 */
enum { TILT_NONE=0, TILT_LEFT=1, TILT_RIGHT=2, TILT_FWD=3, TILT_BWD=4 };
static int      tilt_state = TILT_NONE;
static uint16_t tilt_on_cnt = 0, tilt_off_cnt = 0;

/*======================= 工具函数 =======================*/
static inline void clamp_vec3(float* x, float* y, float* z, float maxmag){
    float n = sqrtf((*x)*(*x)+(*y)*(*y)+(*z)*(*z));
    if(n>maxmag && n>1e-9f){ float s=maxmag/n; *x*=s; *y*=s; *z*=s; }
}

/*======================= 接口实现 =======================*/
void AHRS_LPF_SetAlpha(float alpha_body, float alpha_world){
    lpf_init(&lpf_body_x,  alpha_body);
    lpf_init(&lpf_body_y,  alpha_body);
    lpf_init(&lpf_world_x, alpha_world);
    lpf_init(&lpf_world_y, alpha_world);
}

void AHRS_Reset(void){
    q0=1.0f; q1=q2=q3=0.0f;
    exInt=eyInt=ezInt=0.0f;
    tilt_state = TILT_NONE; tilt_on_cnt=tilt_off_cnt=0;
    AHRS_LPF_SetAlpha(LPF_ALPHA_BODY, LPF_ALPHA_WORLD);
}

/* Mahony 风格互补滤波 + 加速度门控 + 积分限幅
 * 参考：Mahony/Madgwick 以加速度估计重力方向，反馈修正陀螺积分误差。:contentReference[oaicite:3]{index=3}
 */
void AHRS_Update(float gx, float gy, float gz,  /* rad/s */
                 float ax, float ay, float az,  /* g */
                 float dt, ahrs_euler_t* out)
{
    /* --- 加速度门控：仅在 |a|≈1g 时信任重力方向 --- */
    float amag = sqrtf(ax*ax + ay*ay + az*az);
    float use_acc = (amag>ACC_TRUST_LOW_G && amag<ACC_TRUST_HIGH_G) ? ACC_TRUST_GAIN : 0.0f;

    /* 归一化加速度（避免除零） */
    float inv_norm = (amag>1e-6f)? (1.0f/amag) : 1.0f;
    float axn = ax*inv_norm, ayn = ay*inv_norm, azn = az*inv_norm;

    /* 预计重力方向（机体系） */
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* 误差（a × v），加权到 0..1 */
    float ex = (ayn*vz - azn*vy)*use_acc;
    float ey = (azn*vx - axn*vz)*use_acc;
    float ez = (axn*vy - ayn*vx)*use_acc;

    /* 积分限幅（抗饱和/防飘） */
    exInt += AHRS_KI * ex * dt;
    eyInt += AHRS_KI * ey * dt;
    ezInt += AHRS_KI * ez * dt;
    clamp_vec3(&exInt,&eyInt,&ezInt, AHRS_I_MAX);

    /* 纠正陀螺 */
    gx += AHRS_KP*ex + exInt;
    gy += AHRS_KP*ey + eyInt;
    gz += AHRS_KP*ez + ezInt;

    /* 四元数积分 */
    float q0_dot = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float q1_dot = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float q2_dot = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float q3_dot = 0.5f * ( q0*gz + q1*gy - q2*gx);
    q0 += q0_dot*dt; q1 += q1_dot*dt; q2 += q2_dot*dt; q3 += q3_dot*dt;

    /* 归一化四元数 */
    float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(qn<1e-9f){ AHRS_Reset(); qn=1.0f; }
    q0/=qn; q1/=qn; q2/=qn; q3/=qn;

    /* 输出欧拉角（3-2-1/Tait-Bryan） */
    if(out){
        /* 注意：这里遵循常见配方，参考资料详述转换与奇异性处理。:contentReference[oaicite:4]{index=4} */
        float sinr_cosp = 2.0f*(q0*q1 + q2*q3);
        float cosr_cosp = 1.0f - 2.0f*(q1*q1 + q2*q2);
        float roll  = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f*(q0*q2 - q3*q1);
        float pitch = (fabsf(sinp)>=1.0f) ? copysignf((float)M_PI/2.0f, sinp) : asinf(sinp);

        float siny_cosp = 2.0f*(q0*q3 + q1*q2);
        float cosy_cosp = 1.0f - 2.0f*(q2*q2 + q3*q3);
        float yaw   = atan2f(siny_cosp, cosy_cosp);

        /* 方向签名（如机体轴与数学轴不一致，可在此翻转） */
        roll  = ROLL_DIR_SIGN  * roll;
        pitch = PITCH_DIR_SIGN * pitch;

        out->roll_deg  = roll  * 57.29578f;
        out->pitch_deg = pitch * 57.29578f;
        out->yaw_deg   = yaw   * 57.29578f;
    }
}

/* 去重力（机体系） */
void AHRS_RemoveGravity(float ax_g, float ay_g, float az_g,
                        float* lin_x_g, float* lin_y_g, float* lin_z_g)
{
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    *lin_x_g = ax_g - vx;
    *lin_y_g = ay_g - vy;
    *lin_z_g = az_g - vz;
}

/* 机体->世界 旋转（Z 轴向上） */
void AHRS_BodyToWorld(float bx, float by, float bz,
                      float* wx, float* wy, float* wz)
{
    float a=q0,b=q1,c=q2,d=q3;  /* R = R(q) */
    float a2=a*a, b2=b*b, c2=c*c, d2=d*d;
    float R11=a2+b2-c2-d2, R12=2.0f*(b*c-a*d), R13=2.0f*(b*d+a*c);
    float R21=2.0f*(b*c+a*d), R22=a2-b2+c2-d2, R23=2.0f*(c*d-a*b);
    float R31=2.0f*(b*d-a*c), R32=2.0f*(c*d+a*b), R33=a2-b2-c2+d2;
    *wx = R11*bx + R12*by + R13*bz;
    *wy = R21*bx + R22*by + R23*bz;
    *wz = R31*bx + R32*by + R33*bz;
}

/*=================== 倾斜/平移检测（机体系，带迟滞/状态机） ===================*/
void Detect_TiltMotion_Body(float roll_deg, float pitch_deg,
                            float ax_g, float ay_g, float az_g,
                            tilt_motion_t* out)
{
    memset(out, 0, sizeof(*out));

    /* 方向签名修正，避免“总是右倾”的坐标约定问题 */
    float roll  = ROLL_DIR_SIGN  * roll_deg;
    float pitch = PITCH_DIR_SIGN * pitch_deg;

    /* 倾斜主导轴判定（带 merge 窗口） */
    float absr=fabsf(roll), absp=fabsf(pitch);
    int   cand = TILT_NONE;           /* 候选状态 */
    float cand_deg = 0.0f;

    if(absr >= absp + TILT_MERGE_DEG){
        if(roll  > TILT_TH_ON_DEG)  { cand=TILT_RIGHT; cand_deg=absr; }
        else if(roll < -TILT_TH_ON_DEG){ cand=TILT_LEFT;  cand_deg=absr; }
    }else if(absp >= absr + TILT_MERGE_DEG){
        if(pitch >  TILT_TH_ON_DEG) { cand=TILT_FWD;   cand_deg=absp; }
        else if(pitch<-TILT_TH_ON_DEG){ cand=TILT_BWD;   cand_deg=absp; }
    }else{ /* 接近：择大 */
        if     (roll  >  TILT_TH_ON_DEG && absr>=absp){ cand=TILT_RIGHT; cand_deg=absr; }
        else if(roll  < -TILT_TH_ON_DEG && absr>=absp){ cand=TILT_LEFT;  cand_deg=absr; }
        else if(pitch >  TILT_TH_ON_DEG && absp> absr){ cand=TILT_FWD;   cand_deg=absp; }
        else if(pitch < -TILT_TH_ON_DEG && absp> absr){ cand=TILT_BWD;   cand_deg=absp; }
    }

    /* 迟滞：进入/退出需连续 N 帧
       - 若 cand==tilt_state：累计“退出计数”清零
       - 若 cand!=tilt_state 且 cand!=NONE：累计“进入计数”到阈值才切换
       - 若 cand==NONE：累计“退出计数”到阈值才回到 NONE
     */
    if(cand==TILT_NONE){
        /* 检查是否达到退出条件（roll/pitch 都回到 OFF 阈值内） */
        if( (fabsf(roll)  < TILT_TH_OFF_DEG) &&
            (fabsf(pitch) < TILT_TH_OFF_DEG) ){
            if(tilt_off_cnt < 0xFFFF) tilt_off_cnt++;
            tilt_on_cnt = 0;
            if(tilt_off_cnt >= TILT_OFF_FRAMES){ tilt_state=TILT_NONE; tilt_off_cnt=0; }
        }else{
            tilt_off_cnt = 0; tilt_on_cnt = 0;
        }
    }else{
        /* cand 为某个方向 */
        if(cand == tilt_state){
            tilt_on_cnt = 0;
            /* 如果当前方向维持，但角度跌回 OFF 阈值内，开始退出计数 */
            if( (cand==TILT_RIGHT && roll  <  TILT_TH_OFF_DEG) ||
                (cand==TILT_LEFT  && roll  > -TILT_TH_OFF_DEG) ||
                (cand==TILT_FWD   && pitch <  TILT_TH_OFF_DEG) ||
                (cand==TILT_BWD   && pitch > -TILT_TH_OFF_DEG) ){
                if(tilt_off_cnt < 0xFFFF) tilt_off_cnt++;
                if(tilt_off_cnt >= TILT_OFF_FRAMES){ tilt_state=TILT_NONE; tilt_off_cnt=0; }
            }else{
                tilt_off_cnt = 0;
            }
        }else{
            /* cand != 当前状态：尝试切换 */
            if(tilt_on_cnt < 0xFFFF) tilt_on_cnt++;
            tilt_off_cnt = 0;
            if(tilt_on_cnt >= TILT_ON_FRAMES){ tilt_state=cand; tilt_on_cnt=0; }
        }
    }

    /* 输出当前稳定状态 */
    switch(tilt_state){
        case TILT_LEFT:  out->tilt_left = 1;  out->tilt_degree = fabsf(roll);  break;
        case TILT_RIGHT: out->tilt_right= 1;  out->tilt_degree = fabsf(roll);  break;
        case TILT_FWD:   out->tilt_forward=1; out->tilt_degree = fabsf(pitch); break;
        case TILT_BWD:   out->tilt_backward=1;out->tilt_degree = fabsf(pitch); break;
        default: break;
    }

    /* 线性加速度与运动方向（机体系），含一阶低通与方向签名 */
    float lx, ly, lz; AHRS_RemoveGravity(ax_g, ay_g, az_g, &lx, &ly, &lz);
    float lx_f = lpf_apply(&lpf_body_x, FORWARD_DIR_SIGN * lx);
    float ly_f = lpf_apply(&lpf_body_y, LEFT_DIR_SIGN    * ly);
    float horiz = sqrtf(lx_f*lx_f + ly_f*ly_f);
    out->move_g = horiz;

    if (horiz > (MOVE_TH_G + MOVE_HYS_G)) {
        if (ly_f >  MOVE_TH_G) out->move_left     = 1;
        if (ly_f < -MOVE_TH_G) out->move_right    = 1;
        if (lx_f >  MOVE_TH_G) out->move_forward  = 1;
        if (lx_f < -MOVE_TH_G) out->move_backward = 1;
    }
}

/*=================== 世界系版本（如需，可继续保留/调用） ===================*/
void Detect_TiltMotion_World(float roll_deg, float pitch_deg,
                             float ax_g, float ay_g, float az_g,
                             tilt_motion_t* out)
{
    memset(out, 0, sizeof(*out));

    /* 与机体系同样的签名与阈值策略，可按需改 */
    float roll  = ROLL_DIR_SIGN  * roll_deg;
    float pitch = PITCH_DIR_SIGN * pitch_deg;

    /* 倾斜方向判定 */
    float absr=fabsf(roll), absp=fabsf(pitch);
    int cand=TILT_NONE; float cand_deg=0.0f;
    if(absr >= absp + TILT_MERGE_DEG){
        if(roll  > TILT_TH_ON_DEG) cand=TILT_RIGHT, cand_deg=absr;
        else if(roll < -TILT_TH_ON_DEG) cand=TILT_LEFT, cand_deg=absr;
    }else if(absp >= absr + TILT_MERGE_DEG){
        if(pitch >  TILT_TH_ON_DEG) cand=TILT_FWD, cand_deg=absp;
        else if(pitch < -TILT_TH_ON_DEG) cand=TILT_BWD, cand_deg=absp;
    }

    /* 直接输出（世界系版本不做状态机，也可复用上面的状态机以一致行为） */
    switch(cand){
        case TILT_LEFT:  out->tilt_left=1;  out->tilt_degree=cand_deg; break;
        case TILT_RIGHT: out->tilt_right=1; out->tilt_degree=cand_deg; break;
        case TILT_FWD:   out->tilt_forward=1; out->tilt_degree=cand_deg; break;
        case TILT_BWD:   out->tilt_backward=1; out->tilt_degree=cand_deg; break;
        default: break;
    }

    /* 线性加速度：Body→World */
    float lx_b, ly_b, lz_b; AHRS_RemoveGravity(ax_g, ay_g, az_g, &lx_b, &ly_b, &lz_b);
    float lx_w, ly_w, lz_w; AHRS_BodyToWorld(lx_b, ly_b, lz_b, &lx_w, &ly_w, &lz_w);
    float lxw_f = lpf_apply(&lpf_world_x, FORWARD_DIR_SIGN * lx_w);
    float lyw_f = lpf_apply(&lpf_world_y, LEFT_DIR_SIGN    * ly_w);
    float horiz = sqrtf(lxw_f*lxw_f + lyw_f*lyw_f);
    out->move_g = horiz;

    if (horiz > (MOVE_TH_G + MOVE_HYS_G)) {
        if (lyw_f >  MOVE_TH_G) out->move_left     = 1;
        if (lyw_f < -MOVE_TH_G) out->move_right    = 1;
        if (lxw_f >  MOVE_TH_G) out->move_forward  = 1;
        if (lxw_f < -MOVE_TH_G) out->move_backward = 1;
    }
}
