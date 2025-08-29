/* ahrs.c �� �ع��棨��ֱ���滻��
 * �ؼ��Ķ���
 * 1) Mahony �����˲�������ٶ��ſ�������޷�
 * 2) ��б�����룺����ǩ��/��ͨ/������زĬ����ֵ���� APEX=35��
 * 3) ����/����ϵ���Լ��ٶȱ������ӿڻ�������
 */

#include "ahrs.h"
#include <math.h>
#include <string.h>

#define M_PI 3.14159265358979323846

/*======================= �ɵ������������޸ģ� =======================*/
/* ���� �˲����棨Mahony ��� ���� */
#ifndef AHRS_KP
#define AHRS_KP          2.0f      /* �������� Kp����̬��� �� ���������� */
#endif
#ifndef AHRS_KI
#define AHRS_KI          0.05f     /* �������� Ki������ȥ��������ƫ�� */
#endif
#define AHRS_I_MAX       0.5f      /* �����޷�����Ʈ�� */

/* ���� ���ٶ��ſأ�ֻ�� |a|��1g ʱ������������ ���� */
#define ACC_TRUST_LOW_G  0.85f     /* ���У�<0.85g ��Ϊ�߼��ٶ�/���Ŵ� */
#define ACC_TRUST_HIGH_G 1.15f     /* ���У�>1.15g ͬ�� */
#define ACC_TRUST_GAIN   1.0f      /* �ſ�ͨ�����Ȩ�� 0..1��1=ȫȨ�أ� */

/* ���� ��б��⣨����ϵ��������Ĭ���� APEX ���� ���� */
#define TILT_TH_ON_DEG   35.0f     /* ��б��������ֵ�����ȣ� */
#define TILT_TH_OFF_DEG  30.0f     /* ��б���ָ���ֵ�����ز�ȣ� */
#define TILT_MERGE_DEG    3.0f     /* roll/pitch �ӽ�ʱ��������ֵ */
#define TILT_ON_FRAMES   16        /* �������� N ֡���ж����루Լ 40ms@400Hz�� */
#define TILT_OFF_FRAMES  20        /* �����ָ� N ֡���ж��˳� */

/* ���� ���Լ��ٶ��˶��ж� ���� */
#define MOVE_TH_G        0.12f     /* ƽ����ֵ��g�� */
#define MOVE_HYS_G       0.02f     /* �زg�� */

/* ���� ����ǩ���������������������ѧ����Ĳ��죺��Ҫʱ�Ѻ��Ϊ -1�� ���� */

#define ROLL_DIR_SIGN     (-1.0f)


#define PITCH_DIR_SIGN    (-1.0f)


#define FORWARD_DIR_SIGN  (-1.0f)  /* ǰ(+)/��(-) �������� */


#define LEFT_DIR_SIGN     (-1.0f)  /* ��(+)/��(-) �������� */


/* ���� LPF �� ���� */
#define LPF_ALPHA_BODY    0.20f
#define LPF_ALPHA_WORLD   0.20f

/*======================= �ڲ�״̬ =======================*/
static float q0=1.0f, q1=0.0f, q2=0.0f, q3=0.0f; /* ��̬��Ԫ�� */
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

/* ����ϵ/����ϵˮƽ���� LPF */
static lpf1_t lpf_body_x, lpf_body_y, lpf_world_x, lpf_world_y;

/* ��б���״̬�� */
enum { TILT_NONE=0, TILT_LEFT=1, TILT_RIGHT=2, TILT_FWD=3, TILT_BWD=4 };
static int      tilt_state = TILT_NONE;
static uint16_t tilt_on_cnt = 0, tilt_off_cnt = 0;

/*======================= ���ߺ��� =======================*/
static inline void clamp_vec3(float* x, float* y, float* z, float maxmag){
    float n = sqrtf((*x)*(*x)+(*y)*(*y)+(*z)*(*z));
    if(n>maxmag && n>1e-9f){ float s=maxmag/n; *x*=s; *y*=s; *z*=s; }
}

/*======================= �ӿ�ʵ�� =======================*/
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

/* Mahony ��񻥲��˲� + ���ٶ��ſ� + �����޷�
 * �ο���Mahony/Madgwick �Լ��ٶȹ����������򣬷����������ݻ�����:contentReference[oaicite:3]{index=3}
 */
void AHRS_Update(float gx, float gy, float gz,  /* rad/s */
                 float ax, float ay, float az,  /* g */
                 float dt, ahrs_euler_t* out)
{
    /* --- ���ٶ��ſأ����� |a|��1g ʱ������������ --- */
    float amag = sqrtf(ax*ax + ay*ay + az*az);
    float use_acc = (amag>ACC_TRUST_LOW_G && amag<ACC_TRUST_HIGH_G) ? ACC_TRUST_GAIN : 0.0f;

    /* ��һ�����ٶȣ�������㣩 */
    float inv_norm = (amag>1e-6f)? (1.0f/amag) : 1.0f;
    float axn = ax*inv_norm, ayn = ay*inv_norm, azn = az*inv_norm;

    /* Ԥ���������򣨻���ϵ�� */
    float vx = 2.0f*(q1*q3 - q0*q2);
    float vy = 2.0f*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* ��a �� v������Ȩ�� 0..1 */
    float ex = (ayn*vz - azn*vy)*use_acc;
    float ey = (azn*vx - axn*vz)*use_acc;
    float ez = (axn*vy - ayn*vx)*use_acc;

    /* �����޷���������/��Ʈ�� */
    exInt += AHRS_KI * ex * dt;
    eyInt += AHRS_KI * ey * dt;
    ezInt += AHRS_KI * ez * dt;
    clamp_vec3(&exInt,&eyInt,&ezInt, AHRS_I_MAX);

    /* �������� */
    gx += AHRS_KP*ex + exInt;
    gy += AHRS_KP*ey + eyInt;
    gz += AHRS_KP*ez + ezInt;

    /* ��Ԫ������ */
    float q0_dot = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float q1_dot = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float q2_dot = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float q3_dot = 0.5f * ( q0*gz + q1*gy - q2*gx);
    q0 += q0_dot*dt; q1 += q1_dot*dt; q2 += q2_dot*dt; q3 += q3_dot*dt;

    /* ��һ����Ԫ�� */
    float qn = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(qn<1e-9f){ AHRS_Reset(); qn=1.0f; }
    q0/=qn; q1/=qn; q2/=qn; q3/=qn;

    /* ���ŷ���ǣ�3-2-1/Tait-Bryan�� */
    if(out){
        /* ע�⣺������ѭ�����䷽���ο���������ת���������Դ���:contentReference[oaicite:4]{index=4} */
        float sinr_cosp = 2.0f*(q0*q1 + q2*q3);
        float cosr_cosp = 1.0f - 2.0f*(q1*q1 + q2*q2);
        float roll  = atan2f(sinr_cosp, cosr_cosp);

        float sinp = 2.0f*(q0*q2 - q3*q1);
        float pitch = (fabsf(sinp)>=1.0f) ? copysignf((float)M_PI/2.0f, sinp) : asinf(sinp);

        float siny_cosp = 2.0f*(q0*q3 + q1*q2);
        float cosy_cosp = 1.0f - 2.0f*(q2*q2 + q3*q3);
        float yaw   = atan2f(siny_cosp, cosy_cosp);

        /* ����ǩ���������������ѧ�᲻һ�£����ڴ˷�ת�� */
        roll  = ROLL_DIR_SIGN  * roll;
        pitch = PITCH_DIR_SIGN * pitch;

        out->roll_deg  = roll  * 57.29578f;
        out->pitch_deg = pitch * 57.29578f;
        out->yaw_deg   = yaw   * 57.29578f;
    }
}

/* ȥ����������ϵ�� */
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

/* ����->���� ��ת��Z �����ϣ� */
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

/*=================== ��б/ƽ�Ƽ�⣨����ϵ��������/״̬���� ===================*/
void Detect_TiltMotion_Body(float roll_deg, float pitch_deg,
                            float ax_g, float ay_g, float az_g,
                            tilt_motion_t* out)
{
    memset(out, 0, sizeof(*out));

    /* ����ǩ�����������⡰�������㡱������Լ������ */
    float roll  = ROLL_DIR_SIGN  * roll_deg;
    float pitch = PITCH_DIR_SIGN * pitch_deg;

    /* ��б�������ж����� merge ���ڣ� */
    float absr=fabsf(roll), absp=fabsf(pitch);
    int   cand = TILT_NONE;           /* ��ѡ״̬ */
    float cand_deg = 0.0f;

    if(absr >= absp + TILT_MERGE_DEG){
        if(roll  > TILT_TH_ON_DEG)  { cand=TILT_RIGHT; cand_deg=absr; }
        else if(roll < -TILT_TH_ON_DEG){ cand=TILT_LEFT;  cand_deg=absr; }
    }else if(absp >= absr + TILT_MERGE_DEG){
        if(pitch >  TILT_TH_ON_DEG) { cand=TILT_FWD;   cand_deg=absp; }
        else if(pitch<-TILT_TH_ON_DEG){ cand=TILT_BWD;   cand_deg=absp; }
    }else{ /* �ӽ������ */
        if     (roll  >  TILT_TH_ON_DEG && absr>=absp){ cand=TILT_RIGHT; cand_deg=absr; }
        else if(roll  < -TILT_TH_ON_DEG && absr>=absp){ cand=TILT_LEFT;  cand_deg=absr; }
        else if(pitch >  TILT_TH_ON_DEG && absp> absr){ cand=TILT_FWD;   cand_deg=absp; }
        else if(pitch < -TILT_TH_ON_DEG && absp> absr){ cand=TILT_BWD;   cand_deg=absp; }
    }

    /* ���ͣ�����/�˳������� N ֡
       - �� cand==tilt_state���ۼơ��˳�����������
       - �� cand!=tilt_state �� cand!=NONE���ۼơ��������������ֵ���л�
       - �� cand==NONE���ۼơ��˳�����������ֵ�Żص� NONE
     */
    if(cand==TILT_NONE){
        /* ����Ƿ�ﵽ�˳�������roll/pitch ���ص� OFF ��ֵ�ڣ� */
        if( (fabsf(roll)  < TILT_TH_OFF_DEG) &&
            (fabsf(pitch) < TILT_TH_OFF_DEG) ){
            if(tilt_off_cnt < 0xFFFF) tilt_off_cnt++;
            tilt_on_cnt = 0;
            if(tilt_off_cnt >= TILT_OFF_FRAMES){ tilt_state=TILT_NONE; tilt_off_cnt=0; }
        }else{
            tilt_off_cnt = 0; tilt_on_cnt = 0;
        }
    }else{
        /* cand Ϊĳ������ */
        if(cand == tilt_state){
            tilt_on_cnt = 0;
            /* �����ǰ����ά�֣����Ƕȵ��� OFF ��ֵ�ڣ���ʼ�˳����� */
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
            /* cand != ��ǰ״̬�������л� */
            if(tilt_on_cnt < 0xFFFF) tilt_on_cnt++;
            tilt_off_cnt = 0;
            if(tilt_on_cnt >= TILT_ON_FRAMES){ tilt_state=cand; tilt_on_cnt=0; }
        }
    }

    /* �����ǰ�ȶ�״̬ */
    switch(tilt_state){
        case TILT_LEFT:  out->tilt_left = 1;  out->tilt_degree = fabsf(roll);  break;
        case TILT_RIGHT: out->tilt_right= 1;  out->tilt_degree = fabsf(roll);  break;
        case TILT_FWD:   out->tilt_forward=1; out->tilt_degree = fabsf(pitch); break;
        case TILT_BWD:   out->tilt_backward=1;out->tilt_degree = fabsf(pitch); break;
        default: break;
    }

    /* ���Լ��ٶ����˶����򣨻���ϵ������һ�׵�ͨ�뷽��ǩ�� */
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

/*=================== ����ϵ�汾�����裬�ɼ�������/���ã� ===================*/
void Detect_TiltMotion_World(float roll_deg, float pitch_deg,
                             float ax_g, float ay_g, float az_g,
                             tilt_motion_t* out)
{
    memset(out, 0, sizeof(*out));

    /* �����ϵͬ����ǩ������ֵ���ԣ��ɰ���� */
    float roll  = ROLL_DIR_SIGN  * roll_deg;
    float pitch = PITCH_DIR_SIGN * pitch_deg;

    /* ��б�����ж� */
    float absr=fabsf(roll), absp=fabsf(pitch);
    int cand=TILT_NONE; float cand_deg=0.0f;
    if(absr >= absp + TILT_MERGE_DEG){
        if(roll  > TILT_TH_ON_DEG) cand=TILT_RIGHT, cand_deg=absr;
        else if(roll < -TILT_TH_ON_DEG) cand=TILT_LEFT, cand_deg=absr;
    }else if(absp >= absr + TILT_MERGE_DEG){
        if(pitch >  TILT_TH_ON_DEG) cand=TILT_FWD, cand_deg=absp;
        else if(pitch < -TILT_TH_ON_DEG) cand=TILT_BWD, cand_deg=absp;
    }

    /* ֱ�����������ϵ�汾����״̬����Ҳ�ɸ��������״̬����һ����Ϊ�� */
    switch(cand){
        case TILT_LEFT:  out->tilt_left=1;  out->tilt_degree=cand_deg; break;
        case TILT_RIGHT: out->tilt_right=1; out->tilt_degree=cand_deg; break;
        case TILT_FWD:   out->tilt_forward=1; out->tilt_degree=cand_deg; break;
        case TILT_BWD:   out->tilt_backward=1; out->tilt_degree=cand_deg; break;
        default: break;
    }

    /* ���Լ��ٶȣ�Body��World */
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
