#ifndef __AHRS_H__
#define __AHRS_H__

#include <stdint.h>                                         // ��׼����
#include <stdbool.h>                                        // ��������

//==================== ����ϵ����Լ���������ӳ���ģ� ====================
// Լ��������ϵ Body +X ָ����ǰ����+Y ָ�����󡱣�+Z ָ�����ϡ�����ʵ���෴�ķ��š�
//#define FORWARD_DIR_SIGN  (+1.0f)                           // ����/���塰��ǰ����������� Body +X �ķ���
//#define LEFT_DIR_SIGN     (+1.0f)                           // ����/���塰������������� Body +Y �ķ���

//==================== ŷ�����������֮ǰ����һ�£� ======================
typedef struct
{
    float roll_deg;                                         // ����ǣ��ȣ�
    float pitch_deg;                                        // �����ǣ��ȣ�
    float yaw_deg;                                          // ����ǣ��ȣ�
} ahrs_euler_t;

//==================== ��б/ƽ�� �¼�����ṹ�� ==========================
typedef struct
{
    //���� ��б���� ����
    uint8_t tilt_left;                                      // ����б
    uint8_t tilt_right;                                     // ����б
    uint8_t tilt_forward;                                   // ǰ��б
    uint8_t tilt_backward;                                  // ����б

    //���� ƽ�Ʒ��򣨻���ϵ������ϵ���ӵ��ú��������� ����
    uint8_t move_left;                                      // ����ƽ��
    uint8_t move_right;                                     // ����ƽ��
    uint8_t move_forward;                                   // ��ǰƽ��
    uint8_t move_backward;                                  // ���ƽ��

    //���� �̶����� ����
    float   tilt_degree;                                    // ��б�Ƕȣ��ȣ�
    float   move_g;                                         // ƽ��ǿ�ȣ�g��
} tilt_motion_t;

//==================== һ�׵�ͨ�˲�����LPF1�� ===========================
typedef struct
{
    float alpha;                                            // �˲�ϵ�� ����(0,1]��ԽСԽƽ��
    float y;                                                // ��һ���˲���� y(k-1)
    unsigned inited:1;                                         // �Ƿ��ʼ����־
} lpf1_t;

void LPF1_Init(lpf1_t *f, float alpha);                     // ��ʼ�� LPF1
float LPF1_Apply(lpf1_t *f, float x);                       // ���� x����� y = y + ��*(x-y)
void  AHRS_LPF_SetAlpha(float alpha_body, float alpha_world);// ���û���ϵ/����ϵ LPF ��

//==================== ��̬�˲����Ľӿ� ================================
void AHRS_Reset(void);                                      // ��λ�˲�������Ԫ��/�����
void AHRS_Update(float gx, float gy, float gz,              // ���ݣ�rad/s��
                 float ax, float ay, float az,              // �Ӽƣ�g��
                 float dt, ahrs_euler_t *out);              // �������� & ���ŷ����

//==================== �м���㣺�����۳� / ����任 =====================
void AHRS_RemoveGravity(float ax_g, float ay_g, float az_g, // ԭʼ���ٶȣ�g��
                        float *lin_x_g, float *lin_y_g,     // ���������ϵ���Լ��ٶȣ�g��
                        float *lin_z_g);
void AHRS_BodyToWorld(float bx, float by, float bz,         // ���룺����ϵ����
                      float *wx, float *wy, float *wz);     // ���������ϵ������Z �����ϣ�

//==================== ��б/ƽ�� ��⣨Body & World ���棩 ==============
void Detect_TiltMotion_Body(float roll_deg, float pitch_deg, // ���ڻ���ϵ���Լ��ٶ�
                            float ax_g, float ay_g, float az_g,
                            tilt_motion_t *out);
void Detect_TiltMotion_World(float roll_deg, float pitch_deg, // ��������ϵ���Լ��ٶ�
                             float ax_g, float ay_g, float az_g,
                             tilt_motion_t *out);



#endif
