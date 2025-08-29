#ifndef __AHRS_H__
#define __AHRS_H__

#include <stdint.h>                                         // 标准整型
#include <stdbool.h>                                        // 布尔类型

//==================== 机体系坐标约定（按板子朝向改） ====================
// 约定：机体系 Body +X 指“向前”，+Y 指“向左”，+Z 指“向上”；若实物相反改符号。
//#define FORWARD_DIR_SIGN  (+1.0f)                           // 世界/机体“向前”正方向相对 Body +X 的符号
//#define LEFT_DIR_SIGN     (+1.0f)                           // 世界/机体“向左”正方向相对 Body +Y 的符号

//==================== 欧拉角输出（与之前保持一致） ======================
typedef struct
{
    float roll_deg;                                         // 横滚角（度）
    float pitch_deg;                                        // 俯仰角（度）
    float yaw_deg;                                          // 航向角（度）
} ahrs_euler_t;

//==================== 倾斜/平移 事件输出结构体 ==========================
typedef struct
{
    //—— 倾斜方向 ——
    uint8_t tilt_left;                                      // 左倾斜
    uint8_t tilt_right;                                     // 右倾斜
    uint8_t tilt_forward;                                   // 前倾斜
    uint8_t tilt_backward;                                  // 后倾斜

    //—— 平移方向（机体系或世界系，视调用函数而定） ——
    uint8_t move_left;                                      // 向左平移
    uint8_t move_right;                                     // 向右平移
    uint8_t move_forward;                                   // 向前平移
    uint8_t move_backward;                                  // 向后平移

    //—— 程度量化 ——
    float   tilt_degree;                                    // 倾斜角度（度）
    float   move_g;                                         // 平移强度（g）
} tilt_motion_t;

//==================== 一阶低通滤波器（LPF1） ===========================
typedef struct
{
    float alpha;                                            // 滤波系数 α，(0,1]，越小越平滑
    float y;                                                // 上一次滤波输出 y(k-1)
    unsigned inited:1;                                         // 是否初始化标志
} lpf1_t;

void LPF1_Init(lpf1_t *f, float alpha);                     // 初始化 LPF1
float LPF1_Apply(lpf1_t *f, float x);                       // 输入 x，输出 y = y + α*(x-y)
void  AHRS_LPF_SetAlpha(float alpha_body, float alpha_world);// 设置机体系/世界系 LPF α

//==================== 姿态滤波核心接口 ================================
void AHRS_Reset(void);                                      // 复位滤波器（四元数/积分项）
void AHRS_Update(float gx, float gy, float gz,              // 陀螺（rad/s）
                 float ax, float ay, float az,              // 加计（g）
                 float dt, ahrs_euler_t *out);              // 采样周期 & 输出欧拉角

//==================== 中间计算：重力扣除 / 坐标变换 =====================
void AHRS_RemoveGravity(float ax_g, float ay_g, float az_g, // 原始加速度（g）
                        float *lin_x_g, float *lin_y_g,     // 输出：机体系线性加速度（g）
                        float *lin_z_g);
void AHRS_BodyToWorld(float bx, float by, float bz,         // 输入：机体系向量
                      float *wx, float *wy, float *wz);     // 输出：世界系向量（Z 轴向上）

//==================== 倾斜/平移 检测（Body & World 两版） ==============
void Detect_TiltMotion_Body(float roll_deg, float pitch_deg, // 基于机体系线性加速度
                            float ax_g, float ay_g, float az_g,
                            tilt_motion_t *out);
void Detect_TiltMotion_World(float roll_deg, float pitch_deg, // 基于世界系线性加速度
                             float ax_g, float ay_g, float az_g,
                             tilt_motion_t *out);



#endif
