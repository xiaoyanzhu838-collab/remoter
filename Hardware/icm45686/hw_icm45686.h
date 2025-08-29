#ifndef __HW_ICM45686_H__
#define __HW_ICM45686_H__


                             
#include <stdint.h>                                         
//#include <stdbool.h>      





#include "inv_imu.h" 
#include "inv_imu_defs.h" 
#include "inv_imu_driver.h"                             // InvenSense 设备接口
#include "inv_imu_regmap_le.h"                          // 寄存器映射
#include "inv_imu_driver_advanced.h" 
#include "inv_imu_edmp.h"
#include "inv_imu_edmp_defs.h"
#include "inv_imu_edmp_memmap.h"
#include "inv_imu_selftest.h"
#include "inv_imu_version.h"
#include "inv_imu_transport.h"                          // 传输层

//--------------------------------------------------------------------------------------------------------------
typedef struct                                              // 应用上下文
{
    inv_imu_device_t     dev;                               // 设备对象
    inv_imu_transport_t  tr;                                // 传输对象
    float                odr_hz;                            // 实际 ODR（Hz）
} icm_app_t;


int  icm_app_init(icm_app_t *app);                          // 初始化（WHOAMI/复位/ODR/FIFO/INT）
int  icm_set_int(icm_app_t *app);            								// 设置 INT1/INT2 低电平有效
int  icm_read_accel_gyro(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad);// 直接寄存器读取一帧
int  icm_read_fifo_serv(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad, uint8_t *has_frame);// FIFO 取一帧

//--------------------------------------------------------------------------------------------------------------
//—— APEX 事件 ——
typedef struct
{
    uint8_t tilt;                                          // 倾斜
    uint8_t smd;                                           // 显著运动
    uint8_t r2w;                                           // 抬腕唤醒
} icm_apex_event_t;

int icm_read_apex(icm_app_t *app, icm_apex_event_t *evt);// 读取 APEX 状态											 
													 
//--------------------------------------------------------------------------------------------------------------
int  imu_spi_write(uint8_t reg, const uint8_t *buf, uint32_t len);

int  imu_spi_read(uint8_t reg, uint8_t *buf, uint32_t len);

//--------------------------------------------------------------------------------------------------------------






extern volatile uint8_t g_imu_int1_flag;                          // INT1 标志
extern volatile uint8_t g_imu_int2_flag;                          // INT2 标志



/* ---------- ICM-45686 用户银行 0：关键寄存器地址（官方手册） ---------- */
/* 数据寄存器 */
#define REG_ACCEL_DATA_X1_UI   0x00
#define REG_GYRO_DATA_X1_UI    0x06
#define REG_TEMP_DATA1_UI      0x0C
#define REG_TEMP_DATA0_UI      0x0D
#define REG_TMST_FSYNCH        0x0E
#define REG_TMST_FSYNCL        0x0F

/* 电源/配置 */
#define REG_PWR_MGMT0          0x10
#define REG_INT1_CONFIG0       0x16  /* bit1: FIFO_THS 中断使能；bit2: DRDY 等 */
#define REG_WHO_AM_I           0x72  /* WHO_AM_I 读到 0xE9 */

/* FIFO 计数与数据端口（注意：COUNT = 帧数，不是字节数） */
#define REG_FIFO_COUNT_0       0x12  /* 高 8 位，高位在前 */
#define REG_FIFO_COUNT_1       0x13  /* 低 8 位 */
#define REG_FIFO_DATA          0x14  /* 读此寄存器自动出队一字节 */

/* 量程与 ODR */
#define REG_ACCEL_CONFIG0      0x1B  /* ACCEL_UI_FS_SEL[6:4], ACCEL_ODR[3:0] */
#define REG_GYRO_CONFIG0       0x1C  /* GYRO_UI_FS_SEL[7:4],  GYRO_ODR[3:0]  */

/* FIFO 配置组（关键：FIFO_IF_EN 最后设置） */
#define REG_FIFO_CONFIG0       0x1D  /* [7:6] FIFO_MODE: 0=Bypass,1=Stream,2=StopOnFull */
#define REG_FIFO_CONFIG1_0     0x1E  /* FIFO_WM_TH[7:0] （先写低字节再写高字节） */
#define REG_FIFO_CONFIG1_1     0x1F  /* FIFO_WM_TH[15:8] */
#define REG_FIFO_CONFIG2       0x20  /* bit3: FIFO_WR_WM_GT_TH */
#define REG_FIFO_CONFIG3       0x21  /* bit2:GYRO_EN bit1:ACCEL_EN bit0:FIFO_IF_EN（最后置1） */
#define REG_FIFO_CONFIG4       0x22  /* bit1: FIFO_TMST_FSYNC_EN（向帧中插入时间戳/FSYNC） */

/* ---------- 位掩码/字段 ---------- */
#define BIT(n)                 (1u << (n))

/* REG_FIFO_CONFIG0 [7:6] FIFO_MODE */
#define FIFO_MODE_BYPASS       (0u << 6)
#define FIFO_MODE_STREAM       (1u << 6)
#define FIFO_MODE_STOP_ON_FULL (2u << 6)

/* REG_INT1_CONFIG0 位 */
#define INT1_EN_FIFO_THS       BIT(1)
#define INT1_EN_DRDY           BIT(2)  /* 视需要打开/关闭 */

/* REG_FIFO_CONFIG2 位 */
#define FIFO_WR_WM_GT_TH       BIT(3)

/* REG_FIFO_CONFIG3 位 */
#define FIFO_EN_GYRO           BIT(2)
#define FIFO_EN_ACCEL          BIT(1)
#define FIFO_IF_EN             BIT(0)

/* REG_FIFO_CONFIG4 位 */
#define FIFO_TMST_FSYNC_EN     BIT(1)





#endif
