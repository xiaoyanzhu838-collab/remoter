#ifndef __HW_ICM45686_H__
#define __HW_ICM45686_H__


                             
#include <stdint.h>                                         
//#include <stdbool.h>      





#include "inv_imu.h" 
#include "inv_imu_defs.h" 
#include "inv_imu_driver.h"                             // InvenSense �豸�ӿ�
#include "inv_imu_regmap_le.h"                          // �Ĵ���ӳ��
#include "inv_imu_driver_advanced.h" 
#include "inv_imu_edmp.h"
#include "inv_imu_edmp_defs.h"
#include "inv_imu_edmp_memmap.h"
#include "inv_imu_selftest.h"
#include "inv_imu_version.h"
#include "inv_imu_transport.h"                          // �����

//--------------------------------------------------------------------------------------------------------------
typedef struct                                              // Ӧ��������
{
    inv_imu_device_t     dev;                               // �豸����
    inv_imu_transport_t  tr;                                // �������
    float                odr_hz;                            // ʵ�� ODR��Hz��
} icm_app_t;


int  icm_app_init(icm_app_t *app);                          // ��ʼ����WHOAMI/��λ/ODR/FIFO/INT��
int  icm_set_int(icm_app_t *app);            								// ���� INT1/INT2 �͵�ƽ��Ч
int  icm_read_accel_gyro(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad);// ֱ�ӼĴ�����ȡһ֡
int  icm_read_fifo_serv(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad, uint8_t *has_frame);// FIFO ȡһ֡

//--------------------------------------------------------------------------------------------------------------
//���� APEX �¼� ����
typedef struct
{
    uint8_t tilt;                                          // ��б
    uint8_t smd;                                           // �����˶�
    uint8_t r2w;                                           // ̧����
} icm_apex_event_t;

int icm_read_apex(icm_app_t *app, icm_apex_event_t *evt);// ��ȡ APEX ״̬											 
													 
//--------------------------------------------------------------------------------------------------------------
int  imu_spi_write(uint8_t reg, const uint8_t *buf, uint32_t len);

int  imu_spi_read(uint8_t reg, uint8_t *buf, uint32_t len);

//--------------------------------------------------------------------------------------------------------------






extern volatile uint8_t g_imu_int1_flag;                          // INT1 ��־
extern volatile uint8_t g_imu_int2_flag;                          // INT2 ��־



/* ---------- ICM-45686 �û����� 0���ؼ��Ĵ�����ַ���ٷ��ֲᣩ ---------- */
/* ���ݼĴ��� */
#define REG_ACCEL_DATA_X1_UI   0x00
#define REG_GYRO_DATA_X1_UI    0x06
#define REG_TEMP_DATA1_UI      0x0C
#define REG_TEMP_DATA0_UI      0x0D
#define REG_TMST_FSYNCH        0x0E
#define REG_TMST_FSYNCL        0x0F

/* ��Դ/���� */
#define REG_PWR_MGMT0          0x10
#define REG_INT1_CONFIG0       0x16  /* bit1: FIFO_THS �ж�ʹ�ܣ�bit2: DRDY �� */
#define REG_WHO_AM_I           0x72  /* WHO_AM_I ���� 0xE9 */

/* FIFO ���������ݶ˿ڣ�ע�⣺COUNT = ֡���������ֽ����� */
#define REG_FIFO_COUNT_0       0x12  /* �� 8 λ����λ��ǰ */
#define REG_FIFO_COUNT_1       0x13  /* �� 8 λ */
#define REG_FIFO_DATA          0x14  /* ���˼Ĵ����Զ�����һ�ֽ� */

/* ������ ODR */
#define REG_ACCEL_CONFIG0      0x1B  /* ACCEL_UI_FS_SEL[6:4], ACCEL_ODR[3:0] */
#define REG_GYRO_CONFIG0       0x1C  /* GYRO_UI_FS_SEL[7:4],  GYRO_ODR[3:0]  */

/* FIFO �����飨�ؼ���FIFO_IF_EN ������ã� */
#define REG_FIFO_CONFIG0       0x1D  /* [7:6] FIFO_MODE: 0=Bypass,1=Stream,2=StopOnFull */
#define REG_FIFO_CONFIG1_0     0x1E  /* FIFO_WM_TH[7:0] ����д���ֽ���д���ֽڣ� */
#define REG_FIFO_CONFIG1_1     0x1F  /* FIFO_WM_TH[15:8] */
#define REG_FIFO_CONFIG2       0x20  /* bit3: FIFO_WR_WM_GT_TH */
#define REG_FIFO_CONFIG3       0x21  /* bit2:GYRO_EN bit1:ACCEL_EN bit0:FIFO_IF_EN�������1�� */
#define REG_FIFO_CONFIG4       0x22  /* bit1: FIFO_TMST_FSYNC_EN����֡�в���ʱ���/FSYNC�� */

/* ---------- λ����/�ֶ� ---------- */
#define BIT(n)                 (1u << (n))

/* REG_FIFO_CONFIG0 [7:6] FIFO_MODE */
#define FIFO_MODE_BYPASS       (0u << 6)
#define FIFO_MODE_STREAM       (1u << 6)
#define FIFO_MODE_STOP_ON_FULL (2u << 6)

/* REG_INT1_CONFIG0 λ */
#define INT1_EN_FIFO_THS       BIT(1)
#define INT1_EN_DRDY           BIT(2)  /* ����Ҫ��/�ر� */

/* REG_FIFO_CONFIG2 λ */
#define FIFO_WR_WM_GT_TH       BIT(3)

/* REG_FIFO_CONFIG3 λ */
#define FIFO_EN_GYRO           BIT(2)
#define FIFO_EN_ACCEL          BIT(1)
#define FIFO_IF_EN             BIT(0)

/* REG_FIFO_CONFIG4 λ */
#define FIFO_TMST_FSYNC_EN     BIT(1)





#endif
