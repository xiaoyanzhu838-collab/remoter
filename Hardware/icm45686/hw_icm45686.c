#include "hw_icm45686.h"

#include "main.h"  

#include <string.h>                                       
#include <math.h>     


#define M_PI 3.14159265358979323846


extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim2;

volatile uint8_t g_imu_int1_flag = 0;                     // INT1 ��־
volatile uint8_t g_imu_int2_flag = 0;                     // INT2 ��־





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM_INT1_Pin)
			g_imu_int1_flag = 1;    // ��� INT1
    else if (GPIO_Pin == ICM_INT2_Pin)
			g_imu_int2_flag = 1;		// ��� INT2
}


static inline float gyro_lsb_to_rads(int16_t raw, float fs_dps) // ԭʼ -> rad/s
{ 
	return ((float)raw) * (fs_dps / 32768.0f) * (float)M_PI / 180.0f;
}

static inline float accel_lsb_to_g(int16_t raw, float fs_g)     // ԭʼ -> g
{ 
	return ((float)raw) * (fs_g / 32768.0f);
}



static void imu_cs_low(void)
{
	HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_RESET);
}

static void imu_cs_high(void)
{
	HAL_GPIO_WritePin(SPI4_NSS_GPIO_Port, SPI4_NSS_Pin, GPIO_PIN_SET);
}


int imu_spi_write(uint8_t reg, const uint8_t *buf, uint32_t len)
{
	uint8_t hdr = reg & 0x7F;
	
	imu_cs_low();
	
  if (HAL_SPI_Transmit(&hspi4, &hdr, 1, 1000) != HAL_OK)
	{ 
		imu_cs_high();
		
//		printf("spi4 err\r\n");
		
		return -1; 
	}
  if (len && HAL_SPI_Transmit(&hspi4, (uint8_t*)buf, len, 1000) != HAL_OK)
	{ 
		imu_cs_high(); 
		
//		printf("spi4 err\r\n");
		
		return -1; 
	}
	
  imu_cs_high();                                  		// �ͷ� CS
	
  return 0;                                            // �ɹ�
	
}

int imu_spi_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t hdr = reg | 0x80;                             // ��������bit7=1
	
    imu_cs_low();                                     		
	
    if (HAL_SPI_Transmit(&hspi4, &hdr, 1, 1000) != HAL_OK)
		{ 
			imu_cs_high(); 
			
//		printf("spi4 err\r\n");
			
			return -1; 
		}
    if (len && HAL_SPI_Receive(&hspi4, buf, len, 1000) != HAL_OK)
		{ 
			imu_cs_high(); 
			
//		printf("spi4 err\r\n");
			
			return -1; 
		}
		
    imu_cs_high();                                  // �ͷ� CS
    return 0;                                       // �ɹ�
}



void imu_sleep_us(uint32_t us)                        		// ΢����ʱ�������� DWT��
{
    uint32_t start = (uint32_t)htim2.Instance->CNT;   		// ������ʼ����
	
    while ( (uint32_t)(htim2.Instance->CNT - start) < us);
}


int icm_set_int(icm_app_t *app)
{
    int rc = 0;

    // --- INT1 ���ã����У� ---
    int1_config2_t i1 = {0};
    rc |= inv_imu_read_reg(&app->tr, INT1_CONFIG2, 1, (uint8_t*)&i1);
    i1.int1_polarity = 0;  // �͵�ƽ��Ч
    i1.int1_mode     = 1;  // ��ƽ����
    i1.int1_drive    = 0;  // ����
    rc |= inv_imu_write_reg(&app->tr, INT1_CONFIG2, 1, (uint8_t*)&i1);

    // --- INT2 ��Ϊ���ã����У� ---
    int2_config2_t i2 = {0};
    rc |= inv_imu_read_reg(&app->tr, INT2_CONFIG2, 1, (uint8_t*)&i2);
    i2.int2_polarity = 0;  // �͵�ƽ��Ч
    i2.int2_mode     = 1;  // ��ƽ����
    i2.int2_drive    = 0;  // ����
    rc |= inv_imu_write_reg(&app->tr, INT2_CONFIG2, 1, (uint8_t*)&i2);

    // --- 1) ���� APEX ���ܣ�Tilt, R2W, SMD�� ---
    uint8_t apex_en = 0;
    rc |= inv_imu_read_reg(&app->tr, 0x29 /* EDMP_APEX_EN0 */, 1, &apex_en);
    apex_en |= (1 << 3) | (1 << 6) | (1 << 7);  // ���� Tilt, R2W, SMD
    rc |= inv_imu_write_reg(&app->tr, 0x29, 1, &apex_en);

    // --- 2) ��� APEX �¼����Σ�ʹ������� ---
    uint8_t cfg0 = 0;
    rc |= inv_imu_read_reg(&app->tr, 0x39 /* INT_APEX_CONFIG0 */, 1, &cfg0);
    cfg0 &= ~(1 << 3);  // ��� Tilt ����λ
    rc |= inv_imu_write_reg(&app->tr, 0x39, 1, &cfg0);

    uint8_t cfg1 = 0;
    rc |= inv_imu_read_reg(&app->tr, 0x3A /* INT_APEX_CONFIG1 */, 1, &cfg1);
    cfg1 &= ~((1 << 1) | (1 << 0));  // ��� SMD / R2W ����λ
    rc |= inv_imu_write_reg(&app->tr, 0x3A, 1, &cfg1);

    // --- 3) ӳ�� APEX �¼��� INT2 ���� ---
    int2_config1_t i21 = {0};
    rc |= inv_imu_read_reg(&app->tr, INT2_CONFIG1, 1, (uint8_t*)&i21);
    i21.int2_status_en_apex_event = 1;  // ���� APEX ���� INT2
    rc |= inv_imu_write_reg(&app->tr, INT2_CONFIG1, 1, (uint8_t*)&i21);

    return rc;
}











/* ==================== �ؼ���FIFO �ٷ�˳������ ==================== */
/*
�ٷ����飨��������
1) �ر� FIFO��Bypass���� FIFO_IF
2) �� INT1 FIFO_THS �жϣ���ѡ��
3) ����ѡ������ʱ������룺FIFO_CONFIG4.bit1=1������ FSYNC������ TMST ������ã�
4) ��ˮλ�� FIFO_WM_TH����д���ֽ� REG 0x1E����д���ֽ� REG 0x1F��
5) FIFO_CONFIG2.bit3=1 �á�>= ˮλ�ߡ�����������������==��
6) FIFO_CONFIG3 ѡ��д�� FIFO ��Դ���Ӽ�/����/�߷ֱ���/�ⲿ�����������������ȱ� FIFO_IF
7) FIFO_CONFIG0 ��Ϊ Stream ģʽ
8) ���һ����FIFO_CONFIG3.bit0 = 1 �� FIFO_IF���ǳ��ؼ�����
�ο���User Guide 3.5 �� + ��FIFO_IF to be set at last�����Ĵ���/λ˵���� Datasheet 17.28~17.33��*/
static int icm_fifo_configure_stream(icm_app_t *app, uint16_t wm_frames, int enable_ts)
{
    int rc = 0;
    uint8_t v;

    /* 1) �ر� FIFO��Bypass�����ȹ� FIFO_IF_EN */
    v = 0;
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);                       /* ...IF=0��Դȫ�� */
    v = FIFO_MODE_BYPASS | 0x07;                                                      /* [5:0] FIFO_DEPTH���Ƽ� 0x07=2KB������ 8KB �� 0x1F �ҽ��� APEX */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG0, 1, &v);                       /* ���� Bypass */

    /* 2) �� INT1 FIFO_THS����ѡ��ʾ���򿪣� */
    v = 0;
    rc |= inv_imu_read_reg(&app->tr, REG_INT1_CONFIG0, 1, &v);
    v = (uint8_t)((v | INT1_EN_FIFO_THS) & ~INT1_EN_DRDY);                            /* �� FIFO_THS �жϣ��� DRDY��������·ͬʱ���� */
    rc |= inv_imu_write_reg(&app->tr, REG_INT1_CONFIG0, 1, &v);                       /* ��ַ 0x16��bit1=1 ���� FIFO_THS �ж� */

    /* 3) ����ѡ��ʱ������뵽 FIFO ֡ */
    v = 0;
    if (enable_ts) v |= FIFO_TMST_FSYNC_EN;                                           /* REG_FIFO_CONFIG4.bit1 = 1 */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG4, 1, &v);

    /* ע����Ҫʹ�� FSYNC/����ϸ��ʱ������������� TMST/FSYNC �ȼĴ�����
       Datasheet/UG �в��� TMST λ�� IPREG_TOP1����ӼĴ���������Ҫ�� IREG �������̣��˴������������á� */

    /* 4) ����ˮλ�ߣ���λ = ֡����COUNT ���ǡ�֡�������� ��д�� 8 λ����д�� 8 λ */
    uint8_t wm_l = (uint8_t)(wm_frames & 0xFF);
    uint8_t wm_h = (uint8_t)((wm_frames >> 8) & 0xFF);
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG1_0, 1, &wm_l);                  /* �ȵ��ֽ� 0x1E */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG1_1, 1, &wm_h);                  /* �ٸ��ֽ� 0x1F */

    /* 5) дˮλ�ȽϷ�ʽ��>= ��ֵ���� */
    v = 0;
    rc |= inv_imu_read_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);
    v |= FIFO_WR_WM_GT_TH;                                                            /* 0x20 bit3 = 1 */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);

    /* 6) ѡ���� FIFO д���Դ��������Accel+Gyro���߷ֱ��ʹر�=16B ֡�� */
    v = 0;
    v |= FIFO_EN_ACCEL;
    v |= FIFO_EN_GYRO;
    /* �����Ҫ 20-bit �߷ֱ��ʣ���������λ FIFO_HIRES_EN��0x21 bit3��������Ӧ���� 20B ֡ */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);                       /* ע�⣺��ʱ��δ�� FIFO_IF */

    /* 7) �� FIFO ����ģʽ��Stream��֡����д�룩 */
    v = FIFO_MODE_STREAM | 0x07;                                                      /* ���� FIFO_DEPTH=2KB������ 8KB����ȷ���ر�ȫ�� APEX */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG0, 1, &v);

    /* 8) ���һ������ FIFO_IF_EN���ǳ��ؼ��� */
    v = 0;
    rc |= inv_imu_read_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);
    v |= FIFO_IF_EN;                                                                  /* 0x21 bit0 = 1 */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);

    return rc;
}

/* �� FIFO����λ FIFO_FLUSH��0x20 bit7��һ˲�伴�� */
static int icm_fifo_flush(icm_app_t *app)
{
    uint8_t v = 0;
    int rc = inv_imu_read_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);
    if (rc) return rc;
    v |= BIT(7);                                /* FIFO_FLUSH = 1 */
    rc = inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);
    /* ���ٶ������㣬��ȴ�Ӳ���Զ����� */
    return rc;
}











/* ==================== ��ʼ����������ԭʼ���� + �� FIFO ���ã� ==================== */
int icm_app_init(icm_app_t *app)
{
    memset(app, 0, sizeof(*app));
    app->tr.read_reg   = imu_spi_read;
    app->tr.write_reg  = imu_spi_write;
    app->tr.serif_type = UI_SPI4;
    app->tr.sleep_us   = imu_sleep_us;
    app->dev.transport = app->tr;

    /* �� WHO_AM_I = 0xE9 */
    uint8_t who = 0;
    inv_imu_read_reg(&app->tr, REG_WHO_AM_I, 1, &who);
    if (who != 0xE9)
			return -1;

    if (inv_imu_adv_device_reset(&app->dev) != 0)
			return -2;

    /* ���Ӽ�/���ݣ�LN�� */
    uint8_t pwr = 0;
    pwr |= (3u << 2);  /* GYRO_MODE=3(LN) */
    pwr |= (3u << 0);  /* ACCEL_MODE=3(LN) */
    inv_imu_write_reg(&app->tr, REG_PWR_MGMT0, 1, &pwr);

    /* ���� ODR/���̣���4g @ 400Hz����2000dps @ 400Hz */
    uint8_t ac0 = 0;
    ac0 |= (0x3u << 0);          /* ACCEL_ODR = 0b0111 -> 400Hz */
    ac0 |= (0x3u << 4);          /* ACCEL_UI_FS_SEL = 0b011 -> ��4g */
    inv_imu_write_reg(&app->tr, REG_ACCEL_CONFIG0, 1, &ac0);

    uint8_t gc0 = 0;
    gc0 |= (0x7u << 4);          /* GYRO_UI_FS_SEL = 0b0001?�����ֲ�ӳ�䣩������ 0x1=��2000dps��������ľ�ֵ 0x7=��31.25dps����Ϊ 0x1������ */
    gc0 |= (0x7u << 0);          /* GYRO_ODR = 0b0111 -> 400Hz */
    /* ˵����Datasheet �� GYRO_UI_FS_SEL ӳ��Ϊ 0x1=��2000dps��0x0=��4000dps��0x3=��500dps �ȡ��밴�������:contentReference[oaicite:3]{index=3} */
    inv_imu_write_reg(&app->tr, REG_GYRO_CONFIG0, 1, &gc0);

    app->odr_hz = 400.0f;

    /* ���� INT ����/ģʽ����ԭ�к����� */
    if (icm_set_int(app) != 0)
			return -3;

    /* ===== �£����ٷ�˳������ FIFO��ˮλ��=16 ֡������ʱ������룩 ===== */
    if (icm_fifo_configure_stream(app, /*wm_frames=*/16, /*enable_ts=*/1) != 0)
			return -4;

    /* ��λ������һ�� FIFO����������� */
    icm_fifo_flush(app);

    return 0;
}

int  icm_read_accel_gyro(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad)
{
    uint8_t buf[12];                                       // ��������
    if (inv_imu_read_reg(&app->tr, ACCEL_DATA_X1_UI, 12, buf) != 0)
			return -1; // 0x00..0x0B
		
		
    int16_t ax=(int16_t)((buf[0 ]<<8)|buf[1 ]);
    int16_t ay=(int16_t)((buf[2 ]<<8)|buf[3 ]);
    int16_t az=(int16_t)((buf[4 ]<<8)|buf[5 ]);
    int16_t gx=(int16_t)((buf[6 ]<<8)|buf[7 ]);
    int16_t gy=(int16_t)((buf[8 ]<<8)|buf[9 ]);
    int16_t gz=(int16_t)((buf[10]<<8)|buf[11]);
		
		
    *ax_g   = accel_lsb_to_g(ax, 4.0f);                   // g
    *ay_g   = accel_lsb_to_g(ay, 4.0f);
    *az_g   = accel_lsb_to_g(az, 4.0f);
    *gx_rad = gyro_lsb_to_rads(gx, 2000.0f);              // rad/s
    *gy_rad = gyro_lsb_to_rads(gy, 2000.0f);
    *gz_rad = gyro_lsb_to_rads(gz, 2000.0f);
		
		
    return 0;                                              // �ɹ�
}




/* ==================== FIFO ��ȡ���������̣�����ͷ����֡��ȡ�� ==================== */
/* ���أ�0=OK��*has_frame=1 ���������Чһ֡��=0 ����ǰ��֡ */
int icm_read_fifo_serv(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad, uint8_t *has_frame)
{
    *has_frame = 0;

    /* ��ȡ FIFO_COUNT����λ=֡���� */
    uint8_t h = 0, l = 0;
    inv_imu_read_reg(&app->tr, REG_FIFO_COUNT_0, 1, &h);  /* ���ֽ���ǰ */
    inv_imu_read_reg(&app->tr, REG_FIFO_COUNT_1, 1, &l);
    uint16_t frames = ((uint16_t)h << 8) | l;
    if (frames == 0) return 0;                             /* ��֡ */

    /* 1) ��֡ͷ��Header��������֡�� */
    uint8_t hdr = 0;
    inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &hdr);    /* ÿ��һ�� FIFO_DATA �ͳ��� 1 �ֽ� */

    /* ����ͷ�ֽڣ�Datasheet 6.2 FIFO HEADER�� */
    uint8_t ext_header  = (hdr >> 7) & 0x01;  /* �ⲿ������/ѹ��ʱ����=1��ͷ���ֽ� */
    uint8_t accel_en    = (hdr >> 6) & 0x01;
    uint8_t gyro_en     = (hdr >> 5) & 0x01;
    uint8_t hires_en    = (hdr >> 4) & 0x01;  /* 20-bit �߷ֱ��� */
    uint8_t tmst_field  = (hdr >> 3) & 0x01;  /* �Ƿ����ʱ����ֶ� */
    /* ����λ�˴������� */

    /* ���ͷ��չ��ext_header=1�����ٶ��� 2 ��ͷ�ֽڵ����������ⲿ��������Ϣ����ֱ�Ӷ��� */
    if (ext_header) {
        uint8_t hdr2;
        inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &hdr2);
        /* ��Ҫ֧�� ES0/ES1 ��ѹ��ʱ���ٸ��� hdr2 �� ES0/ES1 λ����֡�� */
    }

    /* 2) ���㡰ʣ�ࡱӦ��ȡ���ֽ����������Ѷ��� 1B �� 2B ͷ�� */
    uint16_t payload = 0;
    if (accel_en && gyro_en && !hires_en) {
        /* 16 �ֽ�֡��Accel(6) + Gyro(6) + Temp(1) + Ts(2) = 15������ 1B ͷ = 16B */
        payload = 15 - (ext_header ? 1 : 0);  /* ������չͷ���ܳ����� 16B������ֻ���ʣ�� */
    } else if (accel_en ^ gyro_en) {
        /* 8 �ֽ�֡��Accel-only �� Gyro-only + Temp(1)��ͨ���� Ts���� hdr �� tmst_field�� */
        /* ��׼ 8B �ṹ��ͷ(1) + ����(6) + Temp(1) => ʣ�� 7B���� ext ͷ������-1�� */
        payload = 7 - (ext_header ? 1 : 0);
        /* �� tmst_field=1����������ϣ��������ⲿ������/�߷ֱ�����ϣ����ܳ���仯���˴������� 8B ���� */
    } else if (accel_en && gyro_en && hires_en) {
        /* 20 �ֽ�֡���߷ֱ��� 20-bit���� 6 �� LSB �ֽ� + Temp(2) + Ts(2) */
        /* ͷ(1) + 19B => ʣ�� 19B���� ext ͷ����-1�� */
        payload = 19 - (ext_header ? 1 : 0);
    } else {
        /* �������������ⲿ������֡ 16/20/32B�����˴�ֱ�Ӷ�����֡�Ա����λ */
        /* Ϊ�˲��ƻ�����֡���� FIFO �аѱ�֡�����ֽڶ��� */
        /* �򵥲��ԣ��� 16B �ܳ������� */
        uint8_t trash;
        for (int i = 0; i < 15 - (ext_header ? 1 : 0); ++i) inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &trash);
        return 0;
    }

    uint8_t frm[32] = {0};
    for (uint16_t i = 0; i < payload; ++i) inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &frm[i]);

    /* 3) ���� 16B ����֡������֧����ĵ�ǰ���ã� */
    if (accel_en && gyro_en && !hires_en) {
        /* ��ϳ����� 16B ���壺�����Ѷ� head(1) + payload(15)��frm[]�ﱣ����Ǻ� 15 �ֽ� */
        /* �ֶ�˳�򣨴�ˣ���AxH AxL AyH AyL AzH AzL GxH GxL GyH GyL GzH GzL Temp  TsH TsL */
        int16_t ax = (int16_t)((frm[0] << 8)  | frm[1]);
        int16_t ay = (int16_t)((frm[2] << 8)  | frm[3]);
        int16_t az = (int16_t)((frm[4] << 8)  | frm[5]);
        int16_t gx = (int16_t)((frm[6] << 8)  | frm[7]);
        int16_t gy = (int16_t)((frm[8] << 8)  | frm[9]);
        int16_t gz = (int16_t)((frm[10] << 8) | frm[11]);
        /* uint8_t temp = frm[12];  uint16_t ts = ((uint16_t)frm[13]<<8)|frm[14]; // ����ʹ�� */

        *ax_g   = accel_lsb_to_g(ax, 4.0f);
        *ay_g   = accel_lsb_to_g(ay, 4.0f);
        *az_g   = accel_lsb_to_g(az, 4.0f);
        *gx_rad = gyro_lsb_to_rads(gx, 2000.0f);
        *gy_rad = gyro_lsb_to_rads(gy, 2000.0f);
        *gz_rad = gyro_lsb_to_rads(gz, 2000.0f);

        *has_frame = 1;
        return 0;
    }

    /* 4) ����֡��8B/20B����ʾ����ֻ����������ĵ�ͨ�����˴��ԣ��ɰ���Ҫ��ȫ */
    *has_frame = 0;
    return 0;
}

int icm_read_apex(icm_app_t *app, icm_apex_event_t *evt)// ��ȡ APEX �¼�
{
    uint8_t s0=0,s1=0; int rc=0;                           // ״̬�Ĵ���
	
    memset(evt,0,sizeof(*evt));                            // ����
	
    rc |= inv_imu_read_reg(&app->tr, INT_APEX_STATUS0, 1, &s0); // 0x56
    rc |= inv_imu_read_reg(&app->tr, INT_APEX_STATUS1, 1, &s1); // 0x57
	
    if (rc)
			return rc;                                     // ��ȡʧ��
		
		
    if (s0 & 0x01) evt->tilt = 1;                          // bit0: tilt
    if (s0 & 0x02) evt->smd  = 1;                          // bit1: smd
    if (s0 & 0x80) evt->r2w  = 1;                          // bit7: r2w
		
    return 0;                                              // �ɹ�
		
}



























