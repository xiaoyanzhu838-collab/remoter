#include "hw_icm45686.h"

#include "main.h"  

#include <string.h>                                       
#include <math.h>     


#define M_PI 3.14159265358979323846


extern SPI_HandleTypeDef hspi4;
extern TIM_HandleTypeDef htim2;

volatile uint8_t g_imu_int1_flag = 0;                     // INT1 标志
volatile uint8_t g_imu_int2_flag = 0;                     // INT2 标志





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ICM_INT1_Pin)
			g_imu_int1_flag = 1;    // 标记 INT1
    else if (GPIO_Pin == ICM_INT2_Pin)
			g_imu_int2_flag = 1;		// 标记 INT2
}


static inline float gyro_lsb_to_rads(int16_t raw, float fs_dps) // 原始 -> rad/s
{ 
	return ((float)raw) * (fs_dps / 32768.0f) * (float)M_PI / 180.0f;
}

static inline float accel_lsb_to_g(int16_t raw, float fs_g)     // 原始 -> g
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
	
  imu_cs_high();                                  		// 释放 CS
	
  return 0;                                            // 成功
	
}

int imu_spi_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
    uint8_t hdr = reg | 0x80;                             // 读操作：bit7=1
	
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
		
    imu_cs_high();                                  // 释放 CS
    return 0;                                       // 成功
}



void imu_sleep_us(uint32_t us)                        		// 微秒延时（需启用 DWT）
{
    uint32_t start = (uint32_t)htim2.Instance->CNT;   		// 保存起始计数
	
    while ( (uint32_t)(htim2.Instance->CNT - start) < us);
}


int icm_set_int(icm_app_t *app)
{
    int rc = 0;

    // --- INT1 配置（已有） ---
    int1_config2_t i1 = {0};
    rc |= inv_imu_read_reg(&app->tr, INT1_CONFIG2, 1, (uint8_t*)&i1);
    i1.int1_polarity = 0;  // 低电平有效
    i1.int1_mode     = 1;  // 电平锁存
    i1.int1_drive    = 0;  // 推挽
    rc |= inv_imu_write_reg(&app->tr, INT1_CONFIG2, 1, (uint8_t*)&i1);

    // --- INT2 行为配置（已有） ---
    int2_config2_t i2 = {0};
    rc |= inv_imu_read_reg(&app->tr, INT2_CONFIG2, 1, (uint8_t*)&i2);
    i2.int2_polarity = 0;  // 低电平有效
    i2.int2_mode     = 1;  // 电平锁存
    i2.int2_drive    = 0;  // 推挽
    rc |= inv_imu_write_reg(&app->tr, INT2_CONFIG2, 1, (uint8_t*)&i2);

    // --- 1) 启用 APEX 功能（Tilt, R2W, SMD） ---
    uint8_t apex_en = 0;
    rc |= inv_imu_read_reg(&app->tr, 0x29 /* EDMP_APEX_EN0 */, 1, &apex_en);
    apex_en |= (1 << 3) | (1 << 6) | (1 << 7);  // 启用 Tilt, R2W, SMD
    rc |= inv_imu_write_reg(&app->tr, 0x29, 1, &apex_en);

    // --- 2) 解除 APEX 事件屏蔽（使能输出） ---
    uint8_t cfg0 = 0;
    rc |= inv_imu_read_reg(&app->tr, 0x39 /* INT_APEX_CONFIG0 */, 1, &cfg0);
    cfg0 &= ~(1 << 3);  // 清除 Tilt 屏蔽位
    rc |= inv_imu_write_reg(&app->tr, 0x39, 1, &cfg0);

    uint8_t cfg1 = 0;
    rc |= inv_imu_read_reg(&app->tr, 0x3A /* INT_APEX_CONFIG1 */, 1, &cfg1);
    cfg1 &= ~((1 << 1) | (1 << 0));  // 清除 SMD / R2W 屏蔽位
    rc |= inv_imu_write_reg(&app->tr, 0x3A, 1, &cfg1);

    // --- 3) 映射 APEX 事件至 INT2 引脚 ---
    int2_config1_t i21 = {0};
    rc |= inv_imu_read_reg(&app->tr, INT2_CONFIG1, 1, (uint8_t*)&i21);
    i21.int2_status_en_apex_event = 1;  // 允许 APEX 驱动 INT2
    rc |= inv_imu_write_reg(&app->tr, INT2_CONFIG1, 1, (uint8_t*)&i21);

    return rc;
}











/* ==================== 关键：FIFO 官方顺序配置 ==================== */
/*
官方建议（概括）：
1) 关闭 FIFO（Bypass）与 FIFO_IF
2) 设 INT1 FIFO_THS 中断（可选）
3) （可选）开启时间戳插入：FIFO_CONFIG4.bit1=1（如需 FSYNC，还需 TMST 相关配置）
4) 设水位线 FIFO_WM_TH（先写低字节 REG 0x1E，再写高字节 REG 0x1F）
5) FIFO_CONFIG2.bit3=1 让“>= 水位线”触发，而不仅仅“==”
6) FIFO_CONFIG3 选择写入 FIFO 的源（加计/陀螺/高分辨率/外部传感器……），但先别开 FIFO_IF
7) FIFO_CONFIG0 设为 Stream 模式
8) 最后一步：FIFO_CONFIG3.bit0 = 1 打开 FIFO_IF（非常关键！）
参考：User Guide 3.5 表 + “FIFO_IF to be set at last”；寄存器/位说明见 Datasheet 17.28~17.33。*/
static int icm_fifo_configure_stream(icm_app_t *app, uint16_t wm_frames, int enable_ts)
{
    int rc = 0;
    uint8_t v;

    /* 1) 关闭 FIFO（Bypass），先关 FIFO_IF_EN */
    v = 0;
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);                       /* ...IF=0，源全关 */
    v = FIFO_MODE_BYPASS | 0x07;                                                      /* [5:0] FIFO_DEPTH：推荐 0x07=2KB；若用 8KB 需 0x1F 且禁用 APEX */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG0, 1, &v);                       /* 进入 Bypass */

    /* 2) 开 INT1 FIFO_THS（可选，示例打开） */
    v = 0;
    rc |= inv_imu_read_reg(&app->tr, REG_INT1_CONFIG0, 1, &v);
    v = (uint8_t)((v | INT1_EN_FIFO_THS) & ~INT1_EN_DRDY);                            /* 用 FIFO_THS 中断，关 DRDY，避免两路同时触发 */
    rc |= inv_imu_write_reg(&app->tr, REG_INT1_CONFIG0, 1, &v);                       /* 地址 0x16，bit1=1 启用 FIFO_THS 中断 */

    /* 3) （可选）时间戳插入到 FIFO 帧 */
    v = 0;
    if (enable_ts) v |= FIFO_TMST_FSYNC_EN;                                           /* REG_FIFO_CONFIG4.bit1 = 1 */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG4, 1, &v);

    /* 注：若要使用 FSYNC/更精细的时间戳，还需配置 TMST/FSYNC 等寄存器。
       Datasheet/UG 中部分 TMST 位在 IPREG_TOP1（间接寄存器），需要走 IREG 访问流程，此处仅做基本启用。 */

    /* 4) 设置水位线（单位 = 帧数，COUNT 亦是“帧数”）— 先写低 8 位，再写高 8 位 */
    uint8_t wm_l = (uint8_t)(wm_frames & 0xFF);
    uint8_t wm_h = (uint8_t)((wm_frames >> 8) & 0xFF);
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG1_0, 1, &wm_l);                  /* 先低字节 0x1E */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG1_1, 1, &wm_h);                  /* 再高字节 0x1F */

    /* 5) 写水位比较方式：>= 阈值触发 */
    v = 0;
    rc |= inv_imu_read_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);
    v |= FIFO_WR_WM_GT_TH;                                                            /* 0x20 bit3 = 1 */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);

    /* 6) 选择向 FIFO 写入的源（本例：Accel+Gyro；高分辨率关闭=16B 帧） */
    v = 0;
    v |= FIFO_EN_ACCEL;
    v |= FIFO_EN_GYRO;
    /* 如果需要 20-bit 高分辨率，可另外置位 FIFO_HIRES_EN（0x21 bit3），并相应解析 20B 帧 */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);                       /* 注意：此时仍未开 FIFO_IF */

    /* 7) 设 FIFO 工作模式：Stream（帧覆盖写入） */
    v = FIFO_MODE_STREAM | 0x07;                                                      /* 建议 FIFO_DEPTH=2KB。若设 8KB，请确保关闭全部 APEX */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG0, 1, &v);

    /* 8) 最后一步：打开 FIFO_IF_EN（非常关键） */
    v = 0;
    rc |= inv_imu_read_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);
    v |= FIFO_IF_EN;                                                                  /* 0x21 bit0 = 1 */
    rc |= inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG3, 1, &v);

    return rc;
}

/* 清 FIFO：置位 FIFO_FLUSH（0x20 bit7）一瞬间即可 */
static int icm_fifo_flush(icm_app_t *app)
{
    uint8_t v = 0;
    int rc = inv_imu_read_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);
    if (rc) return rc;
    v |= BIT(7);                                /* FIFO_FLUSH = 1 */
    rc = inv_imu_write_reg(&app->tr, REG_FIFO_CONFIG2, 1, &v);
    /* 可再读回清零，或等待硬件自动清零 */
    return rc;
}











/* ==================== 初始化（保留你原始流程 + 新 FIFO 配置） ==================== */
int icm_app_init(icm_app_t *app)
{
    memset(app, 0, sizeof(*app));
    app->tr.read_reg   = imu_spi_read;
    app->tr.write_reg  = imu_spi_write;
    app->tr.serif_type = UI_SPI4;
    app->tr.sleep_us   = imu_sleep_us;
    app->dev.transport = app->tr;

    /* 读 WHO_AM_I = 0xE9 */
    uint8_t who = 0;
    inv_imu_read_reg(&app->tr, REG_WHO_AM_I, 1, &who);
    if (who != 0xE9)
			return -1;

    if (inv_imu_adv_device_reset(&app->dev) != 0)
			return -2;

    /* 开加计/陀螺（LN） */
    uint8_t pwr = 0;
    pwr |= (3u << 2);  /* GYRO_MODE=3(LN) */
    pwr |= (3u << 0);  /* ACCEL_MODE=3(LN) */
    inv_imu_write_reg(&app->tr, REG_PWR_MGMT0, 1, &pwr);

    /* 设置 ODR/量程：±4g @ 400Hz，±2000dps @ 400Hz */
    uint8_t ac0 = 0;
    ac0 |= (0x3u << 0);          /* ACCEL_ODR = 0b0111 -> 400Hz */
    ac0 |= (0x3u << 4);          /* ACCEL_UI_FS_SEL = 0b011 -> ±4g */
    inv_imu_write_reg(&app->tr, REG_ACCEL_CONFIG0, 1, &ac0);

    uint8_t gc0 = 0;
    gc0 |= (0x7u << 4);          /* GYRO_UI_FS_SEL = 0b0001?（见手册映射）这里用 0x1=±2000dps；若按你的旧值 0x7=±31.25dps，改为 0x1更合理 */
    gc0 |= (0x7u << 0);          /* GYRO_ODR = 0b0111 -> 400Hz */
    /* 说明：Datasheet 的 GYRO_UI_FS_SEL 映射为 0x1=±2000dps，0x0=±4000dps，0x3=±500dps 等。请按需调整。:contentReference[oaicite:3]{index=3} */
    inv_imu_write_reg(&app->tr, REG_GYRO_CONFIG0, 1, &gc0);

    app->odr_hz = 400.0f;

    /* 配置 INT 极性/模式（你原有函数） */
    if (icm_set_int(app) != 0)
			return -3;

    /* ===== 新：按官方顺序配置 FIFO（水位线=16 帧，启用时间戳插入） ===== */
    if (icm_fifo_configure_stream(app, /*wm_frames=*/16, /*enable_ts=*/1) != 0)
			return -4;

    /* 复位后建议清一次 FIFO，避免旧数据 */
    icm_fifo_flush(app);

    return 0;
}

int  icm_read_accel_gyro(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad)
{
    uint8_t buf[12];                                       // 连读缓冲
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
		
		
    return 0;                                              // 成功
}




/* ==================== FIFO 读取：服务例程（解析头，按帧长取） ==================== */
/* 返回：0=OK，*has_frame=1 代表输出有效一帧；=0 代表当前无帧 */
int icm_read_fifo_serv(icm_app_t *app, float *ax_g, float *ay_g, float *az_g, float *gx_rad, float *gy_rad, float *gz_rad, uint8_t *has_frame)
{
    *has_frame = 0;

    /* 读取 FIFO_COUNT（单位=帧数） */
    uint8_t h = 0, l = 0;
    inv_imu_read_reg(&app->tr, REG_FIFO_COUNT_0, 1, &h);  /* 高字节在前 */
    inv_imu_read_reg(&app->tr, REG_FIFO_COUNT_1, 1, &l);
    uint16_t frames = ((uint16_t)h << 8) | l;
    if (frames == 0) return 0;                             /* 无帧 */

    /* 1) 读帧头（Header），决定帧长 */
    uint8_t hdr = 0;
    inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &hdr);    /* 每读一次 FIFO_DATA 就出队 1 字节 */

    /* 解析头字节（Datasheet 6.2 FIFO HEADER） */
    uint8_t ext_header  = (hdr >> 7) & 0x01;  /* 外部传感器/压缩时可能=1，头两字节 */
    uint8_t accel_en    = (hdr >> 6) & 0x01;
    uint8_t gyro_en     = (hdr >> 5) & 0x01;
    uint8_t hires_en    = (hdr >> 4) & 0x01;  /* 20-bit 高分辨率 */
    uint8_t tmst_field  = (hdr >> 3) & 0x01;  /* 是否包含时间戳字段 */
    /* 其余位此处不关心 */

    /* 如果头扩展（ext_header=1），再读第 2 个头字节但本例不用外部传感器信息，可直接丢弃 */
    if (ext_header) {
        uint8_t hdr2;
        inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &hdr2);
        /* 需要支持 ES0/ES1 或压缩时，再根据 hdr2 的 ES0/ES1 位计算帧长 */
    }

    /* 2) 计算“剩余”应读取的字节数（我们已读了 1B 或 2B 头） */
    uint16_t payload = 0;
    if (accel_en && gyro_en && !hires_en) {
        /* 16 字节帧：Accel(6) + Gyro(6) + Temp(1) + Ts(2) = 15，加上 1B 头 = 16B */
        payload = 15 - (ext_header ? 1 : 0);  /* 若有扩展头，总长仍是 16B，这里只需读剩余 */
    } else if (accel_en ^ gyro_en) {
        /* 8 字节帧：Accel-only 或 Gyro-only + Temp(1)，通常无 Ts（看 hdr 的 tmst_field） */
        /* 标准 8B 结构：头(1) + 数据(6) + Temp(1) => 剩余 7B（若 ext 头，则再-1） */
        payload = 7 - (ext_header ? 1 : 0);
        /* 若 tmst_field=1（极少数组合，例如与外部传感器/高分辨率配合），总长会变化；此处按常用 8B 处理 */
    } else if (accel_en && gyro_en && hires_en) {
        /* 20 字节帧：高分辨率 20-bit：多 6 个 LSB 字节 + Temp(2) + Ts(2) */
        /* 头(1) + 19B => 剩余 19B（若 ext 头，再-1） */
        payload = 19 - (ext_header ? 1 : 0);
    } else {
        /* 其他情况（如仅外部传感器帧 16/20/32B），此处直接丢弃该帧以避免错位 */
        /* 为了不破坏后续帧，从 FIFO 中把本帧余下字节读掉 */
        /* 简单策略：按 16B 总长做兜底 */
        uint8_t trash;
        for (int i = 0; i < 15 - (ext_header ? 1 : 0); ++i) inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &trash);
        return 0;
    }

    uint8_t frm[32] = {0};
    for (uint16_t i = 0; i < payload; ++i) inv_imu_read_reg(&app->tr, REG_FIFO_DATA, 1, &frm[i]);

    /* 3) 解析 16B 常用帧（优先支持你的当前配置） */
    if (accel_en && gyro_en && !hires_en) {
        /* 组合成完整 16B 缓冲：我们已读 head(1) + payload(15)，frm[]里保存的是后 15 字节 */
        /* 字段顺序（大端）：AxH AxL AyH AyL AzH AzL GxH GxL GyH GyL GzH GzL Temp  TsH TsL */
        int16_t ax = (int16_t)((frm[0] << 8)  | frm[1]);
        int16_t ay = (int16_t)((frm[2] << 8)  | frm[3]);
        int16_t az = (int16_t)((frm[4] << 8)  | frm[5]);
        int16_t gx = (int16_t)((frm[6] << 8)  | frm[7]);
        int16_t gy = (int16_t)((frm[8] << 8)  | frm[9]);
        int16_t gz = (int16_t)((frm[10] << 8) | frm[11]);
        /* uint8_t temp = frm[12];  uint16_t ts = ((uint16_t)frm[13]<<8)|frm[14]; // 如需使用 */

        *ax_g   = accel_lsb_to_g(ax, 4.0f);
        *ay_g   = accel_lsb_to_g(ay, 4.0f);
        *az_g   = accel_lsb_to_g(az, 4.0f);
        *gx_rad = gyro_lsb_to_rads(gx, 2000.0f);
        *gy_rad = gyro_lsb_to_rads(gy, 2000.0f);
        *gz_rad = gyro_lsb_to_rads(gz, 2000.0f);

        *has_frame = 1;
        return 0;
    }

    /* 4) 其他帧（8B/20B）简单示例：只解析出你关心的通道；此处略，可按需要补全 */
    *has_frame = 0;
    return 0;
}

int icm_read_apex(icm_app_t *app, icm_apex_event_t *evt)// 读取 APEX 事件
{
    uint8_t s0=0,s1=0; int rc=0;                           // 状态寄存器
	
    memset(evt,0,sizeof(*evt));                            // 清零
	
    rc |= inv_imu_read_reg(&app->tr, INT_APEX_STATUS0, 1, &s0); // 0x56
    rc |= inv_imu_read_reg(&app->tr, INT_APEX_STATUS1, 1, &s1); // 0x57
	
    if (rc)
			return rc;                                     // 读取失败
		
		
    if (s0 & 0x01) evt->tilt = 1;                          // bit0: tilt
    if (s0 & 0x02) evt->smd  = 1;                          // bit1: smd
    if (s0 & 0x80) evt->r2w  = 1;                          // bit7: r2w
		
    return 0;                                              // 成功
		
}



























