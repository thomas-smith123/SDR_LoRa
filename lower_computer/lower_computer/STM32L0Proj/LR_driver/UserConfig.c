



#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "UserConfig.h"
#include "sx126x.h"
#include "sx126x_hal.h"
#include <string.h>
#include <stdlib.h>


static volatile uint8_t g_lora_test_rxs;
static volatile uint8_t g_lora_tx_done;
/* CAD/LBT 过程标志：DIO1 中断触发 CAD_DONE 后由 CadDone() 更新。 */
static volatile uint8_t g_lora_cad_done;
static volatile uint8_t g_lora_cad_detected;


uint8_t IrqFired = false;
sx126x_rx_buffer_status_t offset = {0};
sx126x_pkt_status_lora_t RadioPktStatus;
sx126x_irq_mask_t radioFlag = 0;

static volatile RadioOperatingModes_t OperatingMode;
void LoraOpenRXMode(uint8_t Timerout);
void RxEn(void);
void TxEn(void);

#define LORA_TX_INTERVAL_MS        500U
#define LORA_FRE_RAND_DELTA_HZ     1000U
#define LORA_LBT_ENABLE            1U
#define LORA_LBT_CAD_TIMEOUT_MS    80U
#define LORA_LBT_MAX_RETRY         5U
#define LORA_LBT_BACKOFF_MIN_MS    50U
#define LORA_LBT_BACKOFF_RAND_MS   200U

/*
 * CAD 阈值，当前按 LoRa SF7/BW125 取一组常用值。
 * 增大 peak/min：不容易误判忙，但可能漏检弱信号；减小 peak/min：更灵敏，但更容易误判信道忙。
 */
#define LORA_CAD_DETECT_PEAK       22U
#define LORA_CAD_DETECT_MIN        10U

static volatile uint32_t g_last_tx_done_tick = 0U;

static uint32_t lora_get_rand_freq_hz(void)
{
    const uint32_t base = (uint32_t)LORA_FRE;
    const int32_t delta = (int32_t)LORA_FRE_RAND_DELTA_HZ;
    /* 在中心频点附近做很小的随机偏移，用于测试场景下降低固定碰撞概率。 */
    const int32_t offset_hz = ((int32_t)(rand() % (int)(2 * delta + 1))) - delta; /* [-delta, +delta] */

    if (offset_hz >= 0)
    {
        return base + (uint32_t)offset_hz;
    }
    else
    {
        const uint32_t neg = (uint32_t)(-offset_hz);
        return (base > neg) ? (base - neg) : 0U;
    }
}

static void LoraSetDataIrq(void)
{
    /* 正常收发数据阶段，DIO1 只映射 RX_DONE/TX_DONE/TIMEOUT/CRC_ERROR 等数据相关中断。 */
    sx126x_set_dio_irq_params(NULL,
                              SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERROR,
                              SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_CRC_ERROR,
                              SX126X_IRQ_NONE,
                              SX126X_IRQ_NONE);
    sx126x_clear_irq_status(NULL, SX126X_IRQ_ALL);
}

static bool LoraChannelActivityDetected(void)
{
    sx126x_cad_params_t cad_params;
    uint32_t start_tick;

    /* 使用 4 个 LoRa 符号做 CAD，速度和可靠性折中；更多符号更稳但更耗时。 */
    cad_params.cad_symb_nb = SX126X_CAD_04_SYMB;
    cad_params.cad_detect_peak = LORA_CAD_DETECT_PEAK;
    cad_params.cad_detect_min = LORA_CAD_DETECT_MIN;
    /* CAD_ONLY：只检测信道活动，不自动进入 RX 或 TX。 */
    cad_params.cad_exit_mode = SX126X_CAD_ONLY;
    cad_params.cad_timeout = 0U;

    /* 清空上一轮 CAD 结果和 IRQ 标志，避免误用旧状态。 */
    g_lora_cad_done = false;
    g_lora_cad_detected = false;
    IrqFired = false;
    radioFlag = 0;

    RxEn();
    sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);
    /* CAD 阶段把 CAD_DONE/CAD_DETECTED 映射到 DIO1。 */
    sx126x_set_dio_irq_params(NULL,
                              SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED,
                              SX126X_IRQ_CAD_DONE | SX126X_IRQ_CAD_DETECTED,
                              SX126X_IRQ_NONE,
                              SX126X_IRQ_NONE);
    sx126x_clear_irq_status(NULL, SX126X_IRQ_ALL);
    sx126x_set_cad_params(NULL, &cad_params);
    sx1262SetOperatingMode(MODE_CAD);
    /* 启动 LLCC68 Channel Activity Detection。 */
    sx126x_set_cad(NULL);

    start_tick = HAL_GetTick();
    /* 等待 CAD_DONE 中断；中断回调会读取 radioFlag，DX_Lora_RadioIrqProcess() 再调用 CadDone()。 */
    while ((g_lora_cad_done == false) && ((uint32_t)(HAL_GetTick() - start_tick) < LORA_LBT_CAD_TIMEOUT_MS))
    {
        DX_Lora_RadioIrqProcess();
    }

    sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC);

    if (g_lora_cad_done == false)
    {
        /* CAD 超时没有得到结果时，为安全起见按“信道忙”处理，不立即发射。 */
        return true;
    }

    /* true 表示检测到 LoRa 活动，信道忙；false 表示未检测到活动，信道可用。 */
    return (g_lora_cad_detected != false);
}

static bool LoraWaitClearChannel(void)
{
    for (uint8_t retry = 0U; retry < LORA_LBT_MAX_RETRY; retry++)
    {
        /* LBT：Listen Before Talk，先侦听信道，空闲才允许发射。 */
        if (LoraChannelActivityDetected() == false)
        {
            LoraSetDataIrq();
            return true;
        }

        /* 信道忙时随机退避，避免多个节点固定间隔同时重试导致再次碰撞。 */
        HAL_Delay(LORA_LBT_BACKOFF_MIN_MS + (uint32_t)(rand() % LORA_LBT_BACKOFF_RAND_MS));
    }

    /* 多次检测仍忙：恢复普通数据中断配置，并放弃本次发送。 */
    LoraSetDataIrq();
    return false;
}



void gpio_init(void)
{
    
    // 定义GPIO的结构体变量
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // 使能LED的GPIO对应的时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin =  LCC68_BUSY_PIN;       
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; 
    GPIO_InitStruct.Pull = GPIO_PULLUP;        
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    // 初始化引脚配置
    HAL_GPIO_Init(LCC68_BUSY_PORT, &GPIO_InitStruct);
    HAL_Delay(20);


    GPIO_InitStruct.Pin =  LCC68_NRST_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 设置为推挽输出模式
    GPIO_InitStruct.Pull = GPIO_PULLUP;         // 默认上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;// 引脚输出速度设置为慢

    // 初始化引脚配置
    HAL_GPIO_Init(LCC68_NRST_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =  LCC68_RXEN_PIN;
    HAL_GPIO_Init(LCC68_RXEN_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =  LCC68_TXEN_PIN;
    HAL_GPIO_Init(LCC68_TXEN_PORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(LCC68_NRST_PORT, LCC68_NRST_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCC68_RXEN_PORT, LCC68_RXEN_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCC68_TXEN_PORT, LCC68_TXEN_PIN, GPIO_PIN_RESET);

    HAL_Delay(20);
}

void DIO1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PB0/DIO1 用作 LLCC68 中断输入；重新配置前先关中断，避免配置过程中误触发。 */
    HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);

    GPIO_InitStruct.Pin = LCC68_DIO1_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(LCC68_DIO1_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}


void SetTxHz(uint16_t HZ)
{
    sx126x_set_rf_freq(NULL,HZ * 1000000);
	
}




RadioOperatingModes_t sx1262GetOperatingMode(void)
{
    return OperatingMode;
}

void sx1262SetOperatingMode(RadioOperatingModes_t mode)
{
    OperatingMode = mode;
}



void RxEn(void)
{
    /* 外部射频开关切到接收通道。 */
    HAL_GPIO_WritePin(LCC68_RXEN_PORT, LCC68_RXEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCC68_TXEN_PORT, LCC68_TXEN_PIN, GPIO_PIN_RESET);

}
void TxEn(void)
{
    /* 外部射频开关切到发射通道。 */
    HAL_GPIO_WritePin(LCC68_RXEN_PORT, LCC68_RXEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCC68_TXEN_PORT, LCC68_TXEN_PIN, GPIO_PIN_SET);

}




void LoraInit(void)
{
    g_lora_test_rxs = false;
    g_lora_tx_done = true;

    /* 初始化随机数种子，用于 LBT 随机退避和测试频偏。 */
    srand((unsigned int)HAL_GetTick());
    
    /* IO复位+CS唤醒 模块*/
    sx126x_reset(NULL);
    sx126x_wakeup(NULL); 

    /* 状态机设定 */
    /* 进入 STDBY_RC 待机配置模式 */
    sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC );
    sx126x_set_standby(NULL, SX126X_STANDBY_CFG_XOSC );
	
    /* 选择内部电压调节器模式 高效DC-DC */
    sx126x_set_reg_mode(NULL, SX126X_REG_MODE_DCDC);
	
	   /* 内部FIFO读写地址复位 0x00 */
    sx126x_set_buffer_base_address(NULL,0x00,0x00);
	
	  sx126x_set_pkt_type(NULL,SX126X_PKT_TYPE_LORA);  
		sx126x_set_trimming_capacitor_values(NULL,0x4,0x2f);

    sx126x_mod_params_lora_t params;
    params.bw = SX126X_LORA_BW_125;
    params.sf = SX126X_LORA_SF7;
    params.cr = SX126X_LORA_CR_4_6;
    params.ldro = 0x00;
    sx126x_set_lora_mod_params(NULL, &params);

    sx126x_pkt_params_lora_t params2;
    params2.crc_is_on = 0;
    params2.invert_iq_is_on = 0;
    params2.pld_len_in_bytes = 0xff;
    params2.header_type = SX126X_LORA_PKT_EXPLICIT;
    params2.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    sx126x_set_lora_pkt_params(NULL, &params2);


    sx126x_pa_cfg_params_t  params3;
    params3.pa_duty_cycle = 0x04;
    params3.hp_max = 0x07;
    params3.device_sel = 0x00;
    params3.pa_lut = 0x01;
    sx126x_set_pa_cfg(NULL, &params3);
    		
    //打开dio1的中断 中断触发 SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE
    LoraSetDataIrq();
	
	 sx126x_set_tx_params(NULL,14 ,SX126X_RAMP_10_US);   	
	  /* 设置载波频率(频点) */
	 sx126x_set_rf_freq(NULL,LORA_FRE);
	 #if TEST
        g_lora_test_rxs = true;
        TxEn();
    #else
        LoraOpenRXMode(LORA_SX126x_SYMBOL_TIMEOUT);
    #endif
}


void set_LoraPacketParams(uint8_t size)
{

    sx126x_pkt_params_lora_t params2;
    params2.crc_is_on = 0;
    params2.invert_iq_is_on = 0;
    params2.pld_len_in_bytes = size;
    params2.header_type = SX126X_LORA_PKT_EXPLICIT;
    params2.preamble_len_in_symb = LORA_PREAMBLE_LENGTH;
    sx126x_set_lora_pkt_params(NULL, &params2);
}


void LoraDataSend(uint8_t *data,uint8_t len)
{
    #if LORA_LBT_ENABLE
    /* 发送前先做 CAD/LBT 信道侦听；信道忙则随机退避重试。 */
    if (LoraWaitClearChannel() == false)
    {
        /* 信道持续占用时放弃本次发送，回到 RX，避免和其他节点信号交叠。 */
        g_lora_tx_done = true;
        g_lora_test_rxs = true;
        LoraOpenRXMode(LORA_SX126x_SYMBOL_TIMEOUT);
        return;
    }
    #else
    /* 如关闭 LBT，则直接恢复普通数据 IRQ 配置后发送。 */
    LoraSetDataIrq();
    #endif

   /* 信道空闲：切换射频开关到 TX，写入 FIFO，然后启动发射。 */
   TxEn();
   set_LoraPacketParams(len);
   sx126x_write_buffer(NULL, 0x00, data, len);
   sx126x_set_tx(NULL,6000);
   g_lora_tx_done = false;
   g_lora_test_rxs = false;
   sx1262SetOperatingMode(MODE_TX);
}




void LoraOpenRXMode(uint8_t Timerout)
{
     g_lora_test_rxs = true;
     sx126x_set_rx(NULL,Timerout);
     sx1262SetOperatingMode(MODE_RX);
     RxEn();

}



//以下为接收，发送处理
void OnTxDone(void)
{    
    g_lora_tx_done = true;
    g_lora_test_rxs = true;

    g_last_tx_done_tick = HAL_GetTick();

    LoraOpenRXMode(LORA_SX126x_SYMBOL_TIMEOUT);
}

void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr)
{
		static uint8_t num = 0;
    g_lora_tx_done = true;
    g_lora_test_rxs = true;
		num += 1;
	  printf("%s",payload);
    LoraOpenRXMode(LORA_SX126x_SYMBOL_TIMEOUT);

}

void RxError(void)
{

    g_lora_tx_done = true;
    g_lora_test_rxs = true;




}

void CadDone ( bool channelActivityDetected )
{
    /* CAD_DONE 中断处理：保存本次信道检测结果，供 LoraChannelActivityDetected() 返回。 */
    g_lora_test_rxs = true;
    g_lora_tx_done = true;
    g_lora_cad_done = true;
    g_lora_cad_detected = channelActivityDetected;


}

void RxTimeout(void)
{
    g_lora_tx_done = true;
    g_lora_test_rxs = true;   
    LoraOpenRXMode(LORA_SX126x_SYMBOL_TIMEOUT);    
}

void TxTimeout(void)
{
    g_lora_tx_done = true;
    g_lora_test_rxs = true;   
    LoraOpenRXMode(LORA_SX126x_SYMBOL_TIMEOUT);    
}


uint8_t radioRxbuff[255] = {0};
void Hz_set(char *data,uint8_t len)
{
	uint32_t num = strtol(data, NULL, 10);

	printf("==>%d\r\n",num * 1000000);
	SetTxHz(num);
}

void Data_Processing(uint8_t id, uint32_t interval_ms)
{
    (void)id;
    if (interval_ms == 0U)
    {
        interval_ms = LORA_TX_INTERVAL_MS;
    }

    if ((g_lora_tx_done == true) && (g_lora_test_rxs == true))
    {
        const uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - (uint32_t)g_last_tx_done_tick) >= interval_ms)
        {
            uint8_t payload[32];
            const int payload_len = snprintf((char*)payload, sizeof(payload), "Hi%02u", (unsigned int)id);

            sx126x_set_rf_freq(NULL, lora_get_rand_freq_hz());
            LoraDataSend(payload, (uint8_t)((payload_len > 0) ? payload_len : 0));
        }
    }
}





void DX_Lora_RadioIrqProcess(void)
{

    if(IrqFired == true)
    {
        __disable_irq();
        IrqFired = false;
        __enable_irq();

        if( ( radioFlag & SX126X_IRQ_TX_DONE ) == SX126X_IRQ_TX_DONE )
        {
            sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC );
            OnTxDone();
        }
        if( ( radioFlag & SX126X_IRQ_RX_DONE ) ==  SX126X_IRQ_RX_DONE )
        {
            sx126x_set_standby(NULL,SX126X_STANDBY_CFG_RC );
            sx126x_get_rx_buffer_status(NULL, &offset);
            sx126x_read_buffer(NULL, offset.buffer_start_pointer, radioRxbuff, offset.pld_len_in_bytes);
            sx126x_get_lora_pkt_status(NULL, &RadioPktStatus);
            OnRxDone(&radioRxbuff[0],  offset.pld_len_in_bytes, RadioPktStatus.rssi_pkt_in_dbm + RadioPktStatus.snr_pkt_in_db, RadioPktStatus.snr_pkt_in_db);
            memset(radioRxbuff,0,255);
         
        }

         if( ( radioFlag &  SX126X_IRQ_CRC_ERROR ) ==  SX126X_IRQ_CRC_ERROR )
         {
            sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC );
            RxError();
         }

        if( ( radioFlag &  SX126X_IRQ_CAD_DONE ) ==  SX126X_IRQ_CAD_DONE )
         {
             sx126x_set_standby(NULL,SX126X_STANDBY_CFG_RC );
             CadDone((radioFlag & SX126X_IRQ_CAD_DETECTED) == SX126X_IRQ_CAD_DETECTED);
         }

         if((radioFlag & SX126X_IRQ_TIMEOUT) == SX126X_IRQ_TIMEOUT)
         {
             sx126x_set_standby(NULL, SX126X_STANDBY_CFG_RC );
            if( sx1262GetOperatingMode( ) == MODE_TX )
            {
                TxTimeout();
                
            }else if( sx1262GetOperatingMode( ) == MODE_RX )
            {
                RxTimeout();
            }            
        }

        if( ( radioFlag & SX126X_IRQ_PREAMBLE_DETECTED ) == SX126X_IRQ_PREAMBLE_DETECTED )
        {
             //__NOP( );
        }
        
        if( ( radioFlag & SX126X_IRQ_SYNC_WORD_VALID ) == SX126X_IRQ_SYNC_WORD_VALID )
        {
             //__NOP( );
        }
        
        if( ( radioFlag & SX126X_IRQ_HEADER_VALID ) == SX126X_IRQ_HEADER_VALID )
        {
              //__NOP( );
        }
               
        if( ( radioFlag & SX126X_IRQ_HEADER_ERROR ) == SX126X_IRQ_HEADER_ERROR )
        {        
            RxTimeout();
        }


    }

}
