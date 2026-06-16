



#ifndef __USER_CONFIG_H
#define __USER_CONFIG_H


#include "main.h"
#include "sx126x.h"




//打开直接进行定频测试
#define TEST 0



//spi
#define LCC68_NSS_PORT   GPIOA
#define LCC68_NSS_PIN    GPIO_PIN_4

#define LCC68_SCK_PORT   GPIOA
#define LCC68_SCK_PIN    GPIO_PIN_5

#define LCC68_MOSI_PORT  GPIOA
#define LCC68_MOSI_PIN   GPIO_PIN_7

#define LCC68_MISO_PORT  GPIOA
#define LCC68_MISO_PIN   GPIO_PIN_6


#define LCC68_NRST_PORT NRST_LORA_GPIO_Port
#define LCC68_NRST_PIN  NRST_LORA_Pin

#define LCC68_BUSY_PORT BUSY_GPIO_Port
#define LCC68_BUSY_PIN  BUSY_Pin



#define LCC68_DIO1_PORT DIO1_GPIO_Port
#define LCC68_DIO1_PIN  DIO1_Pin



//Uart
#define LOG_UART_TX_PORT GPIOB
#define LOG_UART_TX_PIN  GPIO_PIN_6
#define LOG_UART_RX_PORT GPIOB
#define LOG_UART_RX_PIN  GPIO_PIN_7


#define LCC68_RXEN_PORT RXEN_GPIO_Port
#define LCC68_RXEN_PIN  RXEN_Pin

#define LCC68_TXEN_PORT TXEN_GPIO_Port
#define LCC68_TXEN_PIN  TXEN_Pin





#define LORA_FRE									433000000	// frequency
#define LORA_PREAMBLE_LENGTH                        8        // PREAMBLE LENGTH
#define LORA_SX126x_SYMBOL_TIMEOUT                  0         // Symbols(SX126x)
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false			// PAYLOAD FIX LENGTH
#define LORA_IQ_INVERSION_ON                        false			// IQ INVERSION



#define SIZE_DATA  255



/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
    MODE_SLEEP                              = 0x00,         //! The radio is in sleep mode
    MODE_STDBY_RC,                                          //! The radio is in standby mode with RC oscillator
    MODE_STDBY_XOSC,                                        //! The radio is in standby mode with XOSC oscillator
    MODE_FS,                                                //! The radio is in frequency synthesis mode
    MODE_TX,                                                //! The radio is in transmit mode
    MODE_RX,                                                //! The radio is in receive mode
    MODE_RX_DC,                                             //! The radio is in receive duty cycle mode
    MODE_CAD                                                //! The radio is in channel activity detection mode
}RadioOperatingModes_t;








extern uint8_t rxbuff[SIZE_DATA]; 
extern uint8_t DataLen;
extern uint8_t pdata;


extern uint8_t IrqFired;
extern sx126x_irq_mask_t radioFlag;
extern sx126x_rx_buffer_status_t offset;
extern sx126x_pkt_status_lora_t RadioPktStatus;







extern void Data_Processing(uint8_t id, uint32_t interval_ms);

extern void LoraInit(void);

extern void gpio_init(void);
extern void DIO1_Init(void);

extern void LoraDataSend(uint8_t *data,uint8_t len);
extern void DX_Lora_RadioIrqProcess(void);

extern RadioOperatingModes_t sx1262GetOperatingMode(void);

extern void sx1262SetOperatingMode(RadioOperatingModes_t mode);



#endif

