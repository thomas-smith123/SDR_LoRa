#include <sx126x_hal.h>
#include "UserConfig.h"
#include "main.h"
#include "sx126x.h"

extern SPI_HandleTypeDef hspi1;

/* BUSY 等待超时保护（单位 ms），防止 LoRa 模块异常时无限死等 */
#define BUSY_WAIT_TIMEOUT_MS  1000U

static int sx126x_wait_busy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (sx126x_get_busy())
    {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            return 0;  /* 超时返回 0 */
        }
    }
    return 1;  /* BUSY 变低返回 1 */
}

static void sx126x_spi_write(const uint8_t *data, uint16_t len)
{
  (void)HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100);
}

static void sx126x_spi_read(uint8_t *data, uint16_t len)
{
  (void)HAL_SPI_Receive(&hspi1, data, len, 100);
}





sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,const uint8_t *data, const uint16_t data_length)
{
      if (!sx126x_wait_busy(BUSY_WAIT_TIMEOUT_MS)) return SX126X_HAL_STATUS_ERROR;
      HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_RESET);
      if(command_length > 0)
      {
        sx126x_spi_write(command,command_length);
      }

      if (!sx126x_wait_busy(BUSY_WAIT_TIMEOUT_MS)) { HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET); return SX126X_HAL_STATUS_ERROR; }   
      
      if(data_length > 0)
      {
        sx126x_spi_write(data,data_length);
      }
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
	  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,uint8_t *data, const uint16_t data_length)
{          
      if (!sx126x_wait_busy(BUSY_WAIT_TIMEOUT_MS)) return SX126X_HAL_STATUS_ERROR;
      HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_RESET);
      
      if(command_length > 0)
      {
        sx126x_spi_write(command,command_length);
      }
      if (!sx126x_wait_busy(BUSY_WAIT_TIMEOUT_MS)) { HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET); return SX126X_HAL_STATUS_ERROR; }   
      
      if(data_length > 0)
      {
      sx126x_spi_read((uint8_t *)data, data_length);
      }
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
	  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
    if (!sx126x_wait_busy(BUSY_WAIT_TIMEOUT_MS)) return SX126X_HAL_STATUS_ERROR;
    HAL_Delay(20);
	HAL_GPIO_WritePin(LCC68_NRST_PORT, LCC68_NRST_PIN, GPIO_PIN_RESET);
  HAL_Delay(50);
	HAL_GPIO_WritePin(LCC68_NRST_PORT, LCC68_NRST_PIN, GPIO_PIN_SET);
  HAL_Delay(20);
    (void)sx126x_wait_busy(BUSY_WAIT_TIMEOUT_MS);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
      if (!sx126x_wait_busy(BUSY_WAIT_TIMEOUT_MS)) return SX126X_HAL_STATUS_ERROR;
      uint8_t data[] = {0xc0,0x00};
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_RESET);
	  // 向0xC0寄存器写入0x00 （用户实现）
      sx126x_spi_write(data,2);
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
	  return SX126X_HAL_STATUS_OK;
}

int sx126x_get_busy(void)
{
	return HAL_GPIO_ReadPin(LCC68_BUSY_PORT,LCC68_BUSY_PIN);
}
