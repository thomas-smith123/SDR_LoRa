#include <sx126x_hal.h>
//#include "driver_usart.h"
//#include "driver_spi.h"
//#include "driver_key.h"
//#include "driver_timer.h"
//#include "driver_gpio.h"
#include "UserConfig.h"
//#include "stm32f1xx_hal_gpio.h"
#include "sx126x.h"
#include "stm32l0xx_hal.h"






static sx126x_hal_status_t sx126x_wait_while_busy(void)
{
  uint32_t start_tick = HAL_GetTick();

  while (sx126x_get_busy())
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) >= 100U)
    {
      return SX126X_HAL_STATUS_ERROR;
    }
    HAL_Delay(1);
  }

  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,const uint8_t *data, const uint16_t data_length)
{
    if (sx126x_wait_while_busy() != SX126X_HAL_STATUS_OK)
    {
        return SX126X_HAL_STATUS_ERROR;
    }
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_RESET);
      if(command_length > 0)
      {
        SPI_WriteBytes((uint8_t *)command,command_length);
      }

    if (sx126x_wait_while_busy() != SX126X_HAL_STATUS_OK)
    {
        HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
        return SX126X_HAL_STATUS_ERROR;
    }

      if(data_length > 0)
      {
        SPI_WriteBytes((uint8_t *)data,data_length);
      }
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
	  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,uint8_t *data, const uint16_t data_length)
{          
    if (sx126x_wait_while_busy() != SX126X_HAL_STATUS_OK)
    {
        return SX126X_HAL_STATUS_ERROR;
    }
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_RESET);
      
      if(command_length > 0)
      {
        SPI_WriteBytes((uint8_t *)command,command_length);
      }
    if (sx126x_wait_while_busy() != SX126X_HAL_STATUS_OK)
    {
        HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
        return SX126X_HAL_STATUS_ERROR;
    }

      if(data_length > 0)
      {
	    SPI_ReadBytes((uint8_t *)data, data_length);
      }
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
	  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
	HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCC68_NRST_PORT, LCC68_NRST_PIN, GPIO_PIN_RESET);
    HAL_Delay(50);
	HAL_GPIO_WritePin(LCC68_NRST_PORT, LCC68_NRST_PIN, GPIO_PIN_SET);
    HAL_Delay(20);
	return sx126x_wait_while_busy();
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context)
{
    if (sx126x_wait_while_busy() != SX126X_HAL_STATUS_OK)
    {
        return SX126X_HAL_STATUS_ERROR;
    }
      uint8_t data[] = {0xc0,0x00};
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_RESET);
	  // ?0xC0?????0x00 (????)
      SPI_WriteBytes(data,2);
	  HAL_GPIO_WritePin(LCC68_NSS_PORT, LCC68_NSS_PIN, GPIO_PIN_SET);
	  return SX126X_HAL_STATUS_OK;
}

int sx126x_get_busy(void)
{
	return HAL_GPIO_ReadPin(LCC68_BUSY_PORT,LCC68_BUSY_PIN);
}
