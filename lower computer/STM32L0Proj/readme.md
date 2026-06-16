控制管脚：
    PA1-->ADC_IN1
    PA2-->RXEN
    PA3-->TXEN
    PA4-->SPI1_NSS
    PA5-->SPI1_SCK
    PA6-->SPI1_MISO
    PA7-->SPI1_MOSI
    PB0-->DIO1
    PB1-->DIO2
    PA8-->TMP_CTRL
    PA9-->I2C1_SCL
    PA10-->I2C1_SDA
    PA13-->SYS_SWDIO
    PA15-->SYS_SWCLK
    PB3-->NRST_LORA
    PB6-->USART2_TX
    PB7-->USART2_RX

PA1: ADC_IN1，用于监测电压情况；
PA2-PA7,PB0-PB1,PA15,PB3：是LLCC68的
PA9-PA10：I2C管教，用于对温度传感器和磁传感器的操作；
PB6,PB7：串口，用于调试输出；
PA8：PMOS管控制LoRa、温度、磁场模块的电源导通，用于低功耗设计。
PA13,PA14：SWD调试接口。
TMP117就是温度传感器，I2C地址为1001000x；
MLX90393为磁传感器，I2C地址为0001100x
其中，ID作为一个设备编号。

每5秒打开一次PMOS开关，读取电压、温度、磁场，然后通过lora模块发射。发射数据格式：“ID:****,TEMP:****,Volt:****,MAG:****.”