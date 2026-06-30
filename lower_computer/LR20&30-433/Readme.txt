* 编译器：Keil uVision5
* 基本信息：
	Core: 从SDK抽取的核心文件，方便修改；
	Driver: 用户编写的外设驱动文件；
	LR_driver：lora(LLCC68,SX1262公用)射频驱动文件，当前为Semtech官网下载
	Main: 主函数文件，及配置文件；
	Project: 工程文件，包含目标二进制文件；
	queue:该文件为队列spi接口，添加路径和引用即可调用，demo用于UART的数据处理
	SDK: CMSIS和HAL库，只包含用到的库，添加其它外设需要添加相应库；

	Readme.txt: 工程说明文件；

* 工程说明：
	当前文件为简单的(LLCC68,SX1262公用) demo使用案例，烧录即可使用，目前使用的控制IC为 STM32F103C8T6
	
* 注意：
	本工程示例只添加了代码用到的STM32 HAL库，用户在添加其它外设功能时，需要添加相应的HAL库或者使用工程模板（包含所有HAL库）
	本工程示例只提供用于(LLCC68,SX1262公用) Lora射频模组的驱动的一个简单样例，无法做到所有的功能都实现，具体实现需要自己完成
	本工程示例为了方便移植驱动文件，LR_driver驱动文件未进行任何修改，仅按照驱动文件里的readme修改了添加了如下函数
	The HAL (Hardware Abstraction Layer) is a collection of functions the user shall implement to write platform-dependant calls to the host. The list of functions is the following:
	- sx126x_hal_reset
	- sx126x_hal_wakeup
	- sx126x_hal_write
	- sx126x_hal_read