# ARM-RTOS
This example runs on core
An RTOS kernel is employed and thus the code mainly creates a thread for the RTOS scheduler
The control data is sent by creating SPI and I2C channel.
The high speed data is sent by bulk endpoint by creating a auto DMA channel. 
The high speed bus is a proprietary bus protocol developed by GPIF designer.
This application can be used to transfer control data and high speed data between USB host to main devices such as FPGA, image sensors and etc.
