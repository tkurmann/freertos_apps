This extension was made to be used with the coral dev board (https://coral.ai/products/dev-board).

The imx8mq processer contains 4x A53 cores and a cortex M4. MPU communication is performed over rpmsg, which has a linux tty driver, making it ideally suited for the serial interface of micro-ROS

# Build instructions


# Download code
The easiest way to flash the M4 is to use uboot. You will need an ethernet connection from a local machine to the coral board. 
- Install a tftp server on your local machine
- Connect to console port (for me /dev/ttyUSB0)
- Boot to uboot
- Setup networking
- Run "tftp 0x7e0000 micro-ROS.bin" this will download the binary and copy it the TCM of the M4. Other variants of the imx8 may have a different address space.  
- Run "bootaux 0x7e0000" this will boot the core
- Run "boot" to boot to Linux


# Run on the board
- Boot to Linux
- install tty driver ("sudo insmod /lib/modules/4.14.98-imx/kernel/drivers/rpmsg/imx_rpmsg_tty.ko")
- check if "/dev/ttyRPMSG30" exists
- start agent "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyRPMSG30"


# Notes on Debugging
- Debugging is not very easy if you dont have a JTAG adapter (like me...). The easiest way is to use UART3 as a debug console. The usb<->uart adapter on the board has two interfaces UART1 (typcially /dev/ttyUSB0) and UART3 (/dev/ttyUSB1). The default Linux kernel is compiled to use UART3 as a device, so you will need to modify the device tree arch/arm64/boot/dts/freescale/fsl-imx8mq-phanbell.dts to disable UART3. 



