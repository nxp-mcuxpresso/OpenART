#ifndef RT_CONFIG_H__
#define RT_CONFIG_H__

/* Automatically generated file; DO NOT EDIT. */
/* RT-Thread Configuration */

/* RT-Thread Kernel */

#define RT_NAME_MAX 8
#define RT_ALIGN_SIZE 4
#define RT_THREAD_PRIORITY_32
#define RT_THREAD_PRIORITY_MAX 32
#define RT_TICK_PER_SECOND 1000
#define RT_USING_OVERFLOW_CHECK
#define RT_USING_HOOK
#define RT_USING_IDLE_HOOK
#define RT_IDLE_HOOK_LIST_SIZE 4
#define IDLE_THREAD_STACK_SIZE 256
#define RT_USING_TIMER_SOFT
#define RT_TIMER_THREAD_PRIO 4
#define RT_TIMER_THREAD_STACK_SIZE 512
#define RT_DEBUG
#define RT_DEBUG_COLOR

/* Inter-Thread communication */

#define RT_USING_SEMAPHORE
#define RT_USING_MUTEX
#define RT_USING_EVENT
#define RT_USING_MAILBOX
#define RT_USING_MESSAGEQUEUE
#define RT_USING_SIGNALS

/* Memory Management */

#define RT_USING_MEMPOOL
#define RT_USING_MEMHEAP
#define RT_USING_MEMHEAP_AS_HEAP
#define RT_USING_HEAP

/* Kernel Device Object */

#define RT_USING_DEVICE
#define RT_USING_CONSOLE
#define RT_CONSOLEBUF_SIZE 128
#define RT_CONSOLE_DEVICE_NAME "uart1"
#define RT_VER_NUM 0x40003
#define ARCH_ARM
#define RT_USING_CPU_FFS
#define ARCH_ARM_CORTEX_M
#define ARCH_ARM_CORTEX_FPU
#define ARCH_ARM_CORTEX_M7

/* RT-Thread Components */

#define RT_USING_COMPONENTS_INIT
#define RT_USING_USER_MAIN
#define RT_MAIN_THREAD_STACK_SIZE 8192
#define RT_MAIN_THREAD_PRIORITY 10

/* C++ features */


/* Command shell */

#define RT_USING_FINSH
#define FINSH_THREAD_NAME "tshell"
#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_PRIORITY 20
#define FINSH_THREAD_STACK_SIZE 4096
#define FINSH_CMD_SIZE 80
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_USING_MSH_ONLY
#define FINSH_ARG_MAX 10

/* Device virtual file system */

#define RT_USING_DFS
#define DFS_USING_WORKDIR
#define DFS_FILESYSTEMS_MAX 4
#define DFS_FILESYSTEM_TYPES_MAX 4
#define DFS_FD_MAX 16
#define RT_USING_DFS_MNTTABLE
#define RT_USING_DFS_ELMFAT

/* elm-chan's FatFs, Generic FAT Filesystem Module */

#define RT_DFS_ELM_CODE_PAGE 437
#define RT_DFS_ELM_WORD_ACCESS
#define RT_DFS_ELM_USE_LFN_3
#define RT_DFS_ELM_USE_LFN 3
#define RT_DFS_ELM_MAX_LFN 255
#define RT_DFS_ELM_DRIVES 2
#define RT_DFS_ELM_MAX_SECTOR_SIZE 512
#define RT_DFS_ELM_REENTRANT
#define RT_USING_DFS_DEVFS
#define RT_USING_DFS_RAMFS

/* Device Drivers */

#define RT_USING_DEVICE_IPC
#define RT_PIPE_BUFSZ 512
#define RT_USING_SYSTEM_WORKQUEUE
#define RT_SYSTEM_WORKQUEUE_STACKSIZE 2048
#define RT_SYSTEM_WORKQUEUE_PRIORITY 23
#define RT_USING_SERIAL
#define RT_SERIAL_RB_BUFSZ 64
#define RT_USING_HWTIMER
#define RT_USING_CPUTIME
#define RT_USING_CPUTIME_CORTEXM
#define RT_USING_I2C
#define RT_USING_PIN
#define RT_USING_ADC
#define RT_USING_PWM
#define RT_USING_RTC
#define RT_USING_SDIO
#define RT_SDIO_STACK_SIZE 2048
#define RT_SDIO_THREAD_PRIORITY 15
#define RT_MMCSD_STACK_SIZE 4096
#define RT_MMCSD_THREAD_PREORITY 22
#define RT_MMCSD_MAX_PARTITION 16
#define RT_USING_SPI
#define RT_USING_QSPI
#define RT_USING_WDT
#define RT_USING_SENSOR
#define RT_USING_SENSOR_CMD
#define RT_USING_TOUCH

/* Using USB */


/* POSIX layer and C standard library */

#define RT_USING_LIBC
#define RT_USING_POSIX

/* Network */

/* Socket abstraction layer */

#define RT_USING_SAL
#define SAL_USING_POSIX

/* Network interface device */

#define RT_USING_NETDEV
#define NETDEV_USING_IFCONFIG
#define NETDEV_USING_PING
#define NETDEV_USING_NETSTAT
#define NETDEV_USING_AUTO_DEFAULT
#define NETDEV_IPV4 1
#define NETDEV_IPV6 0

/* light weight TCP/IP stack */


/* AT commands */


/* VBUS(Virtual Software BUS) */


/* Utilities */


/* RT-Thread online packages */

/* IoT - internet of things */


/* Wi-Fi */

/* Marvell WiFi */


/* Wiced WiFi */


/* IoT Cloud */


/* security packages */


/* language packages */

#define MICROPYTHON_USING_MACHINE_TIMER
#define MICROPYTHON_USING_UOS
#define MICROPYTHON_USING_FILE_SYNC_VIA_IDE
#define PKG_MICROPYTHON_HEAP_SIZE 4194304

/* multimedia packages */


/* tools packages */


/* system packages */

/* acceleration: Assembly language or algorithmic acceleration packages */


/* Micrium: Micrium software products porting for RT-Thread */


/* peripheral libraries and drivers */


/* AI packages */


/* miscellaneous packages */


/* samples: kernel and components samples */


/* entertainment: terminal games and other interesting software packages */

#define SOC_IMXRT1170_SERIES

/* Hardware Drivers */

#define BSP_USING_4MFLASH
#define SOC_MIMXRT1176DVMMA

/* On-chip Peripheral Drivers */

#define BSP_USING_GPIO
#define BSP_USING_LPUART
#define BSP_USING_LPUART1
#define BSP_USING_HWTIMER
#define BSP_USING_HWTIMER1
#define BSP_USING_HWTIMER2
#define BSP_USING_I2C
#define HW_I2C1_BADURATE_100kHZ
#define BSP_USING_I2C5
#define HW_I2C5_BADURATE_100kHZ
#define BSP_USING_I2C6
#define HW_I2C6_BADURATE_100kHZ
#define BSP_USING_FLASH

/* Onboard Peripheral Drivers */

#define RT_USING_MIPI_LCD
#define PANEL_RK055HDMIPI4M
#define RT_USING_RGB_LCD
#define BSP_USING_SDIO
#define NXP_USING_USB_STACK
#define BSP_USING_SPIFLASH_PARTITION
#define BSP_SPIFLASH_PARTITION_OFFSET 256
#define BSP_SPIFLASH_PARTITION_SIZE 4096

/* OpenMV Hardware */

#define RT_USING_CSI

/* CSI components */

#define RT_USING_MIPI_CSI
#define RT_USING_LCD
#define RT_USING_MIPI_DSI
#define BSP_SENSOR_BUS_NAME "i2c6"
#define SENSOR_OV5640

/* NXP Software Components */

/* MicroPython */

#define NXP_USING_MICROPYTHON

/* Hardware Module */

#define MICROPY_HW_ENABLE_LED
#define MICROPY_HW_LED_NUM 2
#define MICROPY_HW_ENABLE_SERVO

/* System Module */

#define MICROPY_QSTR_BYTES_IN_HASH 2

/* Tools Module */

/* Network Module */

#define MICROPYTHON_USING_LVGL
#define NXP_MICROPYTHON_THREAD_STACK_SIZE 32768

/* OpenMV */

#define NXP_USING_OPENMV
#define NXP_USING_NNCU
#define WEIT_CACHE_SIZE 61440
#define NXP_USING_OMV_TFLITE
#define NXP_USING_GLOW

#endif
