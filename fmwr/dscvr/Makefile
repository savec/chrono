PROJECT = chrono
TARGET = $(PROJECT).bin
EXECUTABLE = $(PROJECT).elf

CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld 
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump

#DEFS = -DSTM32F40_41xxx -DUSE_STDPERIPH_DRIVER -DHSE_VALUE=8000000UL

MCU = cortex-m3
	
MCFLAGS =	-mcpu=$(MCU)  
MCFLAGS +=	-ggdb 
MCFLAGS +=	-mthumb  
MCFLAGS +=	-mlittle-endian  
# MCFLAGS +=	-mfpu=fpv4-sp-d16  
MCFLAGS +=	-mfloat-abi=soft  
# MCFLAGS +=	-mthumb-interwork  
# MCFLAGS +=	-nostartfiles 
MCFLAGS +=	-specs=nosys.specs
MCFLAGS +=	-Xlinker -Map=$(PROJECT).map

HAL_DIR = $(HOME)/STM32Cube/Repository/STM32Cube_FW_L1_V1.4.0
CMSIS_DIR = $(HAL_DIR)/Drivers/CMSIS
CMSIS_CORE_INC = $(CMSIS_DIR)/Include
CMSIS_CORE_INC_LEGACY = $(CMSIS_CORE_INC)/Legacy
# DSP_DIR = $(CMSIS_DIR)/DSP_Lib

CMSIS_DEVICE_INC = $(CMSIS_DIR)/Device/ST/STM32L1xx/Include
PERIPHERAL_INC = $(HAL_DIR)/Drivers/STM32L1xx_HAL_Driver/Inc
PERIPHERAL_SRC = $(HAL_DIR)/Drivers/STM32L1xx_HAL_Driver/Src

BSP_DRIVERS_DIR = $(HAL_DIR)/Drivers/BSP/STM32L152C-Discovery

FREERTOS = $(HOME)/FreeRTOSV8.2.1/FreeRTOS
# FREERTOS = $(HAL_DIR)/Middlewares/Third_Party/FreeRTOS
FREERTOS_INC = $(FREERTOS)/Source/include
FREERTOS_CMSIS = $(FREERTOS)/Source/CMSIS_RTOS
FREERTOS_PORT = $(FREERTOS)/Source/portable/GCC/ARM_CM3

PROJECT_DIR = .
PROJECT_INC = $(PROJECT_DIR)/Inc
PROJECT_SRC = $(PROJECT_DIR)/Src

# DSP_SRC = $(DSP_DIR)/Source

INCLUDES = 	-I$(CMSIS_CORE_INC) 
INCLUDES += -I$(CMSIS_DEVICE_INC)  
INCLUDES += -I$(CMSIS_DEVICE_INC)  
INCLUDES += -I$(PERIPHERAL_INC)  
INCLUDES += -I$(PROJECT_INC) 
INCLUDES += -I$(FREERTOS_INC) 
INCLUDES += -I$(FREERTOS_CMSIS) 
INCLUDES += -I$(FREERTOS_PORT)
INCLUDES += -I$(BSP_DRIVERS_DIR)


OPTIMIZE = -O0

# LDSCRIPT = $(PROJECT_DIR)/stm32f4xx_flash.ld
LDSCRIPT = $(PROJECT_DIR)/STM32L152XB_FLASH.ld
CFLAGS	= 	$(MCFLAGS)  $(OPTIMIZE)  $(INCLUDES) -Wall -T $(LDSCRIPT) \
			-DSTM32L152xB -DUSE_HAL_DRIVER \
			-D__FPU_PRESENT=0
			# -D__FPU_USED=1

AFLAGS	= $(MCFLAGS) 

MIDDLEWARES_SRC = 		$(FREERTOS)/Source/portable/MemMang/heap_4.c 
MIDDLEWARES_SRC +=		$(FREERTOS)/Source/list.c 
MIDDLEWARES_SRC +=		$(FREERTOS)/Source/tasks.c 
MIDDLEWARES_SRC +=		$(FREERTOS)/Source/event_groups.c 
MIDDLEWARES_SRC +=		$(FREERTOS)/Source/queue.c 
MIDDLEWARES_SRC +=		$(FREERTOS)/Source/croutine.c 
MIDDLEWARES_SRC +=		$(FREERTOS)/Source/timers.c 
MIDDLEWARES_SRC +=		$(FREERTOS)/Source/portable/GCC/ARM_CM3/port.c 
# MIDDLEWARES_SRC +=		$(FREERTOS)/Source/portable/GCC/ARM_CM3/portasm.s

# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_gpio.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_gpio_ex.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_dma.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_dma_ex.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_def.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_rcc.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_rcc_ex.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_cortex.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_flash.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_flash_ex.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_pwr.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_pwr_ex.h;
# Drivers/STM32L1xx_HAL_Driver/Inc/stm32l1xx_hal_flash_ramfunc.h;
# Middlewares/Third_Party/FreeRTOS/Source/portable/IAR/ARM_CM3/portmacro.h;

DRIVERS_SRC =			$(PERIPHERAL_SRC)/stm32l1xx_hal_gpio.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_dma.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_rcc.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_rcc_ex.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_flash.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_flash_ex.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_cortex.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_pwr.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_pwr_ex.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_flash_ramfunc.c \
						$(PERIPHERAL_SRC)/stm32l1xx_hal_lcd.c \
						$(BSP_DRIVERS_DIR)/stm32l152c_discovery.c \
						$(BSP_DRIVERS_DIR)/stm32l152c_discovery_glass_lcd.c

APPLICATION_SRC =       $(PROJECT_SRC)/freertos.c \
				        $(PROJECT_SRC)/stm32l1xx_it.c \
				        $(PROJECT_SRC)/stm32l1xx_hal_msp.c \
				        $(PROJECT_SRC)/main.c  \
				        $(PROJECT_SRC)/system_stm32l1xx.c 


STARTUP = $(PROJECT_SRC)/startup_stm32l152xb.s


SRC = $(MIDDLEWARES_SRC) $(DRIVERS_SRC) $(APPLICATION_SRC) $(STARTUP)

all: $(TARGET)

$(TARGET): $(EXECUTABLE)
	$(CP) -O binary $^ $@

$(EXECUTABLE): $(SRC)
	@$(CC) $(CFLAGS) $^ -o $@

.PHONY: clean burn debug

burn:
	@./st-flash write $(TARGET) 0x8000000

debug:
#	openocd -f openocd_stlinkv2.cfg &
	arm-none-eabi-gdb --quiet -x debug_msc.gdb

clean:
	@rm -f $(TARGET) $(EXECUTABLE)