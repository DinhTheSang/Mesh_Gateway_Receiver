################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../hardware/kit/common/bsp/bsp_stk.c 

OBJS += \
./hardware/kit/common/bsp/bsp_stk.o 

C_DEPS += \
./hardware/kit/common/bsp/bsp_stk.d 


# Each subdirectory must supply rules for building sources it contributes
hardware/kit/common/bsp/bsp_stk.o: ../hardware/kit/common/bsp/bsp_stk.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -std=c99 '-DHAL_CONFIG=1' '-D__STACK_SIZE=0x1000' '-D__HEAP_SIZE=0x1200' '-DMESH_LIB_NATIVE=1' '-DMBEDTLS_CONFIG_FILE="mbedtls_config.h"' '-DEFR32BG13P632F512GM48=1' -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\emlib\inc" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\mbedtls\sl_crypto\include" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\mbedtls\sl_crypto" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\mbedtls\sl_crypto\src" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\mbedtls\include" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\mbedtls" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\protocol\bluetooth\bt_mesh\inc\soc" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\emlib\src" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\radio\rail_lib\protocol\ieee802154" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\hardware\kit\common\halconfig" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\Device\SiliconLabs\EFR32BG13P\Include" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\emdrv\sleep\src" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\radio\rail_lib\common" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\protocol\bluetooth\bt_mesh\inc" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\hardware\kit\common\bsp" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\CMSIS\Include" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\protocol\bluetooth\bt_mesh\src" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\halconfig\inc\hal-config" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\emdrv\uartdrv\inc" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\hardware\kit\common\drivers" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\protocol\bluetooth\bt_mesh\inc\common" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\Device\SiliconLabs\EFR32BG13P\Source\GCC" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\emdrv\sleep\inc" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\radio\rail_lib\protocol\ble" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\emdrv\gpiointerrupt\inc" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\hardware\kit\EFR32BG13_BRD4104A\config" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\radio\rail_lib\chip\efr32\efr32xg1x" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\Device\SiliconLabs\EFR32BG13P\Source" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\bootloader\api" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\emdrv\common\inc" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\middleware" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\middleware\glib" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\middleware\glib\glib" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\middleware\glib\dmd\ssd2119" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\middleware\glib\dmd\display" -I"C:\Users\dthes\SimplicityStudio\v4_workspace\Mesh_Gateway_Receiver-master_29_6\platform\middleware\glib\dmd" -IC:/Users/huekh/SimplicityStudio/v4_workspace/Mesh_Transfer_AES_AD/mbedtls -IC:/Users/huekh/SimplicityStudio/v4_workspace/Mesh_Transfer_AES_AD/mbedtls/include -IC:/Users/huekh/SimplicityStudio/v4_workspace/Mesh_Transfer_AES_AD/mbedtls/include/mbedtls -IC:/Users/huekh/SimplicityStudio/v4_workspace/Mesh_Transfer_AES_AD/mbedtls/sl_crypto -IC:/Users/huekh/SimplicityStudio/v4_workspace/Mesh_Transfer_AES_AD/mbedtls/sl_crypto/include -Os -fno-builtin -flto -Wall -c -fmessage-length=0 -ffunction-sections -fdata-sections -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -MMD -MP -MF"hardware/kit/common/bsp/bsp_stk.d" -MT"hardware/kit/common/bsp/bsp_stk.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

