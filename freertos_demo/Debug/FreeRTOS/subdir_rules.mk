################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
FreeRTOS/%.obj: ../FreeRTOS/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.3.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="P:/Courses/ENCE464/Helrig/freertos_demo" --include_path="P:/Courses/ENCE464/Helrig/freertos_demo/FreeRTOS" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178/examples/boards/ek-tm4c123gxl/drivers" --include_path="P:/Courses/ENCE464/Helrig/freertos_demo/FreeRTOS/include" --include_path="P:/Courses/ENCE464/Helrig/freertos_demo/FreeRTOS/portable/CCS/ARM_CM4F" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178" --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/include" --define=ccs="ccs" --define=PART_TM4C123GH6PM --gcc --abi=eabi --preproc_with_compile --preproc_dependency="FreeRTOS/$(basename $(<F)).d_raw" --obj_directory="FreeRTOS" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


