################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
OrbitOLED/%.obj: ../OrbitOLED/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/bin/armcl" -mv7M4 --code_state=16 --float_support=FPv4SPD16 -me --include_path="P:/Courses/ENCE464/Other files/Blinker/Blink" --include_path="P:/Courses/ENCE464/Other files/Blinker/Blink" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178/examples/boards/ek-tm4c123gxl/drivers" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178/utils" --include_path="C:/ti/TivaWare_C_Series-2.1.4.178/inc" --include_path="C:/ti/ccs920/ccs/tools/compiler/ti-cgt-arm_18.12.4.LTS/include" --include_path="P:/Courses/ENCE464/Other files/Blinker/Blink/FreeRTOS" --include_path="P:/Courses/ENCE464/Other files/Blinker/Blink/FreeRTOS/include" --include_path="P:/Courses/ENCE464/Other files/Blinker/Blink/FreeRTOS/portable/CCS/ARM_CM4F" --define=ccs="ccs" --define=TARGET_IS_TM4C123_RB2 --define=PART_TM4C123GH6PM -g --gcc --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --preproc_with_compile --preproc_dependency="OrbitOLED/$(basename $(<F)).d_raw" --obj_directory="OrbitOLED" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


