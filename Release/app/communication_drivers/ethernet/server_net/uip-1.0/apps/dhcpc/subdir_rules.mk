################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
app/communication_drivers/ethernet/server_net/uip-1.0/apps/dhcpc/dhcpc.obj: ../app/communication_drivers/ethernet/server_net/uip-1.0/apps/dhcpc/dhcpc.c $(GEN_OPTS) $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: ARM Compiler'
	"C:/ti/ccsv6/tools/compiler/arm_5.1.11/bin/armcl" --code_state=32 --abi=eabi -me -O2 --include_path="C:/ti/ccsv6/tools/compiler/arm_5.1.11/include" --display_error_number --diag_warning=225 --diag_wrap=off --preproc_with_compile --preproc_dependency="app/communication_drivers/ethernet/server_net/uip-1.0/apps/dhcpc/dhcpc.pp" --obj_directory="app/communication_drivers/ethernet/server_net/uip-1.0/apps/dhcpc" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: $<'
	@echo ' '

