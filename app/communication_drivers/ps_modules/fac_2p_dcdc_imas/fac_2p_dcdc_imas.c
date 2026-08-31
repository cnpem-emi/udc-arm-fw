/******************************************************************************
 * Copyright (C) 2020 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fac_2p_dcdc_imas.c
 * @brief FAC-2P DC/DC Stage module for IMAS
 *
 * Module for control of two DC/DC modules of FAC power supplies used by IMAS
 * group on magnets characterization tests. It implements the controller for
 * load current.
 *
 * @author gabriel.brunheira
 * @date 21/02/2020
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"

#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/adcp/adcp.h"
#include "communication_drivers/bsmp/bsmp_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/control/control.h"
#include "communication_drivers/control/wfmref/wfmref.h"
#include "communication_drivers/event_manager/event_manager.h"
#include "communication_drivers/iib/iib_data.h"
#include "communication_drivers/iib/iib_module.h"
#include "communication_drivers/ps_modules/fac_2p_dcdc_imas/fac_2p_dcdc_imas.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/// DSP Net Signals
#define I_LOAD                          g_controller_ctom.net_signals[0]   // HRADC0
#define I_ARM_1                         g_controller_ctom.net_signals[1]   // HRADC1
#define I_ARM_2                         g_controller_ctom.net_signals[2]   // HRADC2

#define I_LOAD_MEAN                     g_controller_ctom.net_signals[3]
#define I_LOAD_ERROR                    g_controller_ctom.net_signals[4]
#define DUTY_I_LOAD_PI                  g_controller_ctom.net_signals[5]

#define I_ARMS_DIFF                     g_controller_ctom.net_signals[6]
#define DUTY_ARMS_DIFF                  g_controller_ctom.net_signals[7]

#define I_LOAD_DIFF                     g_controller_ctom.net_signals[8]

#define DUTY_REF_FF                     g_controller_ctom.net_signals[9]

#define V_CAPBANK_MOD_1_FILTERED        g_controller_ctom.net_signals[10]
#define V_CAPBANK_MOD_2_FILTERED        g_controller_ctom.net_signals[11]

#define IN_FF_V_CAPBANK_MOD_1           g_controller_ctom.net_signals[12]
#define IN_FF_V_CAPBANK_MOD_2           g_controller_ctom.net_signals[13]

#define WFMREF_IDX                      g_controller_ctom.net_signals[31]

#define DUTY_CYCLE_MOD_1                g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_MOD_2                g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define V_CAPBANK_MOD_1                 g_controller_mtoc.net_signals[0]
#define V_CAPBANK_MOD_2                 g_controller_mtoc.net_signals[1]

#define V_OUT_OS_MOD_1                  g_controller_mtoc.net_signals[2]
#define V_OUT_OS_MOD_2                  g_controller_mtoc.net_signals[3]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Module_1_CapBank_Overvoltage,
    Module_2_CapBank_Overvoltage,
    Module_1_CapBank_Undervoltage,
    Module_2_CapBank_Undervoltage,
	Module_1_Vout_Overvoltage,
	Module_2_Vout_Overvoltage,
	Module_1_Vout_Undervoltage,
	Module_2_Vout_Undervoltage,
    IIB_Mod_1_Itlk,
    IIB_Mod_2_Itlk,
	IDB_Master_Itlk,
	IDB_Slave_Itlk,
    External_Interlock
} hard_interlocks_t;

typedef enum
{
    DCCT_Fault,
    Load_Feedback_Fault,
    ARM_1_Overcurrent,
    ARM_2_Overcurrent,
    Arms_High_Difference
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

static volatile iib_fac_os_t iib_fac_2p_dcdc[2];

static void init_iib_modules();

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAC-2P DC/DC operation.
*/
static void adcp_channel_config(void)
{
    // Module 1 CapBank Voltage: 10 V = 300 V
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 300.0/2048.0;
    g_analog_ch_0.Value = &(V_CAPBANK_MOD_1.f);

    // Module 1 OS Output Voltage: 10 V = 300 V
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 300.0/2048.0;
    g_analog_ch_1.Value = &(V_OUT_OS_MOD_1.f);

    // Module 2 CapBank Voltage: 10 V = 300 V
    g_analog_ch_2.Enable = 1;
    g_analog_ch_2.Gain = 300.0/2048.0;
    g_analog_ch_2.Value = &(V_CAPBANK_MOD_2.f);

    // Module 2 OS Output Voltage: 10 V = 300 V
    g_analog_ch_3.Enable = 1;
    g_analog_ch_3.Gain = 300.0/2048.0;
    g_analog_ch_3.Value = &(V_OUT_OS_MOD_2.f);

    g_analog_ch_4.Enable = 0;
    g_analog_ch_5.Enable = 0;
    g_analog_ch_6.Enable = 0;
    g_analog_ch_7.Enable = 0;
}

/**
* @brief Initialize BSMP servers.
*
* Setup BSMP servers for FBP operation.
*
*/
static void bsmp_init_server(void)
{
    create_bsmp_var(31, 0, 4, false, g_ipc_ctom.ps_module[0].ps_soft_interlock.u8);
    create_bsmp_var(32, 0, 4, false, g_ipc_ctom.ps_module[0].ps_hard_interlock.u8);

    create_bsmp_var(33, 0, 4, false, I_LOAD_MEAN.u8);
    create_bsmp_var(34, 0, 4, false, I_LOAD.u8);

    create_bsmp_var(35, 0, 4, false, I_ARM_1.u8);
    create_bsmp_var(36, 0, 4, false, I_ARM_2.u8);

    create_bsmp_var(37, 0, 4, false, V_CAPBANK_MOD_1.u8);
    create_bsmp_var(38, 0, 4, false, V_CAPBANK_MOD_2.u8);

    create_bsmp_var(39, 0, 4, false, V_OUT_OS_MOD_1.u8);
    create_bsmp_var(40, 0, 4, false, V_OUT_OS_MOD_2.u8);

    create_bsmp_var(41, 0, 4, false, DUTY_CYCLE_MOD_1.u8);
    create_bsmp_var(42, 0, 4, false, DUTY_CYCLE_MOD_2.u8);

    create_bsmp_var(43, 0, 4, false, iib_fac_2p_dcdc[0].VdcLink.u8);
    create_bsmp_var(44, 0, 4, false, iib_fac_2p_dcdc[0].Iin.u8);
    create_bsmp_var(45, 0, 4, false, iib_fac_2p_dcdc[0].Iout.u8);
    create_bsmp_var(46, 0, 4, false, iib_fac_2p_dcdc[0].TempIGBT1.u8);
    create_bsmp_var(47, 0, 4, false, iib_fac_2p_dcdc[0].TempIGBT2.u8);
    create_bsmp_var(48, 0, 4, false, iib_fac_2p_dcdc[0].TempL.u8);
    create_bsmp_var(49, 0, 4, false, iib_fac_2p_dcdc[0].TempHeatSink.u8);
    create_bsmp_var(50, 0, 4, false, iib_fac_2p_dcdc[0].DriverVoltage.u8);
    create_bsmp_var(51, 0, 4, false, iib_fac_2p_dcdc[0].Driver1Current.u8);
    create_bsmp_var(52, 0, 4, false, iib_fac_2p_dcdc[0].Driver2Current.u8);
    create_bsmp_var(53, 0, 4, false, iib_fac_2p_dcdc[0].BoardTemperature.u8);
    create_bsmp_var(54, 0, 4, false, iib_fac_2p_dcdc[0].RelativeHumidity.u8);
    create_bsmp_var(55, 0, 4, false, iib_fac_2p_dcdc[0].InterlocksRegister.u8);
    create_bsmp_var(56, 0, 4, false, iib_fac_2p_dcdc[0].AlarmsRegister.u8);

    create_bsmp_var(57, 0, 4, false, iib_fac_2p_dcdc[1].VdcLink.u8);
    create_bsmp_var(58, 0, 4, false, iib_fac_2p_dcdc[1].Iin.u8);
    create_bsmp_var(59, 0, 4, false, iib_fac_2p_dcdc[1].Iout.u8);
    create_bsmp_var(60, 0, 4, false, iib_fac_2p_dcdc[1].TempIGBT1.u8);
    create_bsmp_var(61, 0, 4, false, iib_fac_2p_dcdc[1].TempIGBT2.u8);
    create_bsmp_var(62, 0, 4, false, iib_fac_2p_dcdc[1].TempL.u8);
    create_bsmp_var(63, 0, 4, false, iib_fac_2p_dcdc[1].TempHeatSink.u8);
    create_bsmp_var(64, 0, 4, false, iib_fac_2p_dcdc[1].DriverVoltage.u8);
    create_bsmp_var(65, 0, 4, false, iib_fac_2p_dcdc[1].Driver1Current.u8);
    create_bsmp_var(66, 0, 4, false, iib_fac_2p_dcdc[1].Driver2Current.u8);
    create_bsmp_var(67, 0, 4, false, iib_fac_2p_dcdc[1].BoardTemperature.u8);
    create_bsmp_var(68, 0, 4, false, iib_fac_2p_dcdc[1].RelativeHumidity.u8);
    create_bsmp_var(69, 0, 4, false, iib_fac_2p_dcdc[1].InterlocksRegister.u8);
    create_bsmp_var(70, 0, 4, false, iib_fac_2p_dcdc[1].AlarmsRegister.u8);

    create_bsmp_var(71, 0, 4, false, g_ipc_ctom.ps_module[0].ps_alarms.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fac_2p_dcdc_imas_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib_modules();

    init_wfmref(&WFMREF[0], WFMREF_SELECTED_PARAM[0].u16,
                WFMREF_SYNC_MODE_PARAM[0].u16, ISR_CONTROL_FREQ.f,
                WFMREF_FREQUENCY_PARAM[0].f, WFMREF_GAIN_PARAM[0].f,
                WFMREF_OFFSET_PARAM[0].f, &g_wfmref_data.data[0][0].f,
                SIZE_WFMREF, &g_ipc_ctom.ps_module[0].ps_reference.f);

    init_scope(&g_ipc_mtoc.scope[0], ISR_CONTROL_FREQ.f,
               SCOPE_FREQ_SAMPLING_PARAM[0].f, &(g_buf_samples_ctom[0].f),
               SIZE_BUF_SAMPLES_CTOM, SCOPE_SOURCE_PARAM[0].p_f,
               (void *) 0);
}

static void init_iib_modules()
{
    iib_fac_2p_dcdc[0].CanAddress = 1;
    iib_fac_2p_dcdc[1].CanAddress = 2;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
}

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id)
{
    volatile uint8_t module;

    volatile unsigned long can_module = id;
    volatile unsigned long id_var = 0;

    switch(can_module)
    {
    	case 10:
    	case 11:
    	case 12:
    	case 13:
    	case 14:
    	case 15:
    	case 16:
    	{
    		module = 0;
    		id_var = (id - 10);
    		break;
    	}
    	case 20:
    	case 21:
    	case 22:
    	case 23:
    	case 24:
    	case 25:
    	case 26:
    	{
    		module = 1;
    		id_var = (id - 20);
    		break;
    	}

    	default:
    	{
    		break;
    	}
    }

    switch(id_var)
    {
        case 0:
        {
            memcpy((void *)iib_fac_2p_dcdc[module].VdcLink.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fac_2p_dcdc[module].DriverVoltage.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 1:
        {
            memcpy((void *)iib_fac_2p_dcdc[module].Iin.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fac_2p_dcdc[module].Iout.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 2:
        {
        	memcpy((void *)iib_fac_2p_dcdc[module].Driver1Current.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fac_2p_dcdc[module].Driver2Current.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 3:
        {
            memcpy((void *)iib_fac_2p_dcdc[module].TempIGBT1.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fac_2p_dcdc[module].TempIGBT2.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 4:
        {
        	memcpy((void *)iib_fac_2p_dcdc[module].TempL.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fac_2p_dcdc[module].TempHeatSink.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 5:
        {
        	memcpy((void *)iib_fac_2p_dcdc[module].BoardTemperature.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fac_2p_dcdc[module].RelativeHumidity.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 6:
        {
        	memcpy((void *)iib_fac_2p_dcdc[module].InterlocksRegister.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fac_2p_dcdc[module].AlarmsRegister.u8, (const void *)&data[4], (size_t)4);

        	if(iib_fac_2p_dcdc[module].InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Mod_1_Itlk + module);
        	}

        	else
        	{
        		iib_fac_2p_dcdc[module].InterlocksRegister.u32 = 0;
        	}

            break;
        }
        default:
        {
            break;
        }
    }
}

