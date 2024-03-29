/******************************************************************************
 * Copyright (C) 2018 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file fap.c
 * @brief FAP module
 *
 * Module for control of FAP power supplies. It implements the controller for
 * load current.
 *
 * @author gabriel.brunheira
 * @date 09/08/2018
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
#include "communication_drivers/ps_modules/fap/fap.h"
#include "communication_drivers/ps_modules/ps_modules.h"

/**
 * Controller defines
 */

/// DSP Net Signals
#define I_LOAD_1                g_controller_ctom.net_signals[0]    // HRADC0
#define I_LOAD_2                g_controller_ctom.net_signals[1]    // HRADC1
#define V_DCLINK                g_controller_ctom.net_signals[2]    // HRADC2

#define I_LOAD_MEAN             g_controller_ctom.net_signals[3]
#define I_LOAD_ERROR            g_controller_ctom.net_signals[4]
#define I_LOAD_DIFF             g_controller_ctom.net_signals[16]

#define I_IGBTS_DIFF            g_controller_ctom.net_signals[6]

#define DUTY_MEAN               g_controller_ctom.net_signals[5]
#define DUTY_DIFF               g_controller_ctom.net_signals[7]

#define DUTY_CYCLE_IGBT_1       g_controller_ctom.output_signals[0]
#define DUTY_CYCLE_IGBT_2       g_controller_ctom.output_signals[1]

/// ARM Net Signals
#define I_IGBT_1                g_controller_mtoc.net_signals[0]    // ANI0
#define I_IGBT_2                g_controller_mtoc.net_signals[1]    // ANI1

#define V_LOAD                  g_controller_mtoc.net_signals[2]

/**
 * Interlocks defines
 */
typedef enum
{
    Load_Overcurrent,
    Load_Overvoltage,
    DCLink_Overvoltage,
    DCLink_Undervoltage,
    Welded_Contactor_Fault,
    Opened_Contactor_Fault,
    IGBT_1_Overcurrent,
    IGBT_2_Overcurrent,
    IIB_Itlk
} hard_interlocks_t;

typedef enum
{
    DCCT_1_Fault,
    DCCT_2_Fault,
    DCCT_High_Difference,
    Load_Feedback_1_Fault,
    Load_Feedback_2_Fault,
    IGBTs_Current_High_Difference,
} soft_interlocks_t;

typedef enum
{
    High_Sync_Input_Frequency = 0x00000001
} alarms_t;

static volatile iib_fap_module_t iib_fap;

static void init_iib();

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id);

/**
* @brief Initialize ADCP Channels.
*
* Setup ADCP specific parameters for FAP operation.
*
*/
static void adcp_channel_config(void)
{
    // IGBT 1 current: 10 V = 200 A
    g_analog_ch_0.Enable = 1;
    g_analog_ch_0.Gain = 200.0/2048.0;
    g_analog_ch_0.Value = &(I_IGBT_1.f);

    // IGBT 2 current: 10 V = 200 A
    g_analog_ch_1.Enable = 1;
    g_analog_ch_1.Gain = 200.0/2048.0;
    g_analog_ch_1.Value = &(I_IGBT_2.f);

    g_analog_ch_2.Enable = 0;
    g_analog_ch_3.Enable = 0;
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
    create_bsmp_var(34, 0, 4, false, I_LOAD_1.u8);
    create_bsmp_var(35, 0, 4, false, I_LOAD_2.u8);

    create_bsmp_var(36, 0, 4, false, V_DCLINK.u8);
    create_bsmp_var(37, 0, 4, false, I_IGBT_1.u8);
    create_bsmp_var(38, 0, 4, false, I_IGBT_2.u8);

    create_bsmp_var(39, 0, 4, false, DUTY_CYCLE_IGBT_1.u8);
    create_bsmp_var(40, 0, 4, false, DUTY_CYCLE_IGBT_2.u8);
    create_bsmp_var(41, 0, 4, false, DUTY_DIFF.u8);

    create_bsmp_var(42, 0, 4, false, iib_fap.Vin.u8);
    create_bsmp_var(43, 0, 4, false, iib_fap.Vout.u8);
    create_bsmp_var(44, 0, 4, false, iib_fap.IoutA1.u8);
    create_bsmp_var(45, 0, 4, false, iib_fap.IoutA2.u8);
    create_bsmp_var(46, 0, 4, false, iib_fap.TempIGBT1.u8);
    create_bsmp_var(47, 0, 4, false, iib_fap.TempIGBT2.u8);
    create_bsmp_var(48, 0, 4, false, iib_fap.DriverVoltage.u8);
    create_bsmp_var(49, 0, 4, false, iib_fap.Driver1Current.u8);
    create_bsmp_var(50, 0, 4, false, iib_fap.Driver2Current.u8);
    create_bsmp_var(51, 0, 4, false, iib_fap.TempL.u8);
    create_bsmp_var(52, 0, 4, false, iib_fap.TempHeatSink.u8);
    create_bsmp_var(53, 0, 4, false, iib_fap.GroundLeakage.u8);
    create_bsmp_var(54, 0, 4, false, iib_fap.BoardTemperature.u8);
    create_bsmp_var(55, 0, 4, false, iib_fap.RelativeHumidity.u8);
    create_bsmp_var(56, 0, 4, false, iib_fap.InterlocksRegister.u8);
    create_bsmp_var(57, 0, 4, false, iib_fap.AlarmsRegister.u8);

    create_bsmp_var(58, 0, 4, false, g_ipc_ctom.ps_module[0].ps_alarms.u8);
}

/**
* @brief System configuration for FBP.
*
* Initialize specific parameters e configure peripherals for FBP operation.
*
*/
void fap_system_config()
{
    adcp_channel_config();
    bsmp_init_server();
    init_iib();

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

static void init_iib()
{
    iib_fap.CanAddress = 1;

    init_iib_module_can_data(&g_iib_module_can_data, &handle_can_data);
}

static void handle_can_data(volatile uint8_t *data, volatile unsigned long id)
{
    switch(id)
    {
        case 10:
        {
            memcpy((void *)iib_fap.Vin.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fap.Vout.u8, (const void *)&data[4], (size_t)4);
            V_LOAD.f = iib_fap.Vout.f;

            break;
        }
        case 11:
        {
            memcpy((void *)iib_fap.IoutA1.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fap.IoutA2.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 12:
        {
        	memcpy((void *)iib_fap.DriverVoltage.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap.GroundLeakage.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 13:
        {
        	memcpy((void *)iib_fap.Driver1Current.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap.Driver2Current.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 14:
        {
            memcpy((void *)iib_fap.TempIGBT1.u8, (const void *)&data[0], (size_t)4);
            memcpy((void *)iib_fap.TempIGBT2.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 15:
        {
        	memcpy((void *)iib_fap.TempL.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap.TempHeatSink.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 16:
        {
        	memcpy((void *)iib_fap.BoardTemperature.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap.RelativeHumidity.u8, (const void *)&data[4], (size_t)4);

            break;
        }
        case 17:
        {
        	memcpy((void *)iib_fap.InterlocksRegister.u8, (const void *)&data[0], (size_t)4);
        	memcpy((void *)iib_fap.AlarmsRegister.u8, (const void *)&data[4], (size_t)4);

            if(iib_fap.InterlocksRegister.u32 > 0)
        	{
        		set_hard_interlock(0, IIB_Itlk);
        	}

            else
        	{
        		iib_fap.InterlocksRegister.u32 = 0;
        	}

        	break;
        }
        default:
        {
            break;
        }
    }
}
