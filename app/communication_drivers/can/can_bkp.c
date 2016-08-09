/*
 * can_bkp.c
 *
 *  Created on: 21/01/2016
 *      Author: joao.rosa
 */

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "../shared_memory/structs.h"

#include "set_pinout_udc_v2.0.h"
//#include "set_pinout_ctrl_card.h"

#include "can_bkp.h"

#include "../ipc/ipc_lib.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

//! \addtogroup can_examples_list
//! <h1>Multiple CAN RX (multi_rx)</h1>
//!
//! This example shows how to set up the CAN to receive multiple CAN messages
//! using separate message objects for different messages, and using CAN ID
//! filtering to control which messages are received.  Three message objects
//! are set up to receive 3 of the 4 CAN message IDs that are used by the
//! multi_tx example.  Filtering is used to demonstrate how to receive only
//! specific messages, and therefore not receiving all 4 messages from the
//! multi_tx example.
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - INT_CAN0 - CANIntHandler
//
//*****************************************************************************

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag1 = 0;
volatile bool g_bRXFlag2 = 0;
volatile bool g_bRXFlag3 = 0;
volatile bool g_bRXFlag4 = 0;
volatile bool g_bRXFlag5 = 0;
volatile bool g_bRXFlag6 = 0;
volatile bool g_bRXFlag7 = 0;
volatile bool g_bRXFlag8 = 0;
volatile bool g_bRXFlag9 = 0;


//Interlock
volatile bool ItlkOld1 = 0;
volatile bool ItlkOld2 = 0;

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;

//Rx
tCANMsgObject sCANMessage;
uint8_t pui8MsgData[8];

//Tx
tCANMsgObject sCANMessageTx;
uint8_t pui8MsgDataTx[8];

uint8_t Counter = 0;


// Partir o float em bytes
static union
{
   float f;
   char c[4];
} floatNchars;

typedef struct
{
	float Iout1;
	float Iout2;
	float Vin;
	float Vout;
	uint8_t TempChv1;
	uint8_t TempChv2;
	uint8_t TempL1;
	uint8_t TempL2;
	uint8_t DvrVolt;
	uint8_t DvrCurr;
	uint8_t RH;
	uint8_t ErrAl;
} Q1Module_t;

typedef struct
{
	float Iout;
	float Vin;
	float Vout;
	uint8_t TempChv1;
	uint8_t TempChv2;
	uint8_t TempL1;
	uint8_t TempL2;
	uint8_t DvrVolt;
	uint8_t DvrCurr;
	uint8_t RH;
} Q4Module_t;


Q1Module_t Mod1Q1;
Q1Module_t Mod2Q1;


//*****************************************************************************
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//*****************************************************************************
void
CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
		// Set flag to indicate received message is pending for this message
		// object.
		//
		g_bRXFlag1 = 1;


        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32MsgCount++;



        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 2.
    //
    else if(ui32Status == 2)
    {
        CANIntClear(CAN0_BASE, 2);

        g_bRXFlag2 = 1;

        g_ui32MsgCount++;
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 3.
    //
    else if(ui32Status == 3)
    {
        CANIntClear(CAN0_BASE, 3);

        g_bRXFlag3 = 1;

        g_ui32MsgCount++;
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 4.
    //
    else if(ui32Status == 4)
    {

        CANIntClear(CAN0_BASE, 4);

        g_bRXFlag4 = 1;


        g_ui32MsgCount++;
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 5.
    //
    else if(ui32Status == 5)
    {
        CANIntClear(CAN0_BASE, 5);

        g_bRXFlag5 = 1;

        g_ui32MsgCount++;
        g_bErrFlag = 0;
    }

    //
    // Check if the cause is message object 6.
    //
    else if(ui32Status == 6)
    {
        CANIntClear(CAN0_BASE, 6);

        g_bRXFlag6 = 1;

        g_ui32MsgCount++;
        g_bErrFlag = 0;
    }

    //
	// Check if the cause is message object 7.
	//
	else if(ui32Status == 7)
	{
		CANIntClear(CAN0_BASE, 7);

		g_bErrFlag = 0;
	}

    //
	// Check if the cause is message object 8.
	//
	else if(ui32Status == 8)
	{
		CANIntClear(CAN0_BASE, 8);

		g_bRXFlag8 = 1;

		g_ui32MsgCount++;
		g_bErrFlag = 0;
	}

    //
	// Check if the cause is message object 9.
	//
	else if(ui32Status == 9)
	{
		CANIntClear(CAN0_BASE, 9);

		g_bRXFlag9 = 1;

		g_ui32MsgCount++;
		g_bErrFlag = 0;
	}

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

void
CanCheck(void)
{
    //
    // If the flag for message object 1 is set, that means that the RX
    // interrupt occurred and there is a message ready to be read from
    // this CAN message object.
    //
    if(g_bRXFlag1)
    {


    	sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod1Q1.Iout1 = floatNchars.f; 						//  I Bra�o 1
		DP_Framework_MtoC.NetSignals[2].f = floatNchars.f;


		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod1Q1.Iout2 = floatNchars.f; 						//  I Bra�o 2
		DP_Framework_MtoC.NetSignals[3].f = floatNchars.f;

        //
        // Clear the pending message flag so that the interrupt handler can
        // set it again when the next message arrives.
        //
        g_bRXFlag1 = 0;

        Counter++;

        //
        // Print information about the message just received.
        //

    }

    //
    // Check for message received on message object 2.  If so then
    // read message and print information.
    //
    if(g_bRXFlag2)
    {

    	sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 2, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod1Q1.Vin = floatNchars.f; //  Vin
		DP_Framework_MtoC.NetSignals[5].f = floatNchars.f;

		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod1Q1.Vout = floatNchars.f; //  Vout
		DP_Framework_MtoC.NetSignals[9].f = floatNchars.f;

        g_bRXFlag2 = 0;
        Counter++;
    }

    //
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag3)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 3, &sCANMessage, 0);

		Mod1Q1.TempChv1 = pui8MsgData[0];
		Mod1Q1.TempChv2 = pui8MsgData[1];
		Mod1Q1.TempL1 = pui8MsgData[2];
		Mod1Q1.TempL2 = pui8MsgData[3];
		Mod1Q1.DvrVolt = pui8MsgData[4];
		Mod1Q1.DvrCurr = pui8MsgData[5];
		Mod1Q1.RH = pui8MsgData[6];
		//Mod1Q1.ErrAl = pui8MsgData[7];

		g_bRXFlag3 = 0;
		Counter++;
	}

	//

	//
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag4)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 4, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod2Q1.Iout1 = floatNchars.f; //  I Bra�o 1
		//DP_Framework_MtoC.NetSignals[2].f = floatNchars.f;

		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod2Q1.Iout2 = floatNchars.f; //  I Bra�o 2
		//DP_Framework_MtoC.NetSignals[3].f = floatNchars.f;

		g_bRXFlag4 = 0;
		Counter++;
	}

    //
    // Check for message received on message object 3.  If so then
    // read message and print information.
    //
    if(g_bRXFlag5)
    {

    	sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 5, &sCANMessage, 0);

		floatNchars.c[0] = pui8MsgData[0];
		floatNchars.c[1] = pui8MsgData[1];
		floatNchars.c[2] = pui8MsgData[2];
		floatNchars.c[3] = pui8MsgData[3];
		Mod2Q1.Vin = floatNchars.f; //  Vin
		DP_Framework_MtoC.NetSignals[6].f = floatNchars.f;

		floatNchars.c[0] = pui8MsgData[4];
		floatNchars.c[1] = pui8MsgData[5];
		floatNchars.c[2] = pui8MsgData[6];
		floatNchars.c[3] = pui8MsgData[7];
		Mod2Q1.Vout = floatNchars.f; //  Vout
		DP_Framework_MtoC.NetSignals[10].f = floatNchars.f;

        g_bRXFlag5 = 0;
    }

    //
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag6)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 6, &sCANMessage, 0);

		Mod2Q1.TempChv1 = pui8MsgData[0];
		Mod2Q1.TempChv2 = pui8MsgData[1];
		Mod2Q1.TempL1 = pui8MsgData[2];
		Mod2Q1.TempL2 = pui8MsgData[3];
		Mod2Q1.DvrVolt = pui8MsgData[4];
		Mod2Q1.DvrCurr = pui8MsgData[5];
		Mod2Q1.RH = pui8MsgData[6];
		//Mod2Q1.ErrAl = pui8MsgData[7];

		g_bRXFlag6 = 0;
		Counter++;
	}

	//
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag8)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 8, &sCANMessage, 0);

		Mod1Q1.ErrAl = pui8MsgData[0];

		if(Mod1Q1.ErrAl)
		{
			//ItlkOld1 = Mod1Q1.ErrAl;

			IPC_MtoC_Msg.PSModule.HardInterlocks.u32 = Mod1Q1.ErrAl;
			SendIpcFlag(HARD_INTERLOCK);
		}


		g_bRXFlag8 = 0;
		Counter++;
	}

	//
	// Check for message received on message object 2.  If so then
	// read message and print information.
	//
	if(g_bRXFlag9)
	{

		sCANMessage.pucMsgData = pui8MsgData;

		CANMessageGet(CAN0_BASE, 9, &sCANMessage, 0);

		Mod2Q1.ErrAl = pui8MsgData[0];

		if(Mod2Q1.ErrAl)
		{
			//ItlkOld2 = Mod2Q1.ErrAl;

			IPC_MtoC_Msg.PSModule.HardInterlocks.u32 = Mod2Q1.ErrAl;
			SendIpcFlag(HARD_INTERLOCK);
		}


		g_bRXFlag9 = 0;
		Counter++;
	}
}

void SendCanMessage(unsigned char CanMess)
{
	switch(CanMess)
	{
	case 0:

		break;
	case 255:

		sCANMessageTx.ulMsgID = 0x200;

		break;

	}

	CANMessageSet(CAN0_BASE, 7, &sCANMessageTx, MSG_OBJ_TYPE_TX);
}


void
InitCanBkp(void)
{
    // Initialize the CAN controller
    CANInit(CAN0_BASE);

    // Setup CAN to be clocked off the M3/Master subsystem clock
    CANClkSourceSelect(CAN0_BASE, CAN_CLK_M3);

    // Configure the controller for 1 Mbit operation.
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 1000000);

    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Register interrupt handler in RAM vector table
    IntRegister(INT_CAN0INT0, CANIntHandler);

    // Enable the CAN interrupt on the processor (NVIC).
    IntEnable(INT_CAN0INT0);
    //IntMasterEnable();

    // Enable test mode and select external loopback
    //HWREG(CAN0_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
    //HWREG(CAN0_BASE + CAN_O_TEST) = CAN_TEST_EXL;

    // Enable the CAN for operation.
    CANEnable(CAN0_BASE);

    //
	// Initialize message object 1 to be able to send CAN message 1.  This
	// message object is not shared so it only needs to be initialized one
	// time, and can be used for repeatedly sending the same message ID.
	//
    sCANMessageTx.ulMsgID = 0x200;
    sCANMessageTx.ulMsgIDMask = 0;
    sCANMessageTx.ulFlags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessageTx.ulMsgLen = 8;
    sCANMessageTx.pucMsgData = pui8MsgDataTx;


    //
    // Initialize a message object to receive CAN messages with ID 0x010.
    // The expected ID must be set along with the mask to indicate that all
    // bits in the ID must match.
    //
    sCANMessage.ulMsgID = 0x010;
    sCANMessage.ulMsgIDMask = 0x7FF;
    sCANMessage.ulFlags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER);
    sCANMessage.ulMsgLen = 8;

    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
    // Change the ID to 0x011, and load into message object 2 which will be
    // used for receiving any CAN messages with this ID.  Since only the CAN
    // ID field changes, we don't need to reload all the other fields.
    //
    sCANMessage.ulMsgID = 0x011;
    CANMessageSet(CAN0_BASE, 2, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
    // Change the ID to 0x012, and load into message object 3 which will be
    // used for receiving any CAN messages with this ID.  Since only the CAN
    // ID field changes, we don't need to reload all the other fields.
    //
    sCANMessage.ulMsgID = 0x012;
    CANMessageSet(CAN0_BASE, 3, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
	// Change the ID to 0x020, and load into message object 3 which will be
	// used for receiving any CAN messages with this ID.  Since only the CAN
	// ID field changes, we don't need to reload all the other fields.
	//
	sCANMessage.ulMsgID = 0x020;
	CANMessageSet(CAN0_BASE, 4, &sCANMessage, MSG_OBJ_TYPE_RX);

	//
	// Change the ID to 0x050, and load into message object 3 which will be
	// used for receiving any CAN messages with this ID.  Since only the CAN
	// ID field changes, we don't need to reload all the other fields.
	//
	sCANMessage.ulMsgID = 0x021;
	CANMessageSet(CAN0_BASE, 5, &sCANMessage, MSG_OBJ_TYPE_RX);

	//
	// Change the ID to 0x060, and load into message object 3 which will be
	// used for receiving any CAN messages with this ID.  Since only the CAN
	// ID field changes, we don't need to reload all the other fields.
	//
	sCANMessage.ulMsgID = 0x022;
	CANMessageSet(CAN0_BASE, 6, &sCANMessage, MSG_OBJ_TYPE_RX);

	sCANMessage.ulMsgID = 0x01F;
	CANMessageSet(CAN0_BASE, 8, &sCANMessage, MSG_OBJ_TYPE_RX);

	sCANMessage.ulMsgID = 0x02F;
	CANMessageSet(CAN0_BASE, 9, &sCANMessage, MSG_OBJ_TYPE_RX);

}