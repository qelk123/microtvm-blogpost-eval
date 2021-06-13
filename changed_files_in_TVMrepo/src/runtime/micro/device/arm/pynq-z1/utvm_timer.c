/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

/*!
 * \file utvm_timer.c
 * \brief uTVM timer API definitions for STM32F746XX-series boards
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "utvm_runtime.h"
// NOTE: This expects ST CMSIS to be in your include path.
// Download STM32CubeF7 here:
// https://www.st.com/content/st_com/en/products/embedded-software/mcu-mpu-embedded-software/stm32-embedded-software/stm32cube-mcu-mpu-packages/stm32cubef7.html
// and add Drivers/CMSIS to your C include path.
//#include "Device/ST/STM32F7xx/Include/stm32f746xx.h"
#include <stdio.h>
//#include "platform.h"
//#include "xil_printf.h"


#include "xparameters.h"
#include "xadcps.h"

#include "xil_types.h"

#include "xscugic.h"

#include "xil_exception.h"

#include "xscutimer.h"

//timer info//

#define TIMER_DEVICE_ID     XPAR_XSCUTIMER_0_DEVICE_ID

#define INTC_DEVICE_ID      XPAR_SCUGIC_SINGLE_DEVICE_ID

#define TIMER_IRPT_INTR     XPAR_SCUTIMER_INTR

//#define TIMER_LOAD_VALUE  0x0FFFFFFF

#define TIMER_LOAD_VALUE    0x0FFFFFFFF

#define myXSCUTIMER_SELFTEST_VALUE	0xA55AF00FU

//static XAdcPs  XADCMonInst; //XADC
XScuTimer_Config XScuTimer_ConfigTable[XPAR_XSCUTIMER_NUM_INSTANCES] =
{
	{
		XPAR_PS7_SCUTIMER_0_DEVICE_ID,
		XPAR_PS7_SCUTIMER_0_BASEADDR
	}
};
//static XScuGic Intc; //GIC

static XScuTimer Timer;//timer

//static void SetupInterruptSystem(XScuGic *GicInstancePtr,

        //XScuTimer *TimerInstancePtr, u16 TimerIntrId);

//static void TimerIntrHandler(void *CallBackRef);

//add

XScuTimer_Config *myXScuTimer_LookupConfig(u16 DeviceId)
{
	XScuTimer_Config *CfgPtr = NULL;
	u32 Index;

	for (Index = 0U; Index < XPAR_XSCUTIMER_NUM_INSTANCES; Index++) {
		if (XScuTimer_ConfigTable[Index].DeviceId == DeviceId) {
			CfgPtr = &XScuTimer_ConfigTable[Index];
			break;
		}
	}

	return (XScuTimer_Config *)CfgPtr;
}

s32 myXScuTimer_CfgInitialize(XScuTimer *InstancePtr,
			 XScuTimer_Config *ConfigPtr, u32 EffectiveAddress)
{
	s32 Status;
	//Xil_AssertNonvoid(InstancePtr != NULL);
	//Xil_AssertNonvoid(ConfigPtr != NULL);

	/*
	 * If the device is started, disallow the initialize and return a
	 * status indicating it is started. This allows the user to stop the
	 * device and reinitialize, but prevents a user from inadvertently
	 * initializing.
	 */
	if (InstancePtr->IsStarted != XIL_COMPONENT_IS_STARTED) {
		/*
		 * Copy configuration into the instance structure.
		 */
		InstancePtr->Config.DeviceId = ConfigPtr->DeviceId;

		/*
		 * Save the base address pointer such that the registers of the block
		 * can be accessed and indicate it has not been started yet.
		 */
		InstancePtr->Config.BaseAddr = EffectiveAddress;

		InstancePtr->IsStarted = (u32)0;

		/*
		 * Indicate the instance is ready to use, successfully initialized.
		 */
		InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

		Status =(s32)XST_SUCCESS;
	}
	else {
		Status = (s32)XST_DEVICE_IS_STARTED;
	}
	return Status;
}

s32 myXScuTimer_SelfTest(XScuTimer *InstancePtr)
{
	u32 Register;
	u32 CtrlOrig;
	u32 LoadOrig;
	s32 Status;

	/*
	 * Assert to ensure the inputs are valid and the instance has been
	 * initialized.
	 */
	//Xil_AssertNonvoid(InstancePtr != NULL);
	//Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * Save the contents of the Control Register and stop the timer.
	 */
	CtrlOrig = XScuTimer_ReadReg(InstancePtr->Config.BaseAddr,
				  XSCUTIMER_CONTROL_OFFSET);
	Register = CtrlOrig & (u32)(~XSCUTIMER_CONTROL_ENABLE_MASK);
	XScuTimer_WriteReg(InstancePtr->Config.BaseAddr,
			XSCUTIMER_CONTROL_OFFSET, Register);

	/*
	 * Save the contents of the Load Register.
	 * Load a new test value in the Load Register, read it back and
	 * compare it with the written value.
	 */
	LoadOrig = XScuTimer_ReadReg((InstancePtr)->Config.BaseAddr,
				  XSCUTIMER_LOAD_OFFSET);
	XScuTimer_LoadTimer(InstancePtr, myXSCUTIMER_SELFTEST_VALUE);
	Register = XScuTimer_ReadReg((InstancePtr)->Config.BaseAddr,
				  XSCUTIMER_LOAD_OFFSET);

	/*
	 * Restore the contents of the Load Register and Control Register.
	 */
	XScuTimer_LoadTimer(InstancePtr, LoadOrig);
	XScuTimer_WriteReg(InstancePtr->Config.BaseAddr,
			XSCUTIMER_CONTROL_OFFSET, CtrlOrig);

	/*
	 * Return a Failure if the contents of the Load Register do not
	 * match with the value written to it.
	 */
	if (Register != myXSCUTIMER_SELFTEST_VALUE) {
		Status = (s32)XST_FAILURE;
	}
	else {
		Status = (s32)XST_SUCCESS;
	}

	return Status;
}

void myXScuTimer_Start(XScuTimer *InstancePtr)
{
	u32 Register;

	//Xil_AssertVoid(InstancePtr != NULL);
	//Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * Read the contents of the Control register.
	 */
	Register = XScuTimer_ReadReg(InstancePtr->Config.BaseAddr,
				  XSCUTIMER_CONTROL_OFFSET);

	/*
	 * Set the 'timer enable' bit in the register.
	 */
	Register |= XSCUTIMER_CONTROL_ENABLE_MASK;

	/*
	 * Update the Control register with the new value.
	 */
	XScuTimer_WriteReg(InstancePtr->Config.BaseAddr,
			XSCUTIMER_CONTROL_OFFSET, Register);

	/*
	 * Indicate that the device is started.
	 */
	InstancePtr->IsStarted = XIL_COMPONENT_IS_STARTED;
}

void myXScuTimer_SetPrescaler(XScuTimer *InstancePtr, u8 PrescalerValue)
{
	u32 ControlReg;

	/*
	 * Assert to validate input arguments.
	 */
	//Xil_AssertVoid(InstancePtr != NULL);
	//Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	/*
	 * Read the Timer control register.
	 */
	ControlReg = XScuTimer_ReadReg(InstancePtr->Config.BaseAddr,
					XSCUTIMER_CONTROL_OFFSET);

	/*
	 * Clear all of the prescaler control bits in the register.
	 */
	ControlReg &= (u32)(~XSCUTIMER_CONTROL_PRESCALER_MASK);

	/*
	 * Set the prescaler value.
	 */
	ControlReg |= (((u32)PrescalerValue) << XSCUTIMER_CONTROL_PRESCALER_SHIFT);

	/*
	 * Write the register with the new values.
	 */
	XScuTimer_WriteReg(InstancePtr->Config.BaseAddr,
			  XSCUTIMER_CONTROL_OFFSET, ControlReg);
}

void myXScuTimer_Stop(XScuTimer *InstancePtr)
{
	u32 Register;

	//Xil_AssertVoid(InstancePtr != NULL);
	//Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * Read the contents of the Control register.
	 */
	Register = XScuTimer_ReadReg(InstancePtr->Config.BaseAddr,
				  XSCUTIMER_CONTROL_OFFSET);

	/*
	 * Clear the 'timer enable' bit in the register.
	 */
	Register &= (u32)(~XSCUTIMER_CONTROL_ENABLE_MASK);

	/*
	 * Update the Control register with the new value.
	 */
	XScuTimer_WriteReg(InstancePtr->Config.BaseAddr,
			XSCUTIMER_CONTROL_OFFSET, Register);

	/*
	 * Indicate that the device is stopped.
	 */
	InstancePtr->IsStarted = (u32)0;
}






//#define utvm_SystemCoreClock 216000000UL

int32_t UTVMTimerStart() {
  UTVMTimerReset();

  myXScuTimer_Start(&Timer);
  //TIM2->CR1 = TIM_CR1_CEN;  // Start counter
  return 0;//UTVM_ERR_OK;
}

uint32_t UTVMTimerStop(int32_t* err) {
  uint32_t val;
  val=(uint32_t)((0x0FFFFFFFF-XScuTimer_GetCounterValue(&Timer))/333);
  myXScuTimer_Stop(&Timer);
  //TIM2->CR1 &= TIM_CR1_CEN;
  //if (TIM2->SR & TIM_SR_UIF_Msk) {
    //*err = UTVM_ERR_TIMER_OVERFLOW;
    //return 0;
  //}
  //*err = UTVM_ERR_OK;
  //uint32_t tim_cnt = TIM2->CNT;
  //uint32_t millis = tim_cnt / (utvm_SystemCoreClock / 1000);
  //uint32_t micros =
      //(tim_cnt - (millis * (utvm_SystemCoreClock / 1000))) / (utvm_SystemCoreClock / 1000000);
  return val;//millis * 1000 + micros;
}

void UTVMTimerReset() {
     XScuTimer_Config *TMRConfigPtr;     //timer config

     //printf("------------START-------------\n");

   //  init_platform();

     //

     //私有定时器初始化

     TMRConfigPtr = myXScuTimer_LookupConfig(TIMER_DEVICE_ID);

     myXScuTimer_CfgInitialize(&Timer, TMRConfigPtr,TMRConfigPtr->BaseAddr);

     myXScuTimer_SelfTest(&Timer);

     myXScuTimer_SetPrescaler(&Timer,9);

     //加载计数周期，私有定时器的时钟为CPU的一般，为333MHZ,如果计数1S,加载值为1sx(333x1000x1000)(1/s)-1=0x13D92D3F

     XScuTimer_LoadTimer(&Timer, TIMER_LOAD_VALUE);

     //自动装载

     XScuTimer_EnableAutoReload(&Timer);
  //RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;                       // Hold TIM2 in reset
  //RCC->DCKCFGR1 = (RCC->DCKCFGR1 & ~RCC_DCKCFGR1_TIMPRE_Msk);  // disable 2x clock boost to TIM2
  //RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1_Msk);  // No AHB clock division to APB1 (1:1).
  //RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;             // Enable TIM2 clock.
  //RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;         // Exit TIM2 reset.

  //DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;  // stop TIM2 clock during debug halt.
  //TIM2->ARR = 0xffffffff;
  //if (TIM2->SR & TIM_SR_UIF_Msk) {
    //for (;;) {
    //}
  //}
}

#ifdef __cplusplus
}  // TVM_EXTERN_C
#endif
