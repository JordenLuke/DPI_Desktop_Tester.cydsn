/*******************************************************************************
* File Name: HMC5883L_I2C.c
* Version 3.30
*
* Description:
*  This file provides the source code of APIs for the I2C component.
*  Actual protocol and operation code resides in the interrupt service routine
*  file.
*
* Note:
*
*******************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "HMC5883L_I2C_PVT.h"


/**********************************
*      System variables
**********************************/

uint8 HMC5883L_I2C_initVar = 0u;    /* Defines if component was initialized */

volatile uint8 HMC5883L_I2C_state;  /* Current state of I2C FSM */


/*******************************************************************************
* Function Name: HMC5883L_I2C_Init
********************************************************************************
*
* Summary:
*  Initializes I2C registers with initial values provided from customizer.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void HMC5883L_I2C_Init(void) 
{
    #if(HMC5883L_I2C_FF_IMPLEMENTED)
        HMC5883L_I2C_CFG_REG  = HMC5883L_I2C_DEFAULT_CFG;
        HMC5883L_I2C_XCFG_REG = HMC5883L_I2C_DEFAULT_XCFG;

        #if(CY_PSOC5A)
            HMC5883L_I2C_CLKDIV_REG  = LO8(HMC5883L_I2C_DEFAULT_DIVIDE_FACTOR);
        #else
            HMC5883L_I2C_CLKDIV1_REG = LO8(HMC5883L_I2C_DEFAULT_DIVIDE_FACTOR);
            HMC5883L_I2C_CLKDIV2_REG = HI8(HMC5883L_I2C_DEFAULT_DIVIDE_FACTOR);
        #endif /* (CY_PSOC5A) */

    #else
        uint8 enableInterrupts;

        HMC5883L_I2C_CFG_REG      = HMC5883L_I2C_DEFAULT_CFG;      /* control  */
        HMC5883L_I2C_INT_MASK_REG = HMC5883L_I2C_DEFAULT_INT_MASK; /* int_mask */

        /* Enable interrupts from block */
        enableInterrupts = CyEnterCriticalSection();
        HMC5883L_I2C_INT_ENABLE_REG |= HMC5883L_I2C_INTR_ENABLE; /* aux_ctl */
        CyExitCriticalSection(enableInterrupts);

        #if(HMC5883L_I2C_MODE_MASTER_ENABLED)
            HMC5883L_I2C_MCLK_PRD_REG = HMC5883L_I2C_DEFAULT_MCLK_PRD;
            HMC5883L_I2C_MCLK_CMP_REG = HMC5883L_I2C_DEFAULT_MCLK_CMP;
         #endif /* (HMC5883L_I2C_MODE_MASTER_ENABLED) */

        #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
            HMC5883L_I2C_PERIOD_REG = HMC5883L_I2C_DEFAULT_PERIOD;
        #endif  /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

    #endif /* (HMC5883L_I2C_FF_IMPLEMENTED) */

    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
        HMC5883L_I2C_TimeoutInit();
    #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */

    /* Disable Interrupt and set vector and priority */
    CyIntDisable    (HMC5883L_I2C_ISR_NUMBER);
    CyIntSetPriority(HMC5883L_I2C_ISR_NUMBER, HMC5883L_I2C_ISR_PRIORITY);
    #if(HMC5883L_I2C_INTERN_I2C_INTR_HANDLER)
        (void) CyIntSetVector(HMC5883L_I2C_ISR_NUMBER, &HMC5883L_I2C_ISR);
    #endif /* (HMC5883L_I2C_INTERN_I2C_INTR_HANDLER) */


    /* Put state machine in idle state */
    HMC5883L_I2C_state = HMC5883L_I2C_SM_IDLE;

    #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
        /* Reset status and buffers index */
        HMC5883L_I2C_SlaveClearReadBuf();
        HMC5883L_I2C_SlaveClearWriteBuf();
        HMC5883L_I2C_slStatus = 0u; /* Reset slave status */

        /* Set default address */
        HMC5883L_I2C_SlaveSetAddress(HMC5883L_I2C_DEFAULT_ADDR);
    #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

    #if(HMC5883L_I2C_MODE_MASTER_ENABLED)
        /* Reset status and buffers index */
        HMC5883L_I2C_MasterClearReadBuf();
        HMC5883L_I2C_MasterClearWriteBuf();
        (void) HMC5883L_I2C_MasterClearStatus();
    #endif /* (HMC5883L_I2C_MODE_MASTER_ENABLED) */
}


/*******************************************************************************
* Function Name: HMC5883L_I2C_Enable
********************************************************************************
*
* Summary:
*  Enables I2C operations.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  None
*
*******************************************************************************/
void HMC5883L_I2C_Enable(void) 
{
    #if(HMC5883L_I2C_FF_IMPLEMENTED)
        uint8 enableInterrupts;

        /* Enable power to I2C FF block */
        enableInterrupts = CyEnterCriticalSection();
        HMC5883L_I2C_ACT_PWRMGR_REG  |= HMC5883L_I2C_ACT_PWR_EN;
        HMC5883L_I2C_STBY_PWRMGR_REG |= HMC5883L_I2C_STBY_PWR_EN;
        CyExitCriticalSection(enableInterrupts);

    #else

        #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
            uint8 enableInterrupts;
        #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

        #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
            /* Enable slave bit counter */
            enableInterrupts = CyEnterCriticalSection();
            HMC5883L_I2C_COUNTER_AUX_CTL_REG |= HMC5883L_I2C_CNT7_ENABLE;   /* aux_ctl */
            CyExitCriticalSection(enableInterrupts);
        #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

        HMC5883L_I2C_CFG_REG |= HMC5883L_I2C_ENABLE_MS;

    #endif /* (HMC5883L_I2C_FF_IMPLEMENTED) */

    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
        HMC5883L_I2C_TimeoutEnable();
    #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: HMC5883L_I2C_Start
********************************************************************************
*
* Summary:
*  Starts the I2C hardware. Enables Active mode power template bits or clock
*  gating as appropriate. It is required to be executed before I2C bus
*  operation.
*  The I2C interrupt remains disabled after this function call.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  This component automatically enables it's interrupt.  If I2C is enabled
*  without the interrupt enabled, it could lock up the I2C bus.
*
* Global variables:
*  HMC5883L_I2C_initVar - used to check initial configuration, modified
*  on first function call.
*
* Reentrant:
*  No
*
*******************************************************************************/
void HMC5883L_I2C_Start(void) 
{
    /* Initialize I2C registers, reset I2C buffer index and clears status */
    if(0u == HMC5883L_I2C_initVar)
    {
        HMC5883L_I2C_Init();
        HMC5883L_I2C_initVar = 1u; /* Component initialized */
    }

    HMC5883L_I2C_Enable();
    HMC5883L_I2C_EnableInt();
}


/*******************************************************************************
* Function Name: HMC5883L_I2C_Stop
********************************************************************************
*
* Summary:
*  Disables I2C hardware and disables I2C interrupt. Disables Active mode power
*  template bits or clock gating as appropriate.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void HMC5883L_I2C_Stop(void) 
{
    #if((HMC5883L_I2C_FF_IMPLEMENTED)  || \
        (HMC5883L_I2C_UDB_IMPLEMENTED && HMC5883L_I2C_MODE_SLAVE_ENABLED))
        uint8 enableInterrupts;
    #endif /* ((HMC5883L_I2C_FF_IMPLEMENTED)  || \
               (HMC5883L_I2C_UDB_IMPLEMENTED && HMC5883L_I2C_MODE_SLAVE_ENABLED)) */

    HMC5883L_I2C_DisableInt();

    HMC5883L_I2C_DISABLE_INT_ON_STOP;   /* Interrupt on Stop can be enabled by write */
    (void) HMC5883L_I2C_CSR_REG;        /* Clear CSR reg */
    
    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
        HMC5883L_I2C_TimeoutStop();
    #endif  /* End (HMC5883L_I2C_TIMEOUT_ENABLED) */

    #if(HMC5883L_I2C_FF_IMPLEMENTED)
        #if(CY_PSOC3 || CY_PSOC5LP)
            /* Store registers which are held in reset when Master and Slave bits are cleared */
            #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
                HMC5883L_I2C_backup.addr = HMC5883L_I2C_ADDR_REG;
            #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

            HMC5883L_I2C_backup.clkDiv1  = HMC5883L_I2C_CLKDIV1_REG;
            HMC5883L_I2C_backup.clkDiv2  = HMC5883L_I2C_CLKDIV2_REG;


            /* Reset FF block */
            HMC5883L_I2C_CFG_REG &= ((uint8) ~HMC5883L_I2C_ENABLE_MS);
            CyDelayUs(HMC5883L_I2C_FF_RESET_DELAY);
            HMC5883L_I2C_CFG_REG |= ((uint8)  HMC5883L_I2C_ENABLE_MS);


            /* Restore registers */
            #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
                HMC5883L_I2C_ADDR_REG = HMC5883L_I2C_backup.addr;
            #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

            HMC5883L_I2C_CLKDIV1_REG = HMC5883L_I2C_backup.clkDiv1;
            HMC5883L_I2C_CLKDIV2_REG = HMC5883L_I2C_backup.clkDiv2;

        #endif /* (CY_PSOC3 || CY_PSOC5LP) */

        /* Disable power to I2C block */
        enableInterrupts = CyEnterCriticalSection();
        HMC5883L_I2C_ACT_PWRMGR_REG  &= ((uint8) ~HMC5883L_I2C_ACT_PWR_EN);
        HMC5883L_I2C_STBY_PWRMGR_REG &= ((uint8) ~HMC5883L_I2C_STBY_PWR_EN);
        CyExitCriticalSection(enableInterrupts);

    #else

        #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
            /* Disable slave bit counter */
            enableInterrupts = CyEnterCriticalSection();
            HMC5883L_I2C_COUNTER_AUX_CTL_REG &= ((uint8) ~HMC5883L_I2C_CNT7_ENABLE);
            CyExitCriticalSection(enableInterrupts);
        #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

        HMC5883L_I2C_CFG_REG &= ((uint8) ~HMC5883L_I2C_ENABLE_MS);

    #endif /* (HMC5883L_I2C_FF_IMPLEMENTED) */

    HMC5883L_I2C_ClearPendingInt();  /* Clear interrupt triggers on reset */

    HMC5883L_I2C_state = HMC5883L_I2C_SM_IDLE;  /* Reset software FSM */
}


/* [] END OF FILE */
