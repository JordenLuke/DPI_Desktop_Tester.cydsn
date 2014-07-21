/*******************************************************************************
* File Name: HMC5883L_I2C_PM.c
* Version 3.30
*
* Description:
*  This file provides Low power mode APIs for I2C component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "HMC5883L_I2C_PVT.h"

HMC5883L_I2C_BACKUP_STRUCT HMC5883L_I2C_backup =
{
    HMC5883L_I2C_DISABLE, /* enableState */

    #if(HMC5883L_I2C_FF_IMPLEMENTED)
        HMC5883L_I2C_DEFAULT_XCFG,  /* xcfg */
        HMC5883L_I2C_DEFAULT_CFG,   /* cfg  */

        #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
            HMC5883L_I2C_DEFAULT_ADDR, /* addr */
        #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            LO8(HMC5883L_I2C_DEFAULT_DIVIDE_FACTOR),  /* div */
        #else
            LO8(HMC5883L_I2C_DEFAULT_DIVIDE_FACTOR), /* div1 */
            HI8(HMC5883L_I2C_DEFAULT_DIVIDE_FACTOR), /* div2 */
        #endif /* (CY_PSOC5A) */

    #else  /* (HMC5883L_I2C_UDB_IMPLEMENTED) */
        HMC5883L_I2C_DEFAULT_CFG,    /* control */

        #if(CY_UDB_V0)
            HMC5883L_I2C_INT_ENABLE_MASK, /* aux_ctl */

            #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
                HMC5883L_I2C_DEFAULT_ADDR, /* addr_d0 */
            #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */
        #endif /* (CY_UDB_V0) */
    #endif /* (HMC5883L_I2C_FF_IMPLEMENTED) */

    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
        HMC5883L_I2C_DEFAULT_TMOUT_PERIOD,
        HMC5883L_I2C_DEFAULT_TMOUT_INTR_MASK,

        #if(HMC5883L_I2C_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0)
            HMC5883L_I2C_DEFAULT_TMOUT_PRESCALER_PRD,
        #endif /* (HMC5883L_I2C_TIMEOUT_PRESCALER_ENABLED) */

    #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */
};

#if((HMC5883L_I2C_FF_IMPLEMENTED) && (HMC5883L_I2C_WAKEUP_ENABLED))
    volatile uint8 HMC5883L_I2C_wakeupSource;
#endif /* ((HMC5883L_I2C_FF_IMPLEMENTED) && (HMC5883L_I2C_WAKEUP_ENABLED)) */


/*******************************************************************************
* Function Name: HMC5883L_I2C_SaveConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: disables I2C Master(if was enabled before go
*  to sleep), enables I2C backup regulator. Waits while on-going transaction be
*  will completed and I2C will be ready go to sleep. All incoming transaction
*  will be NACKed till power down will be asserted. The address match event
*  wakes up the chip.
*  Wakeup on address match disabled: saves I2C configuration and non-retention
*  register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  HMC5883L_I2C_backup - used to save component configuration and
*       none-retention registers before enter sleep mode.
*
* Reentrant:
*  No
*
*******************************************************************************/
void HMC5883L_I2C_SaveConfig(void) 
{
    #if(HMC5883L_I2C_FF_IMPLEMENTED)
        #if(HMC5883L_I2C_WAKEUP_ENABLED)
            uint8 enableInterrupts;
        #endif /* (HMC5883L_I2C_WAKEUP_ENABLED) */

        /* Store regiters in either Sleep mode */
        HMC5883L_I2C_backup.cfg  = HMC5883L_I2C_CFG_REG;
        HMC5883L_I2C_backup.xcfg = HMC5883L_I2C_XCFG_REG;

        #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
            HMC5883L_I2C_backup.addr = HMC5883L_I2C_ADDR_REG;
        #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            HMC5883L_I2C_backup.clkDiv   = HMC5883L_I2C_CLKDIV_REG;
        #else
            HMC5883L_I2C_backup.clkDiv1  = HMC5883L_I2C_CLKDIV1_REG;
            HMC5883L_I2C_backup.clkDiv2  = HMC5883L_I2C_CLKDIV2_REG;
        #endif /* (CY_PSOC5A) */

        #if(HMC5883L_I2C_WAKEUP_ENABLED)
            /* Need to disable Master */
            HMC5883L_I2C_CFG_REG &= ((uint8) ~HMC5883L_I2C_ENABLE_MASTER);

            /* Enable the I2C regulator backup */
            enableInterrupts = CyEnterCriticalSection();
            HMC5883L_I2C_PWRSYS_CR1_REG |= HMC5883L_I2C_PWRSYS_CR1_I2C_REG_BACKUP;
            CyExitCriticalSection(enableInterrupts);

            /* 1) Set force NACK to ignore I2C transactions
               2) Wait while I2C will be ready go to Sleep
               3) These bits are cleared on wake up */
            HMC5883L_I2C_XCFG_REG |= HMC5883L_I2C_XCFG_FORCE_NACK;
            while(0u == (HMC5883L_I2C_XCFG_REG & HMC5883L_I2C_XCFG_RDY_TO_SLEEP))
            {
                ; /* Wait when block is ready to Sleep */
            }

            /* Setup wakeup interrupt */
            HMC5883L_I2C_DisableInt();
            (void) CyIntSetVector(HMC5883L_I2C_ISR_NUMBER, &HMC5883L_I2C_WAKEUP_ISR);
            HMC5883L_I2C_wakeupSource = 0u;
            HMC5883L_I2C_EnableInt();

        #endif /* (HMC5883L_I2C_WAKEUP_ENABLED) */

    #else
        /* Store only address match bit */
        HMC5883L_I2C_backup.control = (HMC5883L_I2C_CFG_REG & HMC5883L_I2C_CTRL_ANY_ADDRESS_MASK);

        #if(CY_UDB_V0)
            /* Store interrupt mask bits */
            HMC5883L_I2C_backup.intMask = HMC5883L_I2C_INT_MASK_REG;

            #if(HMC5883L_I2C_MODE & HMC5883L_I2C_MODE_SLAVE)
                HMC5883L_I2C_backup.addr = HMC5883L_I2C_ADDR_REG;
            #endif /* (HMC5883L_I2C_MODE & HMC5883L_I2C_MODE_SLAVE) */

        #endif /* (CY_UDB_V0) */

    #endif /* (HMC5883L_I2C_FF_IMPLEMENTED) */

    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
        HMC5883L_I2C_TimeoutSaveConfig();   /* Save Timeout config */
    #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: HMC5883L_I2C_Sleep
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: All incoming transaction will be NACKed till
*  power down will be asserted. The address match event wakes up the chip.
*  Wakeup on address match disabled: Disables active mode power template bits or
*  clock gating as appropriate. Saves I2C configuration and non-retention
*  register values.
*  Disables I2C interrupt.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void HMC5883L_I2C_Sleep(void) 
{
    #if(HMC5883L_I2C_WAKEUP_ENABLED)
        /* The I2C block should be always enabled if used as wakeup source */
        HMC5883L_I2C_backup.enableState = HMC5883L_I2C_DISABLE;

        #if(HMC5883L_I2C_TIMEOUT_ENABLED)
            HMC5883L_I2C_TimeoutStop();
        #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */

    #else

        HMC5883L_I2C_backup.enableState = ((uint8) HMC5883L_I2C_IS_ENABLED);

        if(HMC5883L_I2C_IS_ENABLED)
        {
            HMC5883L_I2C_Stop();
        }
    #endif /* (HMC5883L_I2C_WAKEUP_ENABLED) */

    HMC5883L_I2C_SaveConfig();
}


/*******************************************************************************
* Function Name: HMC5883L_I2C_RestoreConfig
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep), disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and
*  non-retention register values.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global Variables:
*  HMC5883L_I2C_backup - used to save component configuration and
*  none-retention registers before exit sleep mode.
*
*******************************************************************************/
void HMC5883L_I2C_RestoreConfig(void) 
{
    #if(HMC5883L_I2C_FF_IMPLEMENTED)
        uint8 enableInterrupts;

        if(HMC5883L_I2C_CHECK_PWRSYS_I2C_BACKUP)    /* Enabled if was in Sleep */
        {
            /* Disable back-up regulator */
            enableInterrupts = CyEnterCriticalSection();
            HMC5883L_I2C_PWRSYS_CR1_REG &= ((uint8) ~HMC5883L_I2C_PWRSYS_CR1_I2C_REG_BACKUP);
            CyExitCriticalSection(enableInterrupts);

            /* Re-enable Master */
            HMC5883L_I2C_CFG_REG = HMC5883L_I2C_backup.cfg;
        }
        else /* The I2C_REG_BACKUP was cleaned by PM API: it means Hibernate or wake-up not set */
        {
            #if(HMC5883L_I2C_WAKEUP_ENABLED)
                /* Disable power to I2C block before register restore */
                enableInterrupts = CyEnterCriticalSection();
                HMC5883L_I2C_ACT_PWRMGR_REG  &= ((uint8) ~HMC5883L_I2C_ACT_PWR_EN);
                HMC5883L_I2C_STBY_PWRMGR_REG &= ((uint8) ~HMC5883L_I2C_STBY_PWR_EN);
                CyExitCriticalSection(enableInterrupts);

                /* Enable component after restore complete */
                HMC5883L_I2C_backup.enableState = HMC5883L_I2C_ENABLE;
            #endif /* (HMC5883L_I2C_WAKEUP_ENABLED) */

            /* Restore component registers: Hibernate disable power */
            HMC5883L_I2C_XCFG_REG = HMC5883L_I2C_backup.xcfg;
            HMC5883L_I2C_CFG_REG  = HMC5883L_I2C_backup.cfg;

            #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
                HMC5883L_I2C_ADDR_REG = HMC5883L_I2C_backup.addr;
            #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

            #if(CY_PSOC5A)
                HMC5883L_I2C_CLKDIV_REG  = HMC5883L_I2C_backup.clkDiv;
            #else
                HMC5883L_I2C_CLKDIV1_REG = HMC5883L_I2C_backup.clkDiv1;
                HMC5883L_I2C_CLKDIV2_REG = HMC5883L_I2C_backup.clkDiv2;
            #endif /* (CY_PSOC5A) */
        }

        #if(HMC5883L_I2C_WAKEUP_ENABLED)
            HMC5883L_I2C_DisableInt();
            (void) CyIntSetVector(HMC5883L_I2C_ISR_NUMBER, &HMC5883L_I2C_ISR);
            if(0u != HMC5883L_I2C_wakeupSource)
            {
                HMC5883L_I2C_SetPendingInt();   /* Generate interrupt to process incomming transcation */
            }
            HMC5883L_I2C_EnableInt();
        #endif /* (HMC5883L_I2C_WAKEUP_ENABLED) */

    #else

        #if(CY_UDB_V0)
            uint8 enableInterrupts;

            HMC5883L_I2C_INT_MASK_REG |= HMC5883L_I2C_backup.intMask;

            enableInterrupts = CyEnterCriticalSection();
            HMC5883L_I2C_INT_ENABLE_REG |= HMC5883L_I2C_INT_ENABLE_MASK;
            CyExitCriticalSection(enableInterrupts);

            #if(HMC5883L_I2C_MODE_MASTER_ENABLED)
                /* Restore Master Clock generator */
                HMC5883L_I2C_MCLK_PRD_REG = HMC5883L_I2C_DEFAULT_MCLK_PRD;
                HMC5883L_I2C_MCLK_CMP_REG = HMC5883L_I2C_DEFAULT_MCLK_CMP;
            #endif /* (HMC5883L_I2C_MODE_MASTER_ENABLED) */

            #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
                HMC5883L_I2C_ADDR_REG = HMC5883L_I2C_backup.addr;

                /* Restore slave bit counter period */
                HMC5883L_I2C_PERIOD_REG = HMC5883L_I2C_DEFAULT_PERIOD;
            #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

        #endif /* (CY_UDB_V0) */

        HMC5883L_I2C_CFG_REG = HMC5883L_I2C_backup.control;

    #endif /* (HMC5883L_I2C_FF_IMPLEMENTED) */

    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
        HMC5883L_I2C_TimeoutRestoreConfig();
    #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: HMC5883L_I2C_Wakeup
********************************************************************************
*
* Summary:
*  Wakeup on address match enabled: enables I2C Master (if was enabled before go
*  to sleep) and disables I2C backup regulator.
*  Wakeup on address match disabled: Restores I2C configuration and
*  non-retention register values. Restores Active mode power template bits or
*  clock gating as appropriate.
*  The I2C interrupt remains disabled after function call.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Reentrant:
*  No
*
*******************************************************************************/
void HMC5883L_I2C_Wakeup(void) 
{
    HMC5883L_I2C_RestoreConfig();   /* Restore I2C register settings */

    /* Restore component enable state */
    if(0u != HMC5883L_I2C_backup.enableState)
    {
        HMC5883L_I2C_Enable();
        HMC5883L_I2C_EnableInt();
    }
    else
    {
        #if(HMC5883L_I2C_TIMEOUT_ENABLED)
            HMC5883L_I2C_TimeoutEnable();
        #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */
    }
}


/* [] END OF FILE */
