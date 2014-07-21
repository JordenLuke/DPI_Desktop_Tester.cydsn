/*******************************************************************************
* File Name: MAG_I2C_PM.c
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

#include "MAG_I2C_PVT.h"

MAG_I2C_BACKUP_STRUCT MAG_I2C_backup =
{
    MAG_I2C_DISABLE, /* enableState */

    #if(MAG_I2C_FF_IMPLEMENTED)
        MAG_I2C_DEFAULT_XCFG,  /* xcfg */
        MAG_I2C_DEFAULT_CFG,   /* cfg  */

        #if(MAG_I2C_MODE_SLAVE_ENABLED)
            MAG_I2C_DEFAULT_ADDR, /* addr */
        #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            LO8(MAG_I2C_DEFAULT_DIVIDE_FACTOR),  /* div */
        #else
            LO8(MAG_I2C_DEFAULT_DIVIDE_FACTOR), /* div1 */
            HI8(MAG_I2C_DEFAULT_DIVIDE_FACTOR), /* div2 */
        #endif /* (CY_PSOC5A) */

    #else  /* (MAG_I2C_UDB_IMPLEMENTED) */
        MAG_I2C_DEFAULT_CFG,    /* control */

        #if(CY_UDB_V0)
            MAG_I2C_INT_ENABLE_MASK, /* aux_ctl */

            #if(MAG_I2C_MODE_SLAVE_ENABLED)
                MAG_I2C_DEFAULT_ADDR, /* addr_d0 */
            #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */
        #endif /* (CY_UDB_V0) */
    #endif /* (MAG_I2C_FF_IMPLEMENTED) */

    #if(MAG_I2C_TIMEOUT_ENABLED)
        MAG_I2C_DEFAULT_TMOUT_PERIOD,
        MAG_I2C_DEFAULT_TMOUT_INTR_MASK,

        #if(MAG_I2C_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0)
            MAG_I2C_DEFAULT_TMOUT_PRESCALER_PRD,
        #endif /* (MAG_I2C_TIMEOUT_PRESCALER_ENABLED) */

    #endif /* (MAG_I2C_TIMEOUT_ENABLED) */
};

#if((MAG_I2C_FF_IMPLEMENTED) && (MAG_I2C_WAKEUP_ENABLED))
    volatile uint8 MAG_I2C_wakeupSource;
#endif /* ((MAG_I2C_FF_IMPLEMENTED) && (MAG_I2C_WAKEUP_ENABLED)) */


/*******************************************************************************
* Function Name: MAG_I2C_SaveConfig
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
*  MAG_I2C_backup - used to save component configuration and
*       none-retention registers before enter sleep mode.
*
* Reentrant:
*  No
*
*******************************************************************************/
void MAG_I2C_SaveConfig(void) 
{
    #if(MAG_I2C_FF_IMPLEMENTED)
        #if(MAG_I2C_WAKEUP_ENABLED)
            uint8 enableInterrupts;
        #endif /* (MAG_I2C_WAKEUP_ENABLED) */

        /* Store regiters in either Sleep mode */
        MAG_I2C_backup.cfg  = MAG_I2C_CFG_REG;
        MAG_I2C_backup.xcfg = MAG_I2C_XCFG_REG;

        #if(MAG_I2C_MODE_SLAVE_ENABLED)
            MAG_I2C_backup.addr = MAG_I2C_ADDR_REG;
        #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            MAG_I2C_backup.clkDiv   = MAG_I2C_CLKDIV_REG;
        #else
            MAG_I2C_backup.clkDiv1  = MAG_I2C_CLKDIV1_REG;
            MAG_I2C_backup.clkDiv2  = MAG_I2C_CLKDIV2_REG;
        #endif /* (CY_PSOC5A) */

        #if(MAG_I2C_WAKEUP_ENABLED)
            /* Need to disable Master */
            MAG_I2C_CFG_REG &= ((uint8) ~MAG_I2C_ENABLE_MASTER);

            /* Enable the I2C regulator backup */
            enableInterrupts = CyEnterCriticalSection();
            MAG_I2C_PWRSYS_CR1_REG |= MAG_I2C_PWRSYS_CR1_I2C_REG_BACKUP;
            CyExitCriticalSection(enableInterrupts);

            /* 1) Set force NACK to ignore I2C transactions
               2) Wait while I2C will be ready go to Sleep
               3) These bits are cleared on wake up */
            MAG_I2C_XCFG_REG |= MAG_I2C_XCFG_FORCE_NACK;
            while(0u == (MAG_I2C_XCFG_REG & MAG_I2C_XCFG_RDY_TO_SLEEP))
            {
                ; /* Wait when block is ready to Sleep */
            }

            /* Setup wakeup interrupt */
            MAG_I2C_DisableInt();
            (void) CyIntSetVector(MAG_I2C_ISR_NUMBER, &MAG_I2C_WAKEUP_ISR);
            MAG_I2C_wakeupSource = 0u;
            MAG_I2C_EnableInt();

        #endif /* (MAG_I2C_WAKEUP_ENABLED) */

    #else
        /* Store only address match bit */
        MAG_I2C_backup.control = (MAG_I2C_CFG_REG & MAG_I2C_CTRL_ANY_ADDRESS_MASK);

        #if(CY_UDB_V0)
            /* Store interrupt mask bits */
            MAG_I2C_backup.intMask = MAG_I2C_INT_MASK_REG;

            #if(MAG_I2C_MODE & MAG_I2C_MODE_SLAVE)
                MAG_I2C_backup.addr = MAG_I2C_ADDR_REG;
            #endif /* (MAG_I2C_MODE & MAG_I2C_MODE_SLAVE) */

        #endif /* (CY_UDB_V0) */

    #endif /* (MAG_I2C_FF_IMPLEMENTED) */

    #if(MAG_I2C_TIMEOUT_ENABLED)
        MAG_I2C_TimeoutSaveConfig();   /* Save Timeout config */
    #endif /* (MAG_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: MAG_I2C_Sleep
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
void MAG_I2C_Sleep(void) 
{
    #if(MAG_I2C_WAKEUP_ENABLED)
        /* The I2C block should be always enabled if used as wakeup source */
        MAG_I2C_backup.enableState = MAG_I2C_DISABLE;

        #if(MAG_I2C_TIMEOUT_ENABLED)
            MAG_I2C_TimeoutStop();
        #endif /* (MAG_I2C_TIMEOUT_ENABLED) */

    #else

        MAG_I2C_backup.enableState = ((uint8) MAG_I2C_IS_ENABLED);

        if(MAG_I2C_IS_ENABLED)
        {
            MAG_I2C_Stop();
        }
    #endif /* (MAG_I2C_WAKEUP_ENABLED) */

    MAG_I2C_SaveConfig();
}


/*******************************************************************************
* Function Name: MAG_I2C_RestoreConfig
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
*  MAG_I2C_backup - used to save component configuration and
*  none-retention registers before exit sleep mode.
*
*******************************************************************************/
void MAG_I2C_RestoreConfig(void) 
{
    #if(MAG_I2C_FF_IMPLEMENTED)
        uint8 enableInterrupts;

        if(MAG_I2C_CHECK_PWRSYS_I2C_BACKUP)    /* Enabled if was in Sleep */
        {
            /* Disable back-up regulator */
            enableInterrupts = CyEnterCriticalSection();
            MAG_I2C_PWRSYS_CR1_REG &= ((uint8) ~MAG_I2C_PWRSYS_CR1_I2C_REG_BACKUP);
            CyExitCriticalSection(enableInterrupts);

            /* Re-enable Master */
            MAG_I2C_CFG_REG = MAG_I2C_backup.cfg;
        }
        else /* The I2C_REG_BACKUP was cleaned by PM API: it means Hibernate or wake-up not set */
        {
            #if(MAG_I2C_WAKEUP_ENABLED)
                /* Disable power to I2C block before register restore */
                enableInterrupts = CyEnterCriticalSection();
                MAG_I2C_ACT_PWRMGR_REG  &= ((uint8) ~MAG_I2C_ACT_PWR_EN);
                MAG_I2C_STBY_PWRMGR_REG &= ((uint8) ~MAG_I2C_STBY_PWR_EN);
                CyExitCriticalSection(enableInterrupts);

                /* Enable component after restore complete */
                MAG_I2C_backup.enableState = MAG_I2C_ENABLE;
            #endif /* (MAG_I2C_WAKEUP_ENABLED) */

            /* Restore component registers: Hibernate disable power */
            MAG_I2C_XCFG_REG = MAG_I2C_backup.xcfg;
            MAG_I2C_CFG_REG  = MAG_I2C_backup.cfg;

            #if(MAG_I2C_MODE_SLAVE_ENABLED)
                MAG_I2C_ADDR_REG = MAG_I2C_backup.addr;
            #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */

            #if(CY_PSOC5A)
                MAG_I2C_CLKDIV_REG  = MAG_I2C_backup.clkDiv;
            #else
                MAG_I2C_CLKDIV1_REG = MAG_I2C_backup.clkDiv1;
                MAG_I2C_CLKDIV2_REG = MAG_I2C_backup.clkDiv2;
            #endif /* (CY_PSOC5A) */
        }

        #if(MAG_I2C_WAKEUP_ENABLED)
            MAG_I2C_DisableInt();
            (void) CyIntSetVector(MAG_I2C_ISR_NUMBER, &MAG_I2C_ISR);
            if(0u != MAG_I2C_wakeupSource)
            {
                MAG_I2C_SetPendingInt();   /* Generate interrupt to process incomming transcation */
            }
            MAG_I2C_EnableInt();
        #endif /* (MAG_I2C_WAKEUP_ENABLED) */

    #else

        #if(CY_UDB_V0)
            uint8 enableInterrupts;

            MAG_I2C_INT_MASK_REG |= MAG_I2C_backup.intMask;

            enableInterrupts = CyEnterCriticalSection();
            MAG_I2C_INT_ENABLE_REG |= MAG_I2C_INT_ENABLE_MASK;
            CyExitCriticalSection(enableInterrupts);

            #if(MAG_I2C_MODE_MASTER_ENABLED)
                /* Restore Master Clock generator */
                MAG_I2C_MCLK_PRD_REG = MAG_I2C_DEFAULT_MCLK_PRD;
                MAG_I2C_MCLK_CMP_REG = MAG_I2C_DEFAULT_MCLK_CMP;
            #endif /* (MAG_I2C_MODE_MASTER_ENABLED) */

            #if(MAG_I2C_MODE_SLAVE_ENABLED)
                MAG_I2C_ADDR_REG = MAG_I2C_backup.addr;

                /* Restore slave bit counter period */
                MAG_I2C_PERIOD_REG = MAG_I2C_DEFAULT_PERIOD;
            #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */

        #endif /* (CY_UDB_V0) */

        MAG_I2C_CFG_REG = MAG_I2C_backup.control;

    #endif /* (MAG_I2C_FF_IMPLEMENTED) */

    #if(MAG_I2C_TIMEOUT_ENABLED)
        MAG_I2C_TimeoutRestoreConfig();
    #endif /* (MAG_I2C_TIMEOUT_ENABLED) */
}


/*******************************************************************************
* Function Name: MAG_I2C_Wakeup
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
void MAG_I2C_Wakeup(void) 
{
    MAG_I2C_RestoreConfig();   /* Restore I2C register settings */

    /* Restore component enable state */
    if(0u != MAG_I2C_backup.enableState)
    {
        MAG_I2C_Enable();
        MAG_I2C_EnableInt();
    }
    else
    {
        #if(MAG_I2C_TIMEOUT_ENABLED)
            MAG_I2C_TimeoutEnable();
        #endif /* (MAG_I2C_TIMEOUT_ENABLED) */
    }
}


/* [] END OF FILE */
