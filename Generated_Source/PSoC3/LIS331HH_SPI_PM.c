/*******************************************************************************
* File Name: LIS331HH_SPI_PM.c
* Version 2.40
*
* Description:
*  This file contains the setup, control and status commands to support
*  component operations in low power mode.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "LIS331HH_SPI_PVT.h"

static LIS331HH_SPI_BACKUP_STRUCT LIS331HH_SPI_backup =
{
    LIS331HH_SPI_DISABLED,
    LIS331HH_SPI_BITCTR_INIT,
    #if(CY_UDB_V0)
        LIS331HH_SPI_TX_INIT_INTERRUPTS_MASK,
        LIS331HH_SPI_RX_INIT_INTERRUPTS_MASK
    #endif /* CY_UDB_V0 */
};


/*******************************************************************************
* Function Name: LIS331HH_SPI_SaveConfig
********************************************************************************
*
* Summary:
*  Saves SPIM configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LIS331HH_SPI_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_SaveConfig(void) 
{
    /* Store Status Mask registers */
    #if(CY_UDB_V0)
       LIS331HH_SPI_backup.cntrPeriod      = LIS331HH_SPI_COUNTER_PERIOD_REG;
       LIS331HH_SPI_backup.saveSrTxIntMask = LIS331HH_SPI_TX_STATUS_MASK_REG;
       LIS331HH_SPI_backup.saveSrRxIntMask = LIS331HH_SPI_RX_STATUS_MASK_REG;
    #endif /* (CY_UDB_V0) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores SPIM configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LIS331HH_SPI_backup - used when non-retention registers are restored.
*
* Side Effects:
*  If this API is called without first calling SaveConfig then in the following
*  registers will be default values from Customizer:
*  LIS331HH_SPI_STATUS_MASK_REG and LIS331HH_SPI_COUNTER_PERIOD_REG.
*
*******************************************************************************/
void LIS331HH_SPI_RestoreConfig(void) 
{
    /* Restore the data, saved by SaveConfig() function */
    #if(CY_UDB_V0)
        LIS331HH_SPI_COUNTER_PERIOD_REG = LIS331HH_SPI_backup.cntrPeriod;
        LIS331HH_SPI_TX_STATUS_MASK_REG = ((uint8) LIS331HH_SPI_backup.saveSrTxIntMask);
        LIS331HH_SPI_RX_STATUS_MASK_REG = ((uint8) LIS331HH_SPI_backup.saveSrRxIntMask);
    #endif /* (CY_UDB_V0) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_Sleep
********************************************************************************
*
* Summary:
*  Prepare SPIM Component goes to sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LIS331HH_SPI_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_Sleep(void) 
{
    /* Save components enable state */
    LIS331HH_SPI_backup.enableState = ((uint8) LIS331HH_SPI_IS_ENABLED);

    LIS331HH_SPI_Stop();
    LIS331HH_SPI_SaveConfig();
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_Wakeup
********************************************************************************
*
* Summary:
*  Prepare SPIM Component to wake up.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  LIS331HH_SPI_backup - used when non-retention registers are restored.
*  LIS331HH_SPI_txBufferWrite - modified every function call - resets to
*  zero.
*  LIS331HH_SPI_txBufferRead - modified every function call - resets to
*  zero.
*  LIS331HH_SPI_rxBufferWrite - modified every function call - resets to
*  zero.
*  LIS331HH_SPI_rxBufferRead - modified every function call - resets to
*  zero.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_Wakeup(void) 
{
    LIS331HH_SPI_RestoreConfig();

    #if(LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED)
        LIS331HH_SPI_rxBufferFull  = 0u;
        LIS331HH_SPI_rxBufferRead  = 0u;
        LIS331HH_SPI_rxBufferWrite = 0u;
    #endif /* (LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED) */

    #if(LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED)
        LIS331HH_SPI_txBufferFull  = 0u;
        LIS331HH_SPI_txBufferRead  = 0u;
        LIS331HH_SPI_txBufferWrite = 0u;
    #endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) */

    /* Clear any data from the RX and TX FIFO */
    LIS331HH_SPI_ClearFIFO();

    /* Restore components block enable state */
    if(0u != LIS331HH_SPI_backup.enableState)
    {
        LIS331HH_SPI_Enable();
    }
}


/* [] END OF FILE */
