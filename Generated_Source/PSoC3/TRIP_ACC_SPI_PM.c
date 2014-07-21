/*******************************************************************************
* File Name: TRIP_ACC_SPI_PM.c
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

#include "TRIP_ACC_SPI_PVT.h"

static TRIP_ACC_SPI_BACKUP_STRUCT TRIP_ACC_SPI_backup =
{
    TRIP_ACC_SPI_DISABLED,
    TRIP_ACC_SPI_BITCTR_INIT,
    #if(CY_UDB_V0)
        TRIP_ACC_SPI_TX_INIT_INTERRUPTS_MASK,
        TRIP_ACC_SPI_RX_INIT_INTERRUPTS_MASK
    #endif /* CY_UDB_V0 */
};


/*******************************************************************************
* Function Name: TRIP_ACC_SPI_SaveConfig
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
*  TRIP_ACC_SPI_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void TRIP_ACC_SPI_SaveConfig(void) 
{
    /* Store Status Mask registers */
    #if(CY_UDB_V0)
       TRIP_ACC_SPI_backup.cntrPeriod      = TRIP_ACC_SPI_COUNTER_PERIOD_REG;
       TRIP_ACC_SPI_backup.saveSrTxIntMask = TRIP_ACC_SPI_TX_STATUS_MASK_REG;
       TRIP_ACC_SPI_backup.saveSrRxIntMask = TRIP_ACC_SPI_RX_STATUS_MASK_REG;
    #endif /* (CY_UDB_V0) */
}


/*******************************************************************************
* Function Name: TRIP_ACC_SPI_RestoreConfig
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
*  TRIP_ACC_SPI_backup - used when non-retention registers are restored.
*
* Side Effects:
*  If this API is called without first calling SaveConfig then in the following
*  registers will be default values from Customizer:
*  TRIP_ACC_SPI_STATUS_MASK_REG and TRIP_ACC_SPI_COUNTER_PERIOD_REG.
*
*******************************************************************************/
void TRIP_ACC_SPI_RestoreConfig(void) 
{
    /* Restore the data, saved by SaveConfig() function */
    #if(CY_UDB_V0)
        TRIP_ACC_SPI_COUNTER_PERIOD_REG = TRIP_ACC_SPI_backup.cntrPeriod;
        TRIP_ACC_SPI_TX_STATUS_MASK_REG = ((uint8) TRIP_ACC_SPI_backup.saveSrTxIntMask);
        TRIP_ACC_SPI_RX_STATUS_MASK_REG = ((uint8) TRIP_ACC_SPI_backup.saveSrRxIntMask);
    #endif /* (CY_UDB_V0) */
}


/*******************************************************************************
* Function Name: TRIP_ACC_SPI_Sleep
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
*  TRIP_ACC_SPI_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void TRIP_ACC_SPI_Sleep(void) 
{
    /* Save components enable state */
    TRIP_ACC_SPI_backup.enableState = ((uint8) TRIP_ACC_SPI_IS_ENABLED);

    TRIP_ACC_SPI_Stop();
    TRIP_ACC_SPI_SaveConfig();
}


/*******************************************************************************
* Function Name: TRIP_ACC_SPI_Wakeup
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
*  TRIP_ACC_SPI_backup - used when non-retention registers are restored.
*  TRIP_ACC_SPI_txBufferWrite - modified every function call - resets to
*  zero.
*  TRIP_ACC_SPI_txBufferRead - modified every function call - resets to
*  zero.
*  TRIP_ACC_SPI_rxBufferWrite - modified every function call - resets to
*  zero.
*  TRIP_ACC_SPI_rxBufferRead - modified every function call - resets to
*  zero.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void TRIP_ACC_SPI_Wakeup(void) 
{
    TRIP_ACC_SPI_RestoreConfig();

    #if(TRIP_ACC_SPI_RX_SOFTWARE_BUF_ENABLED)
        TRIP_ACC_SPI_rxBufferFull  = 0u;
        TRIP_ACC_SPI_rxBufferRead  = 0u;
        TRIP_ACC_SPI_rxBufferWrite = 0u;
    #endif /* (TRIP_ACC_SPI_RX_SOFTWARE_BUF_ENABLED) */

    #if(TRIP_ACC_SPI_TX_SOFTWARE_BUF_ENABLED)
        TRIP_ACC_SPI_txBufferFull  = 0u;
        TRIP_ACC_SPI_txBufferRead  = 0u;
        TRIP_ACC_SPI_txBufferWrite = 0u;
    #endif /* (TRIP_ACC_SPI_TX_SOFTWARE_BUF_ENABLED) */

    /* Clear any data from the RX and TX FIFO */
    TRIP_ACC_SPI_ClearFIFO();

    /* Restore components block enable state */
    if(0u != TRIP_ACC_SPI_backup.enableState)
    {
        TRIP_ACC_SPI_Enable();
    }
}


/* [] END OF FILE */
