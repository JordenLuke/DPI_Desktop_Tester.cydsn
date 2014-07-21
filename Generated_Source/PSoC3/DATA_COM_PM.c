/*******************************************************************************
* File Name: DATA_COM_PM.c
* Version 2.30
*
* Description:
*  This file provides Sleep/WakeUp APIs functionality.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "DATA_COM.h"


/***************************************
* Local data allocation
***************************************/

static DATA_COM_BACKUP_STRUCT  DATA_COM_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: DATA_COM_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  DATA_COM_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void DATA_COM_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(DATA_COM_CONTROL_REG_REMOVED == 0u)
            DATA_COM_backup.cr = DATA_COM_CONTROL_REG;
        #endif /* End DATA_COM_CONTROL_REG_REMOVED */

        #if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
            DATA_COM_backup.rx_period = DATA_COM_RXBITCTR_PERIOD_REG;
            DATA_COM_backup.rx_mask = DATA_COM_RXSTATUS_MASK_REG;
            #if (DATA_COM_RXHW_ADDRESS_ENABLED)
                DATA_COM_backup.rx_addr1 = DATA_COM_RXADDRESS1_REG;
                DATA_COM_backup.rx_addr2 = DATA_COM_RXADDRESS2_REG;
            #endif /* End DATA_COM_RXHW_ADDRESS_ENABLED */
        #endif /* End DATA_COM_RX_ENABLED | DATA_COM_HD_ENABLED*/

        #if(DATA_COM_TX_ENABLED)
            #if(DATA_COM_TXCLKGEN_DP)
                DATA_COM_backup.tx_clk_ctr = DATA_COM_TXBITCLKGEN_CTR_REG;
                DATA_COM_backup.tx_clk_compl = DATA_COM_TXBITCLKTX_COMPLETE_REG;
            #else
                DATA_COM_backup.tx_period = DATA_COM_TXBITCTR_PERIOD_REG;
            #endif /*End DATA_COM_TXCLKGEN_DP */
            DATA_COM_backup.tx_mask = DATA_COM_TXSTATUS_MASK_REG;
        #endif /*End DATA_COM_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(DATA_COM_CONTROL_REG_REMOVED == 0u)
            DATA_COM_backup.cr = DATA_COM_CONTROL_REG;
        #endif /* End DATA_COM_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: DATA_COM_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  DATA_COM_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void DATA_COM_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(DATA_COM_CONTROL_REG_REMOVED == 0u)
            DATA_COM_CONTROL_REG = DATA_COM_backup.cr;
        #endif /* End DATA_COM_CONTROL_REG_REMOVED */

        #if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
            DATA_COM_RXBITCTR_PERIOD_REG = DATA_COM_backup.rx_period;
            DATA_COM_RXSTATUS_MASK_REG = DATA_COM_backup.rx_mask;
            #if (DATA_COM_RXHW_ADDRESS_ENABLED)
                DATA_COM_RXADDRESS1_REG = DATA_COM_backup.rx_addr1;
                DATA_COM_RXADDRESS2_REG = DATA_COM_backup.rx_addr2;
            #endif /* End DATA_COM_RXHW_ADDRESS_ENABLED */
        #endif  /* End (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) */

        #if(DATA_COM_TX_ENABLED)
            #if(DATA_COM_TXCLKGEN_DP)
                DATA_COM_TXBITCLKGEN_CTR_REG = DATA_COM_backup.tx_clk_ctr;
                DATA_COM_TXBITCLKTX_COMPLETE_REG = DATA_COM_backup.tx_clk_compl;
            #else
                DATA_COM_TXBITCTR_PERIOD_REG = DATA_COM_backup.tx_period;
            #endif /*End DATA_COM_TXCLKGEN_DP */
            DATA_COM_TXSTATUS_MASK_REG = DATA_COM_backup.tx_mask;
        #endif /*End DATA_COM_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(DATA_COM_CONTROL_REG_REMOVED == 0u)
            DATA_COM_CONTROL_REG = DATA_COM_backup.cr;
        #endif /* End DATA_COM_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: DATA_COM_Sleep
********************************************************************************
*
* Summary:
*  Stops and saves the user configuration. Should be called
*  just prior to entering sleep.
*
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  DATA_COM_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void DATA_COM_Sleep(void)
{

    #if(DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED)
        if((DATA_COM_RXSTATUS_ACTL_REG  & DATA_COM_INT_ENABLE) != 0u)
        {
            DATA_COM_backup.enableState = 1u;
        }
        else
        {
            DATA_COM_backup.enableState = 0u;
        }
    #else
        if((DATA_COM_TXSTATUS_ACTL_REG  & DATA_COM_INT_ENABLE) !=0u)
        {
            DATA_COM_backup.enableState = 1u;
        }
        else
        {
            DATA_COM_backup.enableState = 0u;
        }
    #endif /* End DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED*/

    DATA_COM_Stop();
    DATA_COM_SaveConfig();
}


/*******************************************************************************
* Function Name: DATA_COM_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration. Should be called
*  just after awaking from sleep.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  DATA_COM_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void DATA_COM_Wakeup(void)
{
    DATA_COM_RestoreConfig();
    #if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
        DATA_COM_ClearRxBuffer();
    #endif /* End (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) */
    #if(DATA_COM_TX_ENABLED || DATA_COM_HD_ENABLED)
        DATA_COM_ClearTxBuffer();
    #endif /* End DATA_COM_TX_ENABLED || DATA_COM_HD_ENABLED */

    if(DATA_COM_backup.enableState != 0u)
    {
        DATA_COM_Enable();
    }
}


/* [] END OF FILE */
