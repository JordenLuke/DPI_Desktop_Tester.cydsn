/*******************************************************************************
* File Name: CONFIG_COM_PM.c
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

#include "CONFIG_COM.h"


/***************************************
* Local data allocation
***************************************/

static CONFIG_COM_BACKUP_STRUCT  CONFIG_COM_backup =
{
    /* enableState - disabled */
    0u,
};



/*******************************************************************************
* Function Name: CONFIG_COM_SaveConfig
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
*  CONFIG_COM_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void CONFIG_COM_SaveConfig(void)
{
    #if (CY_UDB_V0)

        #if(CONFIG_COM_CONTROL_REG_REMOVED == 0u)
            CONFIG_COM_backup.cr = CONFIG_COM_CONTROL_REG;
        #endif /* End CONFIG_COM_CONTROL_REG_REMOVED */

        #if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
            CONFIG_COM_backup.rx_period = CONFIG_COM_RXBITCTR_PERIOD_REG;
            CONFIG_COM_backup.rx_mask = CONFIG_COM_RXSTATUS_MASK_REG;
            #if (CONFIG_COM_RXHW_ADDRESS_ENABLED)
                CONFIG_COM_backup.rx_addr1 = CONFIG_COM_RXADDRESS1_REG;
                CONFIG_COM_backup.rx_addr2 = CONFIG_COM_RXADDRESS2_REG;
            #endif /* End CONFIG_COM_RXHW_ADDRESS_ENABLED */
        #endif /* End CONFIG_COM_RX_ENABLED | CONFIG_COM_HD_ENABLED*/

        #if(CONFIG_COM_TX_ENABLED)
            #if(CONFIG_COM_TXCLKGEN_DP)
                CONFIG_COM_backup.tx_clk_ctr = CONFIG_COM_TXBITCLKGEN_CTR_REG;
                CONFIG_COM_backup.tx_clk_compl = CONFIG_COM_TXBITCLKTX_COMPLETE_REG;
            #else
                CONFIG_COM_backup.tx_period = CONFIG_COM_TXBITCTR_PERIOD_REG;
            #endif /*End CONFIG_COM_TXCLKGEN_DP */
            CONFIG_COM_backup.tx_mask = CONFIG_COM_TXSTATUS_MASK_REG;
        #endif /*End CONFIG_COM_TX_ENABLED */


    #else /* CY_UDB_V1 */

        #if(CONFIG_COM_CONTROL_REG_REMOVED == 0u)
            CONFIG_COM_backup.cr = CONFIG_COM_CONTROL_REG;
        #endif /* End CONFIG_COM_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: CONFIG_COM_RestoreConfig
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
*  CONFIG_COM_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void CONFIG_COM_RestoreConfig(void)
{

    #if (CY_UDB_V0)

        #if(CONFIG_COM_CONTROL_REG_REMOVED == 0u)
            CONFIG_COM_CONTROL_REG = CONFIG_COM_backup.cr;
        #endif /* End CONFIG_COM_CONTROL_REG_REMOVED */

        #if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
            CONFIG_COM_RXBITCTR_PERIOD_REG = CONFIG_COM_backup.rx_period;
            CONFIG_COM_RXSTATUS_MASK_REG = CONFIG_COM_backup.rx_mask;
            #if (CONFIG_COM_RXHW_ADDRESS_ENABLED)
                CONFIG_COM_RXADDRESS1_REG = CONFIG_COM_backup.rx_addr1;
                CONFIG_COM_RXADDRESS2_REG = CONFIG_COM_backup.rx_addr2;
            #endif /* End CONFIG_COM_RXHW_ADDRESS_ENABLED */
        #endif  /* End (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) */

        #if(CONFIG_COM_TX_ENABLED)
            #if(CONFIG_COM_TXCLKGEN_DP)
                CONFIG_COM_TXBITCLKGEN_CTR_REG = CONFIG_COM_backup.tx_clk_ctr;
                CONFIG_COM_TXBITCLKTX_COMPLETE_REG = CONFIG_COM_backup.tx_clk_compl;
            #else
                CONFIG_COM_TXBITCTR_PERIOD_REG = CONFIG_COM_backup.tx_period;
            #endif /*End CONFIG_COM_TXCLKGEN_DP */
            CONFIG_COM_TXSTATUS_MASK_REG = CONFIG_COM_backup.tx_mask;
        #endif /*End CONFIG_COM_TX_ENABLED */

    #else /* CY_UDB_V1 */

        #if(CONFIG_COM_CONTROL_REG_REMOVED == 0u)
            CONFIG_COM_CONTROL_REG = CONFIG_COM_backup.cr;
        #endif /* End CONFIG_COM_CONTROL_REG_REMOVED */

    #endif  /* End CY_UDB_V0 */
}


/*******************************************************************************
* Function Name: CONFIG_COM_Sleep
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
*  CONFIG_COM_backup - modified when non-retention registers are saved.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void CONFIG_COM_Sleep(void)
{

    #if(CONFIG_COM_RX_ENABLED || CONFIG_COM_HD_ENABLED)
        if((CONFIG_COM_RXSTATUS_ACTL_REG  & CONFIG_COM_INT_ENABLE) != 0u)
        {
            CONFIG_COM_backup.enableState = 1u;
        }
        else
        {
            CONFIG_COM_backup.enableState = 0u;
        }
    #else
        if((CONFIG_COM_TXSTATUS_ACTL_REG  & CONFIG_COM_INT_ENABLE) !=0u)
        {
            CONFIG_COM_backup.enableState = 1u;
        }
        else
        {
            CONFIG_COM_backup.enableState = 0u;
        }
    #endif /* End CONFIG_COM_RX_ENABLED || CONFIG_COM_HD_ENABLED*/

    CONFIG_COM_Stop();
    CONFIG_COM_SaveConfig();
}


/*******************************************************************************
* Function Name: CONFIG_COM_Wakeup
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
*  CONFIG_COM_backup - used when non-retention registers are restored.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void CONFIG_COM_Wakeup(void)
{
    CONFIG_COM_RestoreConfig();
    #if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
        CONFIG_COM_ClearRxBuffer();
    #endif /* End (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) */
    #if(CONFIG_COM_TX_ENABLED || CONFIG_COM_HD_ENABLED)
        CONFIG_COM_ClearTxBuffer();
    #endif /* End CONFIG_COM_TX_ENABLED || CONFIG_COM_HD_ENABLED */

    if(CONFIG_COM_backup.enableState != 0u)
    {
        CONFIG_COM_Enable();
    }
}


/* [] END OF FILE */
