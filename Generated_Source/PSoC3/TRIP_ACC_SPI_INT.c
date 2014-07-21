/*******************************************************************************
* File Name: TRIP_ACC_SPI_INT.c
* Version 2.40
*
* Description:
*  This file provides all Interrupt Service Routine (ISR) for the SPI Master
*  component.
*
* Note:
*  None.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "TRIP_ACC_SPI_PVT.h"

/* User code required at start of ISR */
/* `#START TRIP_ACC_SPI_ISR_START_DEF` */

/* `#END` */


/*******************************************************************************
* Function Name: TRIP_ACC_SPI_TX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for TX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  TRIP_ACC_SPI_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer.
*  TRIP_ACC_SPI_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer, modified when exist data to
*  sending and FIFO Not Full.
*  TRIP_ACC_SPI_txBuffer[TRIP_ACC_SPI_TX_BUFFER_SIZE] - used to store
*  data to sending.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(TRIP_ACC_SPI_TX_ISR)
{
    #if(TRIP_ACC_SPI_TX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
    #endif /* (TRIP_ACC_SPI_TX_SOFTWARE_BUF_ENABLED) */

    /* User code required at start of ISR */
    /* `#START TRIP_ACC_SPI_TX_ISR_START` */

    /* `#END` */

    #if(TRIP_ACC_SPI_TX_SOFTWARE_BUF_ENABLED)
        /* Check if TX data buffer is not empty and there is space in TX FIFO */
        while(TRIP_ACC_SPI_txBufferRead != TRIP_ACC_SPI_txBufferWrite)
        {
            tmpStatus = TRIP_ACC_SPI_GET_STATUS_TX(TRIP_ACC_SPI_swStatusTx);
            TRIP_ACC_SPI_swStatusTx = tmpStatus;

            if(0u != (TRIP_ACC_SPI_swStatusTx & TRIP_ACC_SPI_STS_TX_FIFO_NOT_FULL))
            {
                if(0u == TRIP_ACC_SPI_txBufferFull)
                {
                   TRIP_ACC_SPI_txBufferRead++;

                    if(TRIP_ACC_SPI_txBufferRead >= TRIP_ACC_SPI_TX_BUFFER_SIZE)
                    {
                        TRIP_ACC_SPI_txBufferRead = 0u;
                    }
                }
                else
                {
                    TRIP_ACC_SPI_txBufferFull = 0u;
                }

                /* Move data from the Buffer to the FIFO */
                CY_SET_REG8(TRIP_ACC_SPI_TXDATA_PTR,
                    TRIP_ACC_SPI_txBuffer[TRIP_ACC_SPI_txBufferRead]);
            }
            else
            {
                break;
            }
        }

        if(TRIP_ACC_SPI_txBufferRead == TRIP_ACC_SPI_txBufferWrite)
        {
            /* TX Buffer is EMPTY: disable interrupt on TX NOT FULL */
            TRIP_ACC_SPI_TX_STATUS_MASK_REG &= ((uint8) ~TRIP_ACC_SPI_STS_TX_FIFO_NOT_FULL);
        }

    #endif /* (TRIP_ACC_SPI_TX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START TRIP_ACC_SPI_TX_ISR_END` */

    /* `#END` */
}


/*******************************************************************************
* Function Name: TRIP_ACC_SPI_RX_ISR
********************************************************************************
*
* Summary:
*  Interrupt Service Routine for RX portion of the SPI Master.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  TRIP_ACC_SPI_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer modified when FIFO contains
*  new data.
*  TRIP_ACC_SPI_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified when overflow occurred.
*  TRIP_ACC_SPI_rxBuffer[TRIP_ACC_SPI_RX_BUFFER_SIZE] - used to store
*  received data, modified when FIFO contains new data.
*  All described above Global variables are used when Software Buffer is used.
*
*******************************************************************************/
CY_ISR(TRIP_ACC_SPI_RX_ISR)
{
    #if(TRIP_ACC_SPI_RX_SOFTWARE_BUF_ENABLED)
        uint8 tmpStatus;
        uint8 rxData;
    #endif /* (TRIP_ACC_SPI_RX_SOFTWARE_BUF_ENABLED) */

    /* User code required at start of ISR */
    /* `#START TRIP_ACC_SPI_RX_ISR_START` */

    /* `#END` */

    #if(TRIP_ACC_SPI_RX_SOFTWARE_BUF_ENABLED)

        tmpStatus = TRIP_ACC_SPI_GET_STATUS_RX(TRIP_ACC_SPI_swStatusRx);
        TRIP_ACC_SPI_swStatusRx = tmpStatus;

        /* Check if RX data FIFO has some data to be moved into the RX Buffer */
        while(0u != (TRIP_ACC_SPI_swStatusRx & TRIP_ACC_SPI_STS_RX_FIFO_NOT_EMPTY))
        {
            rxData = CY_GET_REG8(TRIP_ACC_SPI_RXDATA_PTR);

            /* Set next pointer. */
            TRIP_ACC_SPI_rxBufferWrite++;
            if(TRIP_ACC_SPI_rxBufferWrite >= TRIP_ACC_SPI_RX_BUFFER_SIZE)
            {
                TRIP_ACC_SPI_rxBufferWrite = 0u;
            }

            if(TRIP_ACC_SPI_rxBufferWrite == TRIP_ACC_SPI_rxBufferRead)
            {
                TRIP_ACC_SPI_rxBufferRead++;
                if(TRIP_ACC_SPI_rxBufferRead >= TRIP_ACC_SPI_RX_BUFFER_SIZE)
                {
                    TRIP_ACC_SPI_rxBufferRead = 0u;
                }

                TRIP_ACC_SPI_rxBufferFull = 1u;
            }

            /* Move data from the FIFO to the Buffer */
            TRIP_ACC_SPI_rxBuffer[TRIP_ACC_SPI_rxBufferWrite] = rxData;

            tmpStatus = TRIP_ACC_SPI_GET_STATUS_RX(TRIP_ACC_SPI_swStatusRx);
            TRIP_ACC_SPI_swStatusRx = tmpStatus;
        }

    #endif /* (TRIP_ACC_SPI_RX_SOFTWARE_BUF_ENABLED) */

    /* User code required at end of ISR (Optional) */
    /* `#START TRIP_ACC_SPI_RX_ISR_END` */

    /* `#END` */
}

/* [] END OF FILE */
