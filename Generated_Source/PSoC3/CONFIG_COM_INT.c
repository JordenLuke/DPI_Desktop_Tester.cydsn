/*******************************************************************************
* File Name: CONFIG_COM_INT.c
* Version 2.30
*
* Description:
*  This file provides all Interrupt Service functionality of the UART component
*
* Note:
*  Any unusual or non-standard behavior should be noted here. Other-
*  wise, this section should remain blank.
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "CONFIG_COM.h"
#include "CyLib.h"


/***************************************
* Custom Declratations
***************************************/
/* `#START CUSTOM_DECLARATIONS` Place your declaration here */

/* `#END` */

#if( (CONFIG_COM_RX_ENABLED || CONFIG_COM_HD_ENABLED) && \
     (CONFIG_COM_RXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: CONFIG_COM_RXISR
    ********************************************************************************
    *
    * Summary:
    *  Interrupt Service Routine for RX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  CONFIG_COM_rxBuffer - RAM buffer pointer for save received data.
    *  CONFIG_COM_rxBufferWrite - cyclic index for write to rxBuffer,
    *     increments after each byte saved to buffer.
    *  CONFIG_COM_rxBufferRead - cyclic index for read from rxBuffer,
    *     checked to detect overflow condition.
    *  CONFIG_COM_rxBufferOverflow - software overflow flag. Set to one
    *     when CONFIG_COM_rxBufferWrite index overtakes
    *     CONFIG_COM_rxBufferRead index.
    *  CONFIG_COM_rxBufferLoopDetect - additional variable to detect overflow.
    *     Set to one when CONFIG_COM_rxBufferWrite is equal to
    *    CONFIG_COM_rxBufferRead
    *  CONFIG_COM_rxAddressMode - this variable contains the Address mode,
    *     selected in customizer or set by UART_SetRxAddressMode() API.
    *  CONFIG_COM_rxAddressDetected - set to 1 when correct address received,
    *     and analysed to store following addressed data bytes to the buffer.
    *     When not correct address received, set to 0 to skip following data bytes.
    *
    *******************************************************************************/
    CY_ISR(CONFIG_COM_RXISR)
    {
        uint8 readData;
        uint8 increment_pointer = 0u;
        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START CONFIG_COM_RXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        readData = CONFIG_COM_RXSTATUS_REG;

        if((readData & (CONFIG_COM_RX_STS_BREAK | CONFIG_COM_RX_STS_PAR_ERROR |
                        CONFIG_COM_RX_STS_STOP_ERROR | CONFIG_COM_RX_STS_OVERRUN)) != 0u)
        {
            /* ERROR handling. */
            /* `#START CONFIG_COM_RXISR_ERROR` */

            /* `#END` */
        }

        while((readData & CONFIG_COM_RX_STS_FIFO_NOTEMPTY) != 0u)
        {

            #if (CONFIG_COM_RXHW_ADDRESS_ENABLED)
                if(CONFIG_COM_rxAddressMode == (uint8)CONFIG_COM__B_UART__AM_SW_DETECT_TO_BUFFER)
                {
                    if((readData & CONFIG_COM_RX_STS_MRKSPC) != 0u)
                    {
                        if ((readData & CONFIG_COM_RX_STS_ADDR_MATCH) != 0u)
                        {
                            CONFIG_COM_rxAddressDetected = 1u;
                        }
                        else
                        {
                            CONFIG_COM_rxAddressDetected = 0u;
                        }
                    }

                    readData = CONFIG_COM_RXDATA_REG;
                    if(CONFIG_COM_rxAddressDetected != 0u)
                    {   /* store only addressed data */
                        CONFIG_COM_rxBuffer[CONFIG_COM_rxBufferWrite] = readData;
                        increment_pointer = 1u;
                    }
                }
                else /* without software addressing */
                {
                    CONFIG_COM_rxBuffer[CONFIG_COM_rxBufferWrite] = CONFIG_COM_RXDATA_REG;
                    increment_pointer = 1u;
                }
            #else  /* without addressing */
                CONFIG_COM_rxBuffer[CONFIG_COM_rxBufferWrite] = CONFIG_COM_RXDATA_REG;
                increment_pointer = 1u;
            #endif /* End SW_DETECT_TO_BUFFER */

            /* do not increment buffer pointer when skip not adderessed data */
            if( increment_pointer != 0u )
            {
                if(CONFIG_COM_rxBufferLoopDetect != 0u)
                {   /* Set Software Buffer status Overflow */
                    CONFIG_COM_rxBufferOverflow = 1u;
                }
                /* Set next pointer. */
                CONFIG_COM_rxBufferWrite++;

                /* Check pointer for a loop condition */
                if(CONFIG_COM_rxBufferWrite >= CONFIG_COM_RXBUFFERSIZE)
                {
                    CONFIG_COM_rxBufferWrite = 0u;
                }
                /* Detect pre-overload condition and set flag */
                if(CONFIG_COM_rxBufferWrite == CONFIG_COM_rxBufferRead)
                {
                    CONFIG_COM_rxBufferLoopDetect = 1u;
                    /* When Hardware Flow Control selected */
                    #if(CONFIG_COM_FLOW_CONTROL != 0u)
                    /* Disable RX interrupt mask, it will be enabled when user read data from the buffer using APIs */
                        CONFIG_COM_RXSTATUS_MASK_REG  &= (uint8)~CONFIG_COM_RX_STS_FIFO_NOTEMPTY;
                        CyIntClearPending(CONFIG_COM_RX_VECT_NUM);
                        break; /* Break the reading of the FIFO loop, leave the data there for generating RTS signal */
                    #endif /* End CONFIG_COM_FLOW_CONTROL != 0 */
                }
            }

            /* Check again if there is data. */
            readData = CONFIG_COM_RXSTATUS_REG;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START CONFIG_COM_RXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End CONFIG_COM_RX_ENABLED && (CONFIG_COM_RXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH) */


#if(CONFIG_COM_TX_ENABLED && (CONFIG_COM_TXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH))


    /*******************************************************************************
    * Function Name: CONFIG_COM_TXISR
    ********************************************************************************
    *
    * Summary:
    * Interrupt Service Routine for the TX portion of the UART
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  CONFIG_COM_txBuffer - RAM buffer pointer for transmit data from.
    *  CONFIG_COM_txBufferRead - cyclic index for read and transmit data
    *     from txBuffer, increments after each transmited byte.
    *  CONFIG_COM_rxBufferWrite - cyclic index for write to txBuffer,
    *     checked to detect available for transmission bytes.
    *
    *******************************************************************************/
    CY_ISR(CONFIG_COM_TXISR)
    {

        #if(CY_PSOC3)
            uint8 int_en;
        #endif /* CY_PSOC3 */

        /* User code required at start of ISR */
        /* `#START CONFIG_COM_TXISR_START` */

        /* `#END` */

        #if(CY_PSOC3)   /* Make sure nested interrupt is enabled */
            int_en = EA;
            CyGlobalIntEnable;
        #endif /* CY_PSOC3 */

        while((CONFIG_COM_txBufferRead != CONFIG_COM_txBufferWrite) &&
             ((CONFIG_COM_TXSTATUS_REG & CONFIG_COM_TX_STS_FIFO_FULL) == 0u))
        {
            /* Check pointer. */
            if(CONFIG_COM_txBufferRead >= CONFIG_COM_TXBUFFERSIZE)
            {
                CONFIG_COM_txBufferRead = 0u;
            }

            CONFIG_COM_TXDATA_REG = CONFIG_COM_txBuffer[CONFIG_COM_txBufferRead];

            /* Set next pointer. */
            CONFIG_COM_txBufferRead++;
        }

        /* User code required at end of ISR (Optional) */
        /* `#START CONFIG_COM_TXISR_END` */

        /* `#END` */

        #if(CY_PSOC3)
            EA = int_en;
        #endif /* CY_PSOC3 */

    }

#endif /* End CONFIG_COM_TX_ENABLED && (CONFIG_COM_TXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH) */


/* [] END OF FILE */
