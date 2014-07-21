/*******************************************************************************
* File Name: DATA_COM.c
* Version 2.30
*
* Description:
*  This file provides all API functionality of the UART component
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
#include "CyLib.h"
#if(DATA_COM_INTERNAL_CLOCK_USED)
    #include "DATA_COM_IntClock.h"
#endif /* End DATA_COM_INTERNAL_CLOCK_USED */


/***************************************
* Global data allocation
***************************************/

uint8 DATA_COM_initVar = 0u;
#if( DATA_COM_TX_ENABLED && (DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
    volatile uint8 DATA_COM_txBuffer[DATA_COM_TXBUFFERSIZE];
    volatile uint8 DATA_COM_txBufferRead = 0u;
    uint8 DATA_COM_txBufferWrite = 0u;
#endif /* End DATA_COM_TX_ENABLED */
#if( ( DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED ) && \
     (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH) )
    volatile uint8 DATA_COM_rxBuffer[DATA_COM_RXBUFFERSIZE];
    volatile uint8 DATA_COM_rxBufferRead = 0u;
    volatile uint8 DATA_COM_rxBufferWrite = 0u;
    volatile uint8 DATA_COM_rxBufferLoopDetect = 0u;
    volatile uint8 DATA_COM_rxBufferOverflow = 0u;
    #if (DATA_COM_RXHW_ADDRESS_ENABLED)
        volatile uint8 DATA_COM_rxAddressMode = DATA_COM_RXADDRESSMODE;
        volatile uint8 DATA_COM_rxAddressDetected = 0u;
    #endif /* End EnableHWAddress */
#endif /* End DATA_COM_RX_ENABLED */


/*******************************************************************************
* Function Name: DATA_COM_Start
********************************************************************************
*
* Summary:
*  Initialize and Enable the UART component.
*  Enable the clock input to enable operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  The DATA_COM_intiVar variable is used to indicate initial
*  configuration of this component. The variable is initialized to zero (0u)
*  and set to one (1u) the first time UART_Start() is called. This allows for
*  component initialization without re-initialization in all subsequent calls
*  to the DATA_COM_Start() routine.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void DATA_COM_Start(void) 
{
    /* If not Initialized then initialize all required hardware and software */
    if(DATA_COM_initVar == 0u)
    {
        DATA_COM_Init();
        DATA_COM_initVar = 1u;
    }
    DATA_COM_Enable();
}


/*******************************************************************************
* Function Name: DATA_COM_Init
********************************************************************************
*
* Summary:
*  Initialize component's parameters to the parameters set by user in the
*  customizer of the component placed onto schematic. Usually called in
*  DATA_COM_Start().
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void DATA_COM_Init(void) 
{
    #if(DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED)

        #if(DATA_COM_RX_INTERRUPT_ENABLED && (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            /* Set the RX Interrupt. */
            (void)CyIntSetVector(DATA_COM_RX_VECT_NUM, &DATA_COM_RXISR);
            CyIntSetPriority(DATA_COM_RX_VECT_NUM, DATA_COM_RX_PRIOR_NUM);
        #endif /* End DATA_COM_RX_INTERRUPT_ENABLED */

        #if (DATA_COM_RXHW_ADDRESS_ENABLED)
            DATA_COM_SetRxAddressMode(DATA_COM_RXAddressMode);
            DATA_COM_SetRxAddress1(DATA_COM_RXHWADDRESS1);
            DATA_COM_SetRxAddress2(DATA_COM_RXHWADDRESS2);
        #endif /* End DATA_COM_RXHW_ADDRESS_ENABLED */

        /* Init Count7 period */
        DATA_COM_RXBITCTR_PERIOD_REG = DATA_COM_RXBITCTR_INIT;
        /* Configure the Initial RX interrupt mask */
        DATA_COM_RXSTATUS_MASK_REG  = DATA_COM_INIT_RX_INTERRUPTS_MASK;
    #endif /* End DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED*/

    #if(DATA_COM_TX_ENABLED)
        #if(DATA_COM_TX_INTERRUPT_ENABLED && (DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            /* Set the TX Interrupt. */
            (void)CyIntSetVector(DATA_COM_TX_VECT_NUM, &DATA_COM_TXISR);
            CyIntSetPriority(DATA_COM_TX_VECT_NUM, DATA_COM_TX_PRIOR_NUM);
        #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */

        /* Write Counter Value for TX Bit Clk Generator*/
        #if(DATA_COM_TXCLKGEN_DP)
            DATA_COM_TXBITCLKGEN_CTR_REG = DATA_COM_BIT_CENTER;
            DATA_COM_TXBITCLKTX_COMPLETE_REG = (DATA_COM_NUMBER_OF_DATA_BITS +
                        DATA_COM_NUMBER_OF_START_BIT) * DATA_COM_OVER_SAMPLE_COUNT;
        #else
            DATA_COM_TXBITCTR_PERIOD_REG = ((DATA_COM_NUMBER_OF_DATA_BITS +
                        DATA_COM_NUMBER_OF_START_BIT) * DATA_COM_OVER_SAMPLE_8) - 1u;
        #endif /* End DATA_COM_TXCLKGEN_DP */

        /* Configure the Initial TX interrupt mask */
        #if(DATA_COM_TX_INTERRUPT_ENABLED && (DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            DATA_COM_TXSTATUS_MASK_REG = DATA_COM_TX_STS_FIFO_EMPTY;
        #else
            DATA_COM_TXSTATUS_MASK_REG = DATA_COM_INIT_TX_INTERRUPTS_MASK;
        #endif /*End DATA_COM_TX_INTERRUPT_ENABLED*/

    #endif /* End DATA_COM_TX_ENABLED */

    #if(DATA_COM_PARITY_TYPE_SW)  /* Write Parity to Control Register */
        DATA_COM_WriteControlRegister( \
            (DATA_COM_ReadControlRegister() & (uint8)~DATA_COM_CTRL_PARITY_TYPE_MASK) | \
            (uint8)(DATA_COM_PARITY_TYPE << DATA_COM_CTRL_PARITY_TYPE0_SHIFT) );
    #endif /* End DATA_COM_PARITY_TYPE_SW */
}


/*******************************************************************************
* Function Name: DATA_COM_Enable
********************************************************************************
*
* Summary:
*  Enables the UART block operation
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global Variables:
*  DATA_COM_rxAddressDetected - set to initial state (0).
*
*******************************************************************************/
void DATA_COM_Enable(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    #if(DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED)
        /*RX Counter (Count7) Enable */
        DATA_COM_RXBITCTR_CONTROL_REG |= DATA_COM_CNTR_ENABLE;
        /* Enable the RX Interrupt. */
        DATA_COM_RXSTATUS_ACTL_REG  |= DATA_COM_INT_ENABLE;
        #if(DATA_COM_RX_INTERRUPT_ENABLED && (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            CyIntEnable(DATA_COM_RX_VECT_NUM);
            #if (DATA_COM_RXHW_ADDRESS_ENABLED)
                DATA_COM_rxAddressDetected = 0u;
            #endif /* End DATA_COM_RXHW_ADDRESS_ENABLED */
        #endif /* End DATA_COM_RX_INTERRUPT_ENABLED */
    #endif /* End DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED*/

    #if(DATA_COM_TX_ENABLED)
        /*TX Counter (DP/Count7) Enable */
        #if(!DATA_COM_TXCLKGEN_DP)
            DATA_COM_TXBITCTR_CONTROL_REG |= DATA_COM_CNTR_ENABLE;
        #endif /* End DATA_COM_TXCLKGEN_DP */
        /* Enable the TX Interrupt. */
        DATA_COM_TXSTATUS_ACTL_REG |= DATA_COM_INT_ENABLE;
        #if(DATA_COM_TX_INTERRUPT_ENABLED && (DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            CyIntEnable(DATA_COM_TX_VECT_NUM);
        #endif /* End DATA_COM_TX_INTERRUPT_ENABLED*/
     #endif /* End DATA_COM_TX_ENABLED */

    #if(DATA_COM_INTERNAL_CLOCK_USED)
        /* Enable the clock. */
        DATA_COM_IntClock_Start();
    #endif /* End DATA_COM_INTERNAL_CLOCK_USED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: DATA_COM_Stop
********************************************************************************
*
* Summary:
*  Disable the UART component
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void DATA_COM_Stop(void) 
{
    uint8 enableInterrupts;
    enableInterrupts = CyEnterCriticalSection();

    /* Write Bit Counter Disable */
    #if(DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED)
        DATA_COM_RXBITCTR_CONTROL_REG &= (uint8)~DATA_COM_CNTR_ENABLE;
    #endif /* End DATA_COM_RX_ENABLED */

    #if(DATA_COM_TX_ENABLED)
        #if(!DATA_COM_TXCLKGEN_DP)
            DATA_COM_TXBITCTR_CONTROL_REG &= (uint8)~DATA_COM_CNTR_ENABLE;
        #endif /* End DATA_COM_TXCLKGEN_DP */
    #endif /* DATA_COM_TX_ENABLED */

    #if(DATA_COM_INTERNAL_CLOCK_USED)
        /* Disable the clock. */
        DATA_COM_IntClock_Stop();
    #endif /* End DATA_COM_INTERNAL_CLOCK_USED */

    /* Disable internal interrupt component */
    #if(DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED)
        DATA_COM_RXSTATUS_ACTL_REG  &= (uint8)~DATA_COM_INT_ENABLE;
        #if(DATA_COM_RX_INTERRUPT_ENABLED && (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            DATA_COM_DisableRxInt();
        #endif /* End DATA_COM_RX_INTERRUPT_ENABLED */
    #endif /* End DATA_COM_RX_ENABLED */

    #if(DATA_COM_TX_ENABLED)
        DATA_COM_TXSTATUS_ACTL_REG &= (uint8)~DATA_COM_INT_ENABLE;
        #if(DATA_COM_TX_INTERRUPT_ENABLED && (DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            DATA_COM_DisableTxInt();
        #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */
    #endif /* End DATA_COM_TX_ENABLED */

    CyExitCriticalSection(enableInterrupts);
}


/*******************************************************************************
* Function Name: DATA_COM_ReadControlRegister
********************************************************************************
*
* Summary:
*  Read the current state of the control register
*
* Parameters:
*  None.
*
* Return:
*  Current state of the control register.
*
*******************************************************************************/
uint8 DATA_COM_ReadControlRegister(void) 
{
    #if( DATA_COM_CONTROL_REG_REMOVED )
        return(0u);
    #else
        return(DATA_COM_CONTROL_REG);
    #endif /* End DATA_COM_CONTROL_REG_REMOVED */
}


/*******************************************************************************
* Function Name: DATA_COM_WriteControlRegister
********************************************************************************
*
* Summary:
*  Writes an 8-bit value into the control register
*
* Parameters:
*  control:  control register value
*
* Return:
*  None.
*
*******************************************************************************/
void  DATA_COM_WriteControlRegister(uint8 control) 
{
    #if( DATA_COM_CONTROL_REG_REMOVED )
        if(control != 0u) { }      /* release compiler warning */
    #else
       DATA_COM_CONTROL_REG = control;
    #endif /* End DATA_COM_CONTROL_REG_REMOVED */
}


#if(DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED)

    #if(DATA_COM_RX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: DATA_COM_EnableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void DATA_COM_EnableRxInt(void) 
        {
            CyIntEnable(DATA_COM_RX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: DATA_COM_DisableRxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable RX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void DATA_COM_DisableRxInt(void) 
        {
            CyIntDisable(DATA_COM_RX_VECT_NUM);
        }

    #endif /* DATA_COM_RX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: DATA_COM_SetRxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  IntSrc:  An or'd combination of the desired status bit masks (defined in
    *           the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void DATA_COM_SetRxInterruptMode(uint8 intSrc) 
    {
        DATA_COM_RXSTATUS_MASK_REG  = intSrc;
    }


    /*******************************************************************************
    * Function Name: DATA_COM_ReadRxData
    ********************************************************************************
    *
    * Summary:
    *  Returns data in RX Data register without checking status register to
    *  determine if data is valid
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Received data from RX register
    *
    * Global Variables:
    *  DATA_COM_rxBuffer - RAM buffer pointer for save received data.
    *  DATA_COM_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  DATA_COM_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  DATA_COM_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 DATA_COM_ReadRxData(void) 
    {
        uint8 rxData;

        #if(DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_DisableRxInt();
            #endif /* DATA_COM_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = DATA_COM_rxBufferRead;
            loc_rxBufferWrite = DATA_COM_rxBufferWrite;

            if( (DATA_COM_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = DATA_COM_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;

                if(loc_rxBufferRead >= DATA_COM_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                DATA_COM_rxBufferRead = loc_rxBufferRead;

                if(DATA_COM_rxBufferLoopDetect != 0u )
                {
                    DATA_COM_rxBufferLoopDetect = 0u;
                    #if( (DATA_COM_RX_INTERRUPT_ENABLED) && (DATA_COM_FLOW_CONTROL != 0u) && \
                         (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( DATA_COM_HD_ENABLED )
                            if((DATA_COM_CONTROL_REG & DATA_COM_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only in RX
                                *  configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                DATA_COM_RXSTATUS_MASK_REG  |= DATA_COM_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            DATA_COM_RXSTATUS_MASK_REG  |= DATA_COM_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end DATA_COM_HD_ENABLED */
                    #endif /* DATA_COM_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }
            }
            else
            {   /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
                rxData = DATA_COM_RXDATA_REG;
            }

            /* Enable Rx interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_EnableRxInt();
            #endif /* End DATA_COM_RX_INTERRUPT_ENABLED */

        #else /* DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

            /* Needs to check status for RX_STS_FIFO_NOTEMPTY bit*/
            rxData = DATA_COM_RXDATA_REG;

        #endif /* DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: DATA_COM_ReadRxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the current state of the status register
    *  And detect software buffer overflow.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Current state of the status register.
    *
    * Global Variables:
    *  DATA_COM_rxBufferOverflow - used to indicate overload condition.
    *   It set to one in RX interrupt when there isn?t free space in
    *   DATA_COM_rxBufferRead to write new data. This condition returned
    *   and cleared to zero by this API as an
    *   DATA_COM_RX_STS_SOFT_BUFF_OVER bit along with RX Status register
    *   bits.
    *
    *******************************************************************************/
    uint8 DATA_COM_ReadRxStatus(void) 
    {
        uint8 status;

        status = DATA_COM_RXSTATUS_REG & DATA_COM_RX_HW_MASK;

        #if(DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH)
            if( DATA_COM_rxBufferOverflow != 0u )
            {
                status |= DATA_COM_RX_STS_SOFT_BUFF_OVER;
                DATA_COM_rxBufferOverflow = 0u;
            }
        #endif /* DATA_COM_RXBUFFERSIZE */

        return(status);
    }


    /*******************************************************************************
    * Function Name: DATA_COM_GetChar
    ********************************************************************************
    *
    * Summary:
    *  Reads UART RX buffer immediately, if data is not available or an error
    *  condition exists, zero is returned; otherwise, character is read and
    *  returned.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Character read from UART RX buffer. ASCII characters from 1 to 255 are valid.
    *  A returned zero signifies an error condition or no data available.
    *
    * Global Variables:
    *  DATA_COM_rxBuffer - RAM buffer pointer for save received data.
    *  DATA_COM_rxBufferWrite - cyclic index for write to rxBuffer,
    *     checked to identify new data.
    *  DATA_COM_rxBufferRead - cyclic index for read from rxBuffer,
    *     incremented after each byte has been read from buffer.
    *  DATA_COM_rxBufferLoopDetect - creared if loop condition was detected
    *     in RX ISR.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint8 DATA_COM_GetChar(void) 
    {
        uint8 rxData = 0u;
        uint8 rxStatus;

        #if(DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH)
            uint8 loc_rxBufferRead;
            uint8 loc_rxBufferWrite;
            /* Protect variables that could change on interrupt. */
            /* Disable Rx interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_DisableRxInt();
            #endif /* DATA_COM_RX_INTERRUPT_ENABLED */
            loc_rxBufferRead = DATA_COM_rxBufferRead;
            loc_rxBufferWrite = DATA_COM_rxBufferWrite;

            if( (DATA_COM_rxBufferLoopDetect != 0u) || (loc_rxBufferRead != loc_rxBufferWrite) )
            {
                rxData = DATA_COM_rxBuffer[loc_rxBufferRead];
                loc_rxBufferRead++;
                if(loc_rxBufferRead >= DATA_COM_RXBUFFERSIZE)
                {
                    loc_rxBufferRead = 0u;
                }
                /* Update the real pointer */
                DATA_COM_rxBufferRead = loc_rxBufferRead;

                if(DATA_COM_rxBufferLoopDetect > 0u )
                {
                    DATA_COM_rxBufferLoopDetect = 0u;
                    #if( (DATA_COM_RX_INTERRUPT_ENABLED) && (DATA_COM_FLOW_CONTROL != 0u) )
                        /* When Hardware Flow Control selected - return RX mask */
                        #if( DATA_COM_HD_ENABLED )
                            if((DATA_COM_CONTROL_REG & DATA_COM_CTRL_HD_SEND) == 0u)
                            {   /* In Half duplex mode return RX mask only if
                                *  RX configuration set, otherwise
                                *  mask will be returned in LoadRxConfig() API.
                                */
                                DATA_COM_RXSTATUS_MASK_REG  |= DATA_COM_RX_STS_FIFO_NOTEMPTY;
                            }
                        #else
                            DATA_COM_RXSTATUS_MASK_REG  |= DATA_COM_RX_STS_FIFO_NOTEMPTY;
                        #endif /* end DATA_COM_HD_ENABLED */
                    #endif /* DATA_COM_RX_INTERRUPT_ENABLED and Hardware flow control*/
                }

            }
            else
            {   rxStatus = DATA_COM_RXSTATUS_REG;
                if((rxStatus & DATA_COM_RX_STS_FIFO_NOTEMPTY) != 0u)
                {   /* Read received data from FIFO*/
                    rxData = DATA_COM_RXDATA_REG;
                    /*Check status on error*/
                    if((rxStatus & (DATA_COM_RX_STS_BREAK | DATA_COM_RX_STS_PAR_ERROR |
                                   DATA_COM_RX_STS_STOP_ERROR | DATA_COM_RX_STS_OVERRUN)) != 0u)
                    {
                        rxData = 0u;
                    }
                }
            }

            /* Enable Rx interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_EnableRxInt();
            #endif /* DATA_COM_RX_INTERRUPT_ENABLED */

        #else /* DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

            rxStatus =DATA_COM_RXSTATUS_REG;
            if((rxStatus & DATA_COM_RX_STS_FIFO_NOTEMPTY) != 0u)
            {   /* Read received data from FIFO*/
                rxData = DATA_COM_RXDATA_REG;
                /*Check status on error*/
                if((rxStatus & (DATA_COM_RX_STS_BREAK | DATA_COM_RX_STS_PAR_ERROR |
                               DATA_COM_RX_STS_STOP_ERROR | DATA_COM_RX_STS_OVERRUN)) != 0u)
                {
                    rxData = 0u;
                }
            }
        #endif /* DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

        return(rxData);
    }


    /*******************************************************************************
    * Function Name: DATA_COM_GetByte
    ********************************************************************************
    *
    * Summary:
    *  Grab the next available byte of data from the recieve FIFO
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  MSB contains Status Register and LSB contains UART RX data
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    uint16 DATA_COM_GetByte(void) 
    {
        return ( ((uint16)DATA_COM_ReadRxStatus() << 8u) | DATA_COM_ReadRxData() );
    }


    /*******************************************************************************
    * Function Name: DATA_COM_GetRxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of bytes left in the RX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  uint8: Integer count of the number of bytes left
    *  in the RX buffer
    *
    * Global Variables:
    *  DATA_COM_rxBufferWrite - used to calculate left bytes.
    *  DATA_COM_rxBufferRead - used to calculate left bytes.
    *  DATA_COM_rxBufferLoopDetect - checked to decide left bytes amount.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the RX Buffer is.
    *
    *******************************************************************************/
    uint8 DATA_COM_GetRxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH)

            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_DisableRxInt();
            #endif /* DATA_COM_RX_INTERRUPT_ENABLED */

            if(DATA_COM_rxBufferRead == DATA_COM_rxBufferWrite)
            {
                if(DATA_COM_rxBufferLoopDetect > 0u)
                {
                    size = DATA_COM_RXBUFFERSIZE;
                }
                else
                {
                    size = 0u;
                }
            }
            else if(DATA_COM_rxBufferRead < DATA_COM_rxBufferWrite)
            {
                size = (DATA_COM_rxBufferWrite - DATA_COM_rxBufferRead);
            }
            else
            {
                size = (DATA_COM_RXBUFFERSIZE - DATA_COM_rxBufferRead) + DATA_COM_rxBufferWrite;
            }

            /* Enable Rx interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_EnableRxInt();
            #endif /* End DATA_COM_RX_INTERRUPT_ENABLED */

        #else /* DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

            /* We can only know if there is data in the fifo. */
            size = ((DATA_COM_RXSTATUS_REG & DATA_COM_RX_STS_FIFO_NOTEMPTY) != 0u) ? 1u : 0u;

        #endif /* End DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: DATA_COM_ClearRxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the RX RAM buffer by setting the read and write pointers both to zero.
    *  Clears hardware RX FIFO.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_rxBufferWrite - cleared to zero.
    *  DATA_COM_rxBufferRead - cleared to zero.
    *  DATA_COM_rxBufferLoopDetect - cleared to zero.
    *  DATA_COM_rxBufferOverflow - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may
    *  have remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM or FIFO buffer will be lost.
    *******************************************************************************/
    void DATA_COM_ClearRxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* clear the HW FIFO */
        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        DATA_COM_RXDATA_AUX_CTL_REG |=  DATA_COM_RX_FIFO_CLR;
        DATA_COM_RXDATA_AUX_CTL_REG &= (uint8)~DATA_COM_RX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH)
            /* Disable Rx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_DisableRxInt();
            #endif /* End DATA_COM_RX_INTERRUPT_ENABLED */

            DATA_COM_rxBufferRead = 0u;
            DATA_COM_rxBufferWrite = 0u;
            DATA_COM_rxBufferLoopDetect = 0u;
            DATA_COM_rxBufferOverflow = 0u;

            /* Enable Rx interrupt. */
            #if(DATA_COM_RX_INTERRUPT_ENABLED)
                DATA_COM_EnableRxInt();
            #endif /* End DATA_COM_RX_INTERRUPT_ENABLED */
        #endif /* End DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

    }


    /*******************************************************************************
    * Function Name: DATA_COM_SetRxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the receive addressing mode
    *
    * Parameters:
    *  addressMode: Enumerated value indicating the mode of RX addressing
    *  DATA_COM__B_UART__AM_SW_BYTE_BYTE -  Software Byte-by-Byte address
    *                                               detection
    *  DATA_COM__B_UART__AM_SW_DETECT_TO_BUFFER - Software Detect to Buffer
    *                                               address detection
    *  DATA_COM__B_UART__AM_HW_BYTE_BY_BYTE - Hardware Byte-by-Byte address
    *                                               detection
    *  DATA_COM__B_UART__AM_HW_DETECT_TO_BUFFER - Hardware Detect to Buffer
    *                                               address detection
    *  DATA_COM__B_UART__AM_NONE - No address detection
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_rxAddressMode - the parameter stored in this variable for
    *   the farther usage in RX ISR.
    *  DATA_COM_rxAddressDetected - set to initial state (0).
    *
    *******************************************************************************/
    void DATA_COM_SetRxAddressMode(uint8 addressMode)
                                                        
    {
        #if(DATA_COM_RXHW_ADDRESS_ENABLED)
            #if(DATA_COM_CONTROL_REG_REMOVED)
                if(addressMode != 0u) { }     /* release compiler warning */
            #else /* DATA_COM_CONTROL_REG_REMOVED */
                uint8 tmpCtrl;
                tmpCtrl = DATA_COM_CONTROL_REG & (uint8)~DATA_COM_CTRL_RXADDR_MODE_MASK;
                tmpCtrl |= (uint8)(addressMode << DATA_COM_CTRL_RXADDR_MODE0_SHIFT);
                DATA_COM_CONTROL_REG = tmpCtrl;
                #if(DATA_COM_RX_INTERRUPT_ENABLED && \
                   (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH) )
                    DATA_COM_rxAddressMode = addressMode;
                    DATA_COM_rxAddressDetected = 0u;
                #endif /* End DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH*/
            #endif /* End DATA_COM_CONTROL_REG_REMOVED */
        #else /* DATA_COM_RXHW_ADDRESS_ENABLED */
            if(addressMode != 0u) { }     /* release compiler warning */
        #endif /* End DATA_COM_RXHW_ADDRESS_ENABLED */
    }


    /*******************************************************************************
    * Function Name: DATA_COM_SetRxAddress1
    ********************************************************************************
    *
    * Summary:
    *  Set the first hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void DATA_COM_SetRxAddress1(uint8 address) 

    {
        DATA_COM_RXADDRESS1_REG = address;
    }


    /*******************************************************************************
    * Function Name: DATA_COM_SetRxAddress2
    ********************************************************************************
    *
    * Summary:
    *  Set the second hardware address compare value
    *
    * Parameters:
    *  address
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void DATA_COM_SetRxAddress2(uint8 address) 
    {
        DATA_COM_RXADDRESS2_REG = address;
    }

#endif  /* DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED*/


#if( (DATA_COM_TX_ENABLED) || (DATA_COM_HD_ENABLED) )

    #if(DATA_COM_TX_INTERRUPT_ENABLED)

        /*******************************************************************************
        * Function Name: DATA_COM_EnableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Enable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Enable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void DATA_COM_EnableTxInt(void) 
        {
            CyIntEnable(DATA_COM_TX_VECT_NUM);
        }


        /*******************************************************************************
        * Function Name: DATA_COM_DisableTxInt
        ********************************************************************************
        *
        * Summary:
        *  Disable TX interrupt generation
        *
        * Parameters:
        *  None.
        *
        * Return:
        *  None.
        *
        * Theory:
        *  Disable the interrupt output -or- the interrupt component itself
        *
        *******************************************************************************/
        void DATA_COM_DisableTxInt(void) 
        {
            CyIntDisable(DATA_COM_TX_VECT_NUM);
        }

    #endif /* DATA_COM_TX_INTERRUPT_ENABLED */


    /*******************************************************************************
    * Function Name: DATA_COM_SetTxInterruptMode
    ********************************************************************************
    *
    * Summary:
    *  Configure which status bits trigger an interrupt event
    *
    * Parameters:
    *  intSrc: An or'd combination of the desired status bit masks (defined in
    *          the header file)
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Enables the output of specific status bits to the interrupt controller
    *
    *******************************************************************************/
    void DATA_COM_SetTxInterruptMode(uint8 intSrc) 
    {
        DATA_COM_TXSTATUS_MASK_REG = intSrc;
    }


    /*******************************************************************************
    * Function Name: DATA_COM_WriteTxData
    ********************************************************************************
    *
    * Summary:
    *  Write a byte of data to the Transmit FIFO or TX buffer to be sent when the
    *  bus is available. WriteTxData sends a byte without checking for buffer room
    *  or status. It is up to the user to separately check status.
    *
    * Parameters:
    *  TXDataByte: byte of data to place in the transmit FIFO
    *
    * Return:
    * void
    *
    * Global Variables:
    *  DATA_COM_txBuffer - RAM buffer pointer for save data for transmission
    *  DATA_COM_txBufferWrite - cyclic index for write to txBuffer,
    *    incremented after each byte saved to buffer.
    *  DATA_COM_txBufferRead - cyclic index for read from txBuffer,
    *    checked to identify the condition to write to FIFO directly or to TX buffer
    *  DATA_COM_initVar - checked to identify that the component has been
    *    initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void DATA_COM_WriteTxData(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(DATA_COM_initVar != 0u)
        {
            #if(DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH)

                /* Disable Tx interrupt. */
                /* Protect variables that could change on interrupt. */
                #if(DATA_COM_TX_INTERRUPT_ENABLED)
                    DATA_COM_DisableTxInt();
                #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */

                if( (DATA_COM_txBufferRead == DATA_COM_txBufferWrite) &&
                    ((DATA_COM_TXSTATUS_REG & DATA_COM_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    DATA_COM_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(DATA_COM_txBufferWrite >= DATA_COM_TXBUFFERSIZE)
                    {
                        DATA_COM_txBufferWrite = 0u;
                    }

                    DATA_COM_txBuffer[DATA_COM_txBufferWrite] = txDataByte;

                    /* Add to the software buffer. */
                    DATA_COM_txBufferWrite++;

                }

                /* Enable Tx interrupt. */
                #if(DATA_COM_TX_INTERRUPT_ENABLED)
                    DATA_COM_EnableTxInt();
                #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */

            #else /* DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

                /* Add directly to the FIFO. */
                DATA_COM_TXDATA_REG = txDataByte;

            #endif /* End DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH */
        }
    }


    /*******************************************************************************
    * Function Name: DATA_COM_ReadTxStatus
    ********************************************************************************
    *
    * Summary:
    *  Read the status register for the component
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Contents of the status register
    *
    * Theory:
    *  This function reads the status register which is clear on read. It is up to
    *  the user to handle all bits in this return value accordingly, even if the bit
    *  was not enabled as an interrupt source the event happened and must be handled
    *  accordingly.
    *
    *******************************************************************************/
    uint8 DATA_COM_ReadTxStatus(void) 
    {
        return(DATA_COM_TXSTATUS_REG);
    }


    /*******************************************************************************
    * Function Name: DATA_COM_PutChar
    ********************************************************************************
    *
    * Summary:
    *  Wait to send byte until TX register or buffer has room.
    *
    * Parameters:
    *  txDataByte: The 8-bit data value to send across the UART.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_txBuffer - RAM buffer pointer for save data for transmission
    *  DATA_COM_txBufferWrite - cyclic index for write to txBuffer,
    *     checked to identify free space in txBuffer and incremented after each byte
    *     saved to buffer.
    *  DATA_COM_txBufferRead - cyclic index for read from txBuffer,
    *     checked to identify free space in txBuffer.
    *  DATA_COM_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to transmit any byte of data in a single transfer
    *
    *******************************************************************************/
    void DATA_COM_PutChar(uint8 txDataByte) 
    {
            #if(DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH)
                /* The temporary output pointer is used since it takes two instructions
                *  to increment with a wrap, and we can't risk doing that with the real
                *  pointer and getting an interrupt in between instructions.
                */
                uint8 loc_txBufferWrite;
                uint8 loc_txBufferRead;

                do{
                    /* Block if software buffer is full, so we don't overwrite. */
                    #if ((DATA_COM_TXBUFFERSIZE > DATA_COM_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Disable TX interrupt to protect variables that could change on interrupt */
                        CyIntDisable(DATA_COM_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    loc_txBufferWrite = DATA_COM_txBufferWrite;
                    loc_txBufferRead = DATA_COM_txBufferRead;
                    #if ((DATA_COM_TXBUFFERSIZE > DATA_COM_MAX_BYTE_VALUE) && (CY_PSOC3))
                        /* Enable interrupt to continue transmission */
                        CyIntEnable(DATA_COM_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }while( (loc_txBufferWrite < loc_txBufferRead) ? (loc_txBufferWrite == (loc_txBufferRead - 1u)) :
                                        ((loc_txBufferWrite - loc_txBufferRead) ==
                                        (uint8)(DATA_COM_TXBUFFERSIZE - 1u)) );

                if( (loc_txBufferRead == loc_txBufferWrite) &&
                    ((DATA_COM_TXSTATUS_REG & DATA_COM_TX_STS_FIFO_FULL) == 0u) )
                {
                    /* Add directly to the FIFO. */
                    DATA_COM_TXDATA_REG = txDataByte;
                }
                else
                {
                    if(loc_txBufferWrite >= DATA_COM_TXBUFFERSIZE)
                    {
                        loc_txBufferWrite = 0u;
                    }
                    /* Add to the software buffer. */
                    DATA_COM_txBuffer[loc_txBufferWrite] = txDataByte;
                    loc_txBufferWrite++;

                    /* Finally, update the real output pointer */
                    #if ((DATA_COM_TXBUFFERSIZE > DATA_COM_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntDisable(DATA_COM_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                    DATA_COM_txBufferWrite = loc_txBufferWrite;
                    #if ((DATA_COM_TXBUFFERSIZE > DATA_COM_MAX_BYTE_VALUE) && (CY_PSOC3))
                        CyIntEnable(DATA_COM_TX_VECT_NUM);
                    #endif /* End TXBUFFERSIZE > 255 */
                }

            #else /* DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

                while((DATA_COM_TXSTATUS_REG & DATA_COM_TX_STS_FIFO_FULL) != 0u)
                {
                    ; /* Wait for room in the FIFO. */
                }

                /* Add directly to the FIFO. */
                DATA_COM_TXDATA_REG = txDataByte;

            #endif /* End DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: DATA_COM_PutString
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: char pointer to character string of Data to Send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  This function will block if there is not enough memory to place the whole
    *  string, it will block until the entire string has been written to the
    *  transmit buffer.
    *
    *******************************************************************************/
    void DATA_COM_PutString(const char8 string[]) 
    {
        uint16 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(DATA_COM_initVar != 0u)
        {
            /* This is a blocking function, it will not exit until all data is sent*/
            while(string[buf_index] != (char8)0)
            {
                DATA_COM_PutChar((uint8)string[buf_index]);
                buf_index++;
            }
        }
    }


    /*******************************************************************************
    * Function Name: DATA_COM_PutArray
    ********************************************************************************
    *
    * Summary:
    *  Write a Sequence of bytes on the Transmit line. Data comes from RAM or ROM.
    *
    * Parameters:
    *  string: Address of the memory array residing in RAM or ROM.
    *  byteCount: Number of Bytes to be transmitted.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void DATA_COM_PutArray(const uint8 string[], uint8 byteCount)
                                                                    
    {
        uint8 buf_index = 0u;
        /* If not Initialized then skip this function*/
        if(DATA_COM_initVar != 0u)
        {
            do
            {
                DATA_COM_PutChar(string[buf_index]);
                buf_index++;
            }while(buf_index < byteCount);
        }
    }


    /*******************************************************************************
    * Function Name: DATA_COM_PutCRLF
    ********************************************************************************
    *
    * Summary:
    *  Write a character and then carriage return and line feed.
    *
    * Parameters:
    *  txDataByte: uint8 Character to send.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_initVar - checked to identify that the component has been
    *     initialized.
    *
    * Reentrant:
    *  No.
    *
    *******************************************************************************/
    void DATA_COM_PutCRLF(uint8 txDataByte) 
    {
        /* If not Initialized then skip this function*/
        if(DATA_COM_initVar != 0u)
        {
            DATA_COM_PutChar(txDataByte);
            DATA_COM_PutChar(0x0Du);
            DATA_COM_PutChar(0x0Au);
        }
    }


    /*******************************************************************************
    * Function Name: DATA_COM_GetTxBufferSize
    ********************************************************************************
    *
    * Summary:
    *  Determine the amount of space left in the TX buffer and return the count in
    *  bytes
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  Integer count of the number of bytes left in the TX buffer
    *
    * Global Variables:
    *  DATA_COM_txBufferWrite - used to calculate left space.
    *  DATA_COM_txBufferRead - used to calculate left space.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Allows the user to find out how full the TX Buffer is.
    *
    *******************************************************************************/
    uint8 DATA_COM_GetTxBufferSize(void)
                                                            
    {
        uint8 size;

        #if(DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(DATA_COM_TX_INTERRUPT_ENABLED)
                DATA_COM_DisableTxInt();
            #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */

            if(DATA_COM_txBufferRead == DATA_COM_txBufferWrite)
            {
                size = 0u;
            }
            else if(DATA_COM_txBufferRead < DATA_COM_txBufferWrite)
            {
                size = (DATA_COM_txBufferWrite - DATA_COM_txBufferRead);
            }
            else
            {
                size = (DATA_COM_TXBUFFERSIZE - DATA_COM_txBufferRead) + DATA_COM_txBufferWrite;
            }

            /* Enable Tx interrupt. */
            #if(DATA_COM_TX_INTERRUPT_ENABLED)
                DATA_COM_EnableTxInt();
            #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */

        #else /* DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

            size = DATA_COM_TXSTATUS_REG;

            /* Is the fifo is full. */
            if((size & DATA_COM_TX_STS_FIFO_FULL) != 0u)
            {
                size = DATA_COM_FIFO_LENGTH;
            }
            else if((size & DATA_COM_TX_STS_FIFO_EMPTY) != 0u)
            {
                size = 0u;
            }
            else
            {
                /* We only know there is data in the fifo. */
                size = 1u;
            }

        #endif /* End DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH */

        return(size);
    }


    /*******************************************************************************
    * Function Name: DATA_COM_ClearTxBuffer
    ********************************************************************************
    *
    * Summary:
    *  Clears the TX RAM buffer by setting the read and write pointers both to zero.
    *  Clears the hardware TX FIFO.  Any data present in the FIFO will not be sent.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_txBufferWrite - cleared to zero.
    *  DATA_COM_txBufferRead - cleared to zero.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  Setting the pointers to zero makes the system believe there is no data to
    *  read and writing will resume at address 0 overwriting any data that may have
    *  remained in the RAM.
    *
    * Side Effects:
    *  Any received data not read from the RAM buffer will be lost when overwritten.
    *
    *******************************************************************************/
    void DATA_COM_ClearTxBuffer(void) 
    {
        uint8 enableInterrupts;

        /* Enter critical section */
        enableInterrupts = CyEnterCriticalSection();
        /* clear the HW FIFO */
        DATA_COM_TXDATA_AUX_CTL_REG |=  DATA_COM_TX_FIFO_CLR;
        DATA_COM_TXDATA_AUX_CTL_REG &= (uint8)~DATA_COM_TX_FIFO_CLR;
        /* Exit critical section */
        CyExitCriticalSection(enableInterrupts);

        #if(DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH)

            /* Disable Tx interrupt. */
            /* Protect variables that could change on interrupt. */
            #if(DATA_COM_TX_INTERRUPT_ENABLED)
                DATA_COM_DisableTxInt();
            #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */

            DATA_COM_txBufferRead = 0u;
            DATA_COM_txBufferWrite = 0u;

            /* Enable Tx interrupt. */
            #if(DATA_COM_TX_INTERRUPT_ENABLED)
                DATA_COM_EnableTxInt();
            #endif /* End DATA_COM_TX_INTERRUPT_ENABLED */

        #endif /* End DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH */
    }


    /*******************************************************************************
    * Function Name: DATA_COM_SendBreak
    ********************************************************************************
    *
    * Summary:
    *  Write a Break command to the UART
    *
    * Parameters:
    *  uint8 retMode:  Wait mode,
    *   0 - Initialize registers for Break, sends the Break signal and return
    *       imediately.
    *   1 - Wait until Break sending is complete, reinitialize registers to normal
    *       transmission mode then return.
    *   2 - Reinitialize registers to normal transmission mode then return.
    *   3 - both steps: 0 and 1
    *       init registers for Break, send Break signal
    *       wait until Break sending is complete, reinit registers to normal
    *       transmission mode then return.
    *
    * Return:
    *  None.
    *
    * Global Variables:
    *  DATA_COM_initVar - checked to identify that the component has been
    *     initialized.
    *  tx_period - static variable, used for keeping TX period configuration.
    *
    * Reentrant:
    *  No.
    *
    * Theory:
    *  SendBreak function initializes registers to send 13-bit break signal. It is
    *  important to return the registers configuration to normal for continue 8-bit
    *  operation.
    *  Trere are 3 variants for this API usage:
    *  1) SendBreak(3) - function will send the Break signal and take care on the
    *     configuration returning. Funcition will block CPU untill transmition
    *     complete.
    *  2) User may want to use bloking time if UART configured to the low speed
    *     operation
    *     Emample for this case:
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     SendBreak(1);     - complete Break operation
    *  3) Same to 2) but user may want to init and use the interrupt for complete
    *     break operation.
    *     Example for this case:
    *     Init TX interrupt whith "TX - On TX Complete" parameter
    *     SendBreak(0);     - init Break signal transmition
    *         Add your code here to use CPU time
    *     When interrupt appear with UART_TX_STS_COMPLETE status:
    *     SendBreak(2);     - complete Break operation
    *
    * Side Effects:
    *   Uses static variable to keep registers configuration.
    *
    *******************************************************************************/
    void DATA_COM_SendBreak(uint8 retMode) 
    {

        /* If not Initialized then skip this function*/
        if(DATA_COM_initVar != 0u)
        {
            /*Set the Counter to 13-bits and transmit a 00 byte*/
            /*When that is done then reset the counter value back*/
            uint8 tmpStat;

            #if(DATA_COM_HD_ENABLED) /* Half Duplex mode*/

                if( (retMode == DATA_COM_SEND_BREAK) ||
                    (retMode == DATA_COM_SEND_WAIT_REINIT ) )
                {
                    /* CTRL_HD_SEND_BREAK - sends break bits in HD mode*/
                    DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() |
                                                          DATA_COM_CTRL_HD_SEND_BREAK);
                    /* Send zeros*/
                    DATA_COM_TXDATA_REG = 0u;

                    do /*wait until transmit starts*/
                    {
                        tmpStat = DATA_COM_TXSTATUS_REG;
                    }while((tmpStat & DATA_COM_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == DATA_COM_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == DATA_COM_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = DATA_COM_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & DATA_COM_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == DATA_COM_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == DATA_COM_REINIT) ||
                    (retMode == DATA_COM_SEND_WAIT_REINIT) )
                {
                    DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() &
                                                  (uint8)~DATA_COM_CTRL_HD_SEND_BREAK);
                }

            #else /* DATA_COM_HD_ENABLED Full Duplex mode */

                static uint8 tx_period;

                if( (retMode == DATA_COM_SEND_BREAK) ||
                    (retMode == DATA_COM_SEND_WAIT_REINIT) )
                {
                    /* CTRL_HD_SEND_BREAK - skip to send parity bit at Break signal in Full Duplex mode*/
                    #if( (DATA_COM_PARITY_TYPE != DATA_COM__B_UART__NONE_REVB) || \
                                        (DATA_COM_PARITY_TYPE_SW != 0u) )
                        DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() |
                                                              DATA_COM_CTRL_HD_SEND_BREAK);
                    #endif /* End DATA_COM_PARITY_TYPE != DATA_COM__B_UART__NONE_REVB  */

                    #if(DATA_COM_TXCLKGEN_DP)
                        tx_period = DATA_COM_TXBITCLKTX_COMPLETE_REG;
                        DATA_COM_TXBITCLKTX_COMPLETE_REG = DATA_COM_TXBITCTR_BREAKBITS;
                    #else
                        tx_period = DATA_COM_TXBITCTR_PERIOD_REG;
                        DATA_COM_TXBITCTR_PERIOD_REG = DATA_COM_TXBITCTR_BREAKBITS8X;
                    #endif /* End DATA_COM_TXCLKGEN_DP */

                    /* Send zeros*/
                    DATA_COM_TXDATA_REG = 0u;

                    do /* wait until transmit starts */
                    {
                        tmpStat = DATA_COM_TXSTATUS_REG;
                    }while((tmpStat & DATA_COM_TX_STS_FIFO_EMPTY) != 0u);
                }

                if( (retMode == DATA_COM_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == DATA_COM_SEND_WAIT_REINIT) )
                {
                    do /*wait until transmit complete*/
                    {
                        tmpStat = DATA_COM_TXSTATUS_REG;
                    }while(((uint8)~tmpStat & DATA_COM_TX_STS_COMPLETE) != 0u);
                }

                if( (retMode == DATA_COM_WAIT_FOR_COMPLETE_REINIT) ||
                    (retMode == DATA_COM_REINIT) ||
                    (retMode == DATA_COM_SEND_WAIT_REINIT) )
                {

                    #if(DATA_COM_TXCLKGEN_DP)
                        DATA_COM_TXBITCLKTX_COMPLETE_REG = tx_period;
                    #else
                        DATA_COM_TXBITCTR_PERIOD_REG = tx_period;
                    #endif /* End DATA_COM_TXCLKGEN_DP */

                    #if( (DATA_COM_PARITY_TYPE != DATA_COM__B_UART__NONE_REVB) || \
                         (DATA_COM_PARITY_TYPE_SW != 0u) )
                        DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() &
                                                      (uint8)~DATA_COM_CTRL_HD_SEND_BREAK);
                    #endif /* End DATA_COM_PARITY_TYPE != NONE */
                }
            #endif    /* End DATA_COM_HD_ENABLED */
        }
    }


    /*******************************************************************************
    * Function Name: DATA_COM_SetTxAddressMode
    ********************************************************************************
    *
    * Summary:
    *  Set the transmit addressing mode
    *
    * Parameters:
    *  addressMode: 0 -> Space
    *               1 -> Mark
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void DATA_COM_SetTxAddressMode(uint8 addressMode) 
    {
        /* Mark/Space sending enable*/
        if(addressMode != 0u)
        {
            #if( DATA_COM_CONTROL_REG_REMOVED == 0u )
                DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() |
                                                      DATA_COM_CTRL_MARK);
            #endif /* End DATA_COM_CONTROL_REG_REMOVED == 0u */
        }
        else
        {
            #if( DATA_COM_CONTROL_REG_REMOVED == 0u )
                DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() &
                                                    (uint8)~DATA_COM_CTRL_MARK);
            #endif /* End DATA_COM_CONTROL_REG_REMOVED == 0u */
        }
    }

#endif  /* EndDATA_COM_TX_ENABLED */

#if(DATA_COM_HD_ENABLED)


    /*******************************************************************************
    * Function Name: DATA_COM_LoadTxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Rx configuration if required and loads the
    *  Tx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Tx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART.
    *
    * Side Effects:
    *  Disable RX interrupt mask, when software buffer has been used.
    *
    *******************************************************************************/
    void DATA_COM_LoadTxConfig(void) 
    {
        #if((DATA_COM_RX_INTERRUPT_ENABLED) && (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            /* Disable RX interrupts before set TX configuration */
            DATA_COM_SetRxInterruptMode(0u);
        #endif /* DATA_COM_RX_INTERRUPT_ENABLED */

        DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() | DATA_COM_CTRL_HD_SEND);
        DATA_COM_RXBITCTR_PERIOD_REG = DATA_COM_HD_TXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(DATA_COM_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */
    }


    /*******************************************************************************
    * Function Name: DATA_COM_LoadRxConfig
    ********************************************************************************
    *
    * Summary:
    *  Unloads the Tx configuration if required and loads the
    *  Rx configuration. It is the users responsibility to ensure that any
    *  transaction is complete and it is safe to unload the Rx
    *  configuration.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    * Theory:
    *  Valid only for half duplex UART
    *
    * Side Effects:
    *  Set RX interrupt mask based on customizer settings, when software buffer
    *  has been used.
    *
    *******************************************************************************/
    void DATA_COM_LoadRxConfig(void) 
    {
        DATA_COM_WriteControlRegister(DATA_COM_ReadControlRegister() &
                                                (uint8)~DATA_COM_CTRL_HD_SEND);
        DATA_COM_RXBITCTR_PERIOD_REG = DATA_COM_HD_RXBITCTR_INIT;
        #if(CY_UDB_V0) /* Manually clear status register when mode has been changed */
            /* Clear status register */
            CY_GET_REG8(DATA_COM_RXSTATUS_PTR);
        #endif /* CY_UDB_V0 */

        #if((DATA_COM_RX_INTERRUPT_ENABLED) && (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
            /* Enable RX interrupt after set RX configuration */
            DATA_COM_SetRxInterruptMode(DATA_COM_INIT_RX_INTERRUPTS_MASK);
        #endif /* DATA_COM_RX_INTERRUPT_ENABLED */
    }

#endif  /* DATA_COM_HD_ENABLED */


/* [] END OF FILE */
