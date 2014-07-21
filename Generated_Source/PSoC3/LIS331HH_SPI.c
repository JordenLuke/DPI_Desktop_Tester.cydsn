/*******************************************************************************
* File Name: LIS331HH_SPI.c
* Version 2.40
*
* Description:
*  This file provides all API functionality of the SPI Master component.
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

#include "LIS331HH_SPI_PVT.h"

#if(LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED)
    volatile uint8 LIS331HH_SPI_txBuffer[LIS331HH_SPI_TX_BUFFER_SIZE] = {0u};
    volatile uint8 LIS331HH_SPI_txBufferFull;
    volatile uint8 LIS331HH_SPI_txBufferRead;
    volatile uint8 LIS331HH_SPI_txBufferWrite;
#endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) */

#if(LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED)
    volatile uint8 LIS331HH_SPI_rxBuffer[LIS331HH_SPI_RX_BUFFER_SIZE] = {0u};
    volatile uint8 LIS331HH_SPI_rxBufferFull;
    volatile uint8 LIS331HH_SPI_rxBufferRead;
    volatile uint8 LIS331HH_SPI_rxBufferWrite;
#endif /* (LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED) */

uint8 LIS331HH_SPI_initVar = 0u;

volatile uint8 LIS331HH_SPI_swStatusTx;
volatile uint8 LIS331HH_SPI_swStatusRx;


/*******************************************************************************
* Function Name: LIS331HH_SPI_Init
********************************************************************************
*
* Summary:
*  Inits/Restores default SPIM configuration provided with customizer.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*  When this function is called it initializes all of the necessary parameters
*  for execution. i.e. setting the initial interrupt mask, configuring the
*  interrupt service routine, configuring the bit-counter parameters and
*  clearing the FIFO and Status Register.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_Init(void) 
{
    /* Initialize the Bit counter */
    LIS331HH_SPI_COUNTER_PERIOD_REG = LIS331HH_SPI_BITCTR_INIT;

    /* Init TX ISR  */
    #if(0u != LIS331HH_SPI_INTERNAL_TX_INT_ENABLED)
        CyIntDisable         (LIS331HH_SPI_TX_ISR_NUMBER);
        CyIntSetPriority     (LIS331HH_SPI_TX_ISR_NUMBER,  LIS331HH_SPI_TX_ISR_PRIORITY);
        (void) CyIntSetVector(LIS331HH_SPI_TX_ISR_NUMBER, &LIS331HH_SPI_TX_ISR);
    #endif /* (0u != LIS331HH_SPI_INTERNAL_TX_INT_ENABLED) */

    /* Init RX ISR  */
    #if(0u != LIS331HH_SPI_INTERNAL_RX_INT_ENABLED)
        CyIntDisable         (LIS331HH_SPI_RX_ISR_NUMBER);
        CyIntSetPriority     (LIS331HH_SPI_RX_ISR_NUMBER,  LIS331HH_SPI_RX_ISR_PRIORITY);
        (void) CyIntSetVector(LIS331HH_SPI_RX_ISR_NUMBER, &LIS331HH_SPI_RX_ISR);
    #endif /* (0u != LIS331HH_SPI_INTERNAL_RX_INT_ENABLED) */

    /* Clear any stray data from the RX and TX FIFO */
    LIS331HH_SPI_ClearFIFO();

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

    (void) LIS331HH_SPI_ReadTxStatus(); /* Clear Tx status and swStatusTx */
    (void) LIS331HH_SPI_ReadRxStatus(); /* Clear Rx status and swStatusRx */

    /* Configure TX and RX interrupt mask */
    LIS331HH_SPI_TX_STATUS_MASK_REG = LIS331HH_SPI_TX_INIT_INTERRUPTS_MASK;
    LIS331HH_SPI_RX_STATUS_MASK_REG = LIS331HH_SPI_RX_INIT_INTERRUPTS_MASK;
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_Enable
********************************************************************************
*
* Summary:
*  Enable SPIM component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void LIS331HH_SPI_Enable(void) 
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    LIS331HH_SPI_COUNTER_CONTROL_REG |= LIS331HH_SPI_CNTR_ENABLE;
    LIS331HH_SPI_TX_STATUS_ACTL_REG  |= LIS331HH_SPI_INT_ENABLE;
    LIS331HH_SPI_RX_STATUS_ACTL_REG  |= LIS331HH_SPI_INT_ENABLE;
    CyExitCriticalSection(enableInterrupts);

    #if(0u != LIS331HH_SPI_INTERNAL_CLOCK)
        LIS331HH_SPI_IntClock_Enable();
    #endif /* (0u != LIS331HH_SPI_INTERNAL_CLOCK) */

    LIS331HH_SPI_EnableTxInt();
    LIS331HH_SPI_EnableRxInt();
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_Start
********************************************************************************
*
* Summary:
*  Initialize and Enable the SPI Master component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  LIS331HH_SPI_initVar - used to check initial configuration, modified on
*  first function call.
*
* Theory:
*  Enable the clock input to enable operation.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_Start(void) 
{
    if(0u == LIS331HH_SPI_initVar)
    {
        LIS331HH_SPI_Init();
        LIS331HH_SPI_initVar = 1u;
    }

    LIS331HH_SPI_Enable();
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_Stop
********************************************************************************
*
* Summary:
*  Disable the SPI Master component.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the clock input to enable operation.
*
*******************************************************************************/
void LIS331HH_SPI_Stop(void) 
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    LIS331HH_SPI_TX_STATUS_ACTL_REG &= ((uint8) ~LIS331HH_SPI_INT_ENABLE);
    LIS331HH_SPI_RX_STATUS_ACTL_REG &= ((uint8) ~LIS331HH_SPI_INT_ENABLE);
    CyExitCriticalSection(enableInterrupts);

    #if(0u != LIS331HH_SPI_INTERNAL_CLOCK)
        LIS331HH_SPI_IntClock_Disable();
    #endif /* (0u != LIS331HH_SPI_INTERNAL_CLOCK) */

    LIS331HH_SPI_DisableTxInt();
    LIS331HH_SPI_DisableRxInt();
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_EnableTxInt
********************************************************************************
*
* Summary:
*  Enable internal Tx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Enable the internal Tx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void LIS331HH_SPI_EnableTxInt(void) 
{
    #if(0u != LIS331HH_SPI_INTERNAL_TX_INT_ENABLED)
        CyIntEnable(LIS331HH_SPI_TX_ISR_NUMBER);
    #endif /* (0u != LIS331HH_SPI_INTERNAL_TX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_EnableRxInt
********************************************************************************
*
* Summary:
*  Enable internal Rx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Enable the internal Rx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void LIS331HH_SPI_EnableRxInt(void) 
{
    #if(0u != LIS331HH_SPI_INTERNAL_RX_INT_ENABLED)
        CyIntEnable(LIS331HH_SPI_RX_ISR_NUMBER);
    #endif /* (0u != LIS331HH_SPI_INTERNAL_RX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_DisableTxInt
********************************************************************************
*
* Summary:
*  Disable internal Tx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the internal Tx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void LIS331HH_SPI_DisableTxInt(void) 
{
    #if(0u != LIS331HH_SPI_INTERNAL_TX_INT_ENABLED)
        CyIntDisable(LIS331HH_SPI_TX_ISR_NUMBER);
    #endif /* (0u != LIS331HH_SPI_INTERNAL_TX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_DisableRxInt
********************************************************************************
*
* Summary:
*  Disable internal Rx interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the internal Rx interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void LIS331HH_SPI_DisableRxInt(void) 
{
    #if(0u != LIS331HH_SPI_INTERNAL_RX_INT_ENABLED)
        CyIntDisable(LIS331HH_SPI_RX_ISR_NUMBER);
    #endif /* (0u != LIS331HH_SPI_INTERNAL_RX_INT_ENABLED) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_SetTxInterruptMode
********************************************************************************
*
* Summary:
*  Configure which status bits trigger an interrupt event.
*
* Parameters:
*  intSrc: An or'd combination of the desired status bit masks (defined in the
*  header file).
*
* Return:
*  None.
*
* Theory:
*  Enables the output of specific status bits to the interrupt controller.
*
*******************************************************************************/
void LIS331HH_SPI_SetTxInterruptMode(uint8 intSrc) 
{
    LIS331HH_SPI_TX_STATUS_MASK_REG = intSrc;
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_SetRxInterruptMode
********************************************************************************
*
* Summary:
*  Configure which status bits trigger an interrupt event.
*
* Parameters:
*  intSrc: An or'd combination of the desired status bit masks (defined in the
*  header file).
*
* Return:
*  None.
*
* Theory:
*  Enables the output of specific status bits to the interrupt controller.
*
*******************************************************************************/
void LIS331HH_SPI_SetRxInterruptMode(uint8 intSrc) 
{
    LIS331HH_SPI_RX_STATUS_MASK_REG  = intSrc;
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_ReadTxStatus
********************************************************************************
*
* Summary:
*  Read the Tx status register for the component.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the Tx status register.
*
* Global variables:
*  LIS331HH_SPI_swStatusTx - used to store in software status register,
*  modified every function call - resets to zero.
*
* Theory:
*  Allows the user and the API to read the Tx status register for error
*  detection and flow control.
*
* Side Effects:
*  Clear Tx status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 LIS331HH_SPI_ReadTxStatus(void) 
{
    uint8 tmpStatus;

    #if(LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED)
        /* Disable TX interrupt to protect global veriables */
        LIS331HH_SPI_DisableTxInt();

        tmpStatus = LIS331HH_SPI_GET_STATUS_TX(LIS331HH_SPI_swStatusTx);
        LIS331HH_SPI_swStatusTx = 0u;

        LIS331HH_SPI_EnableTxInt();

    #else

        tmpStatus = LIS331HH_SPI_TX_STATUS_REG;

    #endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) */

    return(tmpStatus);
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_ReadRxStatus
********************************************************************************
*
* Summary:
*  Read the Rx status register for the component.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the Rx status register.
*
* Global variables:
*  LIS331HH_SPI_swStatusRx - used to store in software Rx status register,
*  modified every function call - resets to zero.
*
* Theory:
*  Allows the user and the API to read the Rx status register for error
*  detection and flow control.
*
* Side Effects:
*  Clear Rx status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 LIS331HH_SPI_ReadRxStatus(void) 
{
    uint8 tmpStatus;

    #if(LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED)
        /* Disable RX interrupt to protect global veriables */
        LIS331HH_SPI_DisableRxInt();

        tmpStatus = LIS331HH_SPI_GET_STATUS_RX(LIS331HH_SPI_swStatusRx);
        LIS331HH_SPI_swStatusRx = 0u;

        LIS331HH_SPI_EnableRxInt();

    #else

        tmpStatus = LIS331HH_SPI_RX_STATUS_REG;

    #endif /* (LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED) */

    return(tmpStatus);
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_WriteTxData
********************************************************************************
*
* Summary:
*  Write a byte of data to be sent across the SPI.
*
* Parameters:
*  txDataByte: The data value to send across the SPI.
*
* Return:
*  None.
*
* Global variables:
*  LIS331HH_SPI_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer, modified every function
*  call if TX Software Buffer is used.
*  LIS331HH_SPI_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer.
*  LIS331HH_SPI_txBuffer[LIS331HH_SPI_TX_BUFFER_SIZE] - used to store
*  data to sending, modified every function call if TX Software Buffer is used.
*
* Theory:
*  Allows the user to transmit any byte of data in a single transfer.
*
* Side Effects:
*  If this function is called again before the previous byte is finished then
*  the next byte will be appended to the transfer with no time between
*  the byte transfers. Clear Tx status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_WriteTxData(uint8 txData) 
{
    #if(LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED)

        uint8 tempStatus;
        uint8 tmpTxBufferRead;

        /* Block if TX buffer is FULL: don't overwrite */
        do
        {
            tmpTxBufferRead = LIS331HH_SPI_txBufferRead;
            if(0u == tmpTxBufferRead)
            {
                tmpTxBufferRead = (LIS331HH_SPI_TX_BUFFER_SIZE - 1u);
            }
            else
            {
                tmpTxBufferRead--;
            }

        }while(tmpTxBufferRead == LIS331HH_SPI_txBufferWrite);

        /* Disable TX interrupt to protect global veriables */
        LIS331HH_SPI_DisableTxInt();

        tempStatus = LIS331HH_SPI_GET_STATUS_TX(LIS331HH_SPI_swStatusTx);
        LIS331HH_SPI_swStatusTx = tempStatus;


        if((LIS331HH_SPI_txBufferRead == LIS331HH_SPI_txBufferWrite) &&
           (0u != (LIS331HH_SPI_swStatusTx & LIS331HH_SPI_STS_TX_FIFO_NOT_FULL)))
        {
            /* Add directly to the TX FIFO */
            CY_SET_REG8(LIS331HH_SPI_TXDATA_PTR, txData);
        }
        else
        {
            /* Add to the TX software buffer */
            LIS331HH_SPI_txBufferWrite++;
            if(LIS331HH_SPI_txBufferWrite >= LIS331HH_SPI_TX_BUFFER_SIZE)
            {
                LIS331HH_SPI_txBufferWrite = 0u;
            }

            if(LIS331HH_SPI_txBufferWrite == LIS331HH_SPI_txBufferRead)
            {
                LIS331HH_SPI_txBufferRead++;
                if(LIS331HH_SPI_txBufferRead >= LIS331HH_SPI_TX_BUFFER_SIZE)
                {
                    LIS331HH_SPI_txBufferRead = 0u;
                }
                LIS331HH_SPI_txBufferFull = 1u;
            }

            LIS331HH_SPI_txBuffer[LIS331HH_SPI_txBufferWrite] = txData;

            LIS331HH_SPI_TX_STATUS_MASK_REG |= LIS331HH_SPI_STS_TX_FIFO_NOT_FULL;
        }

        LIS331HH_SPI_EnableTxInt();

    #else

        while(0u == (LIS331HH_SPI_TX_STATUS_REG & LIS331HH_SPI_STS_TX_FIFO_NOT_FULL))
        {
            ; /* Wait for room in FIFO */
        }

        /* Put byte in TX FIFO */
        CY_SET_REG8(LIS331HH_SPI_TXDATA_PTR, txData);

    #endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_ReadRxData
********************************************************************************
*
* Summary:
*  Read the next byte of data received across the SPI.
*
* Parameters:
*  None.
*
* Return:
*  The next byte of data read from the FIFO.
*
* Global variables:
*  LIS331HH_SPI_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer.
*  LIS331HH_SPI_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified every function
*  call if RX Software Buffer is used.
*  LIS331HH_SPI_rxBuffer[LIS331HH_SPI_RX_BUFFER_SIZE] - used to store
*  received data.
*
* Theory:
*  Allows the user to read a byte of data received.
*
* Side Effects:
*  Will return invalid data if the FIFO is empty. The user should Call
*  GetRxBufferSize() and if it returns a non-zero value then it is safe to call
*  ReadByte() function.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 LIS331HH_SPI_ReadRxData(void) 
{
    uint8 rxData;

    #if(LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED)

        /* Disable RX interrupt to protect global veriables */
        LIS331HH_SPI_DisableRxInt();

        if(LIS331HH_SPI_rxBufferRead != LIS331HH_SPI_rxBufferWrite)
        {
            if(0u == LIS331HH_SPI_rxBufferFull)
            {
                LIS331HH_SPI_rxBufferRead++;
                if(LIS331HH_SPI_rxBufferRead >= LIS331HH_SPI_RX_BUFFER_SIZE)
                {
                    LIS331HH_SPI_rxBufferRead = 0u;
                }
            }
            else
            {
                LIS331HH_SPI_rxBufferFull = 0u;
            }
        }

        rxData = LIS331HH_SPI_rxBuffer[LIS331HH_SPI_rxBufferRead];

        LIS331HH_SPI_EnableRxInt();

    #else

        rxData = CY_GET_REG8(LIS331HH_SPI_RXDATA_PTR);

    #endif /* (LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED) */

    return(rxData);
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_GetRxBufferSize
********************************************************************************
*
* Summary:
*  Returns the number of bytes/words of data currently held in the RX buffer.
*  If RX Software Buffer not used then function return 0 if FIFO empty or 1 if
*  FIFO not empty. In another case function return size of RX Software Buffer.
*
* Parameters:
*  None.
*
* Return:
*  Integer count of the number of bytes/words in the RX buffer.
*
* Global variables:
*  LIS331HH_SPI_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer.
*  LIS331HH_SPI_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer.
*
* Side Effects:
*  Clear status register of the component.
*
*******************************************************************************/
uint8 LIS331HH_SPI_GetRxBufferSize(void) 
{
    uint8 size;

    #if(LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED)

        /* Disable RX interrupt to protect global veriables */
        LIS331HH_SPI_DisableRxInt();

        if(LIS331HH_SPI_rxBufferRead == LIS331HH_SPI_rxBufferWrite)
        {
            size = 0u;
        }
        else if(LIS331HH_SPI_rxBufferRead < LIS331HH_SPI_rxBufferWrite)
        {
            size = (LIS331HH_SPI_rxBufferWrite - LIS331HH_SPI_rxBufferRead);
        }
        else
        {
            size = (LIS331HH_SPI_RX_BUFFER_SIZE - LIS331HH_SPI_rxBufferRead) + LIS331HH_SPI_rxBufferWrite;
        }

        LIS331HH_SPI_EnableRxInt();

    #else

        /* We can only know if there is data in the RX FIFO */
        size = (0u != (LIS331HH_SPI_RX_STATUS_REG & LIS331HH_SPI_STS_RX_FIFO_NOT_EMPTY)) ? 1u : 0u;

    #endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) */

    return(size);
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_GetTxBufferSize
********************************************************************************
*
* Summary:
*  Returns the number of bytes/words of data currently held in the TX buffer.
*  If TX Software Buffer not used then function return 0 - if FIFO empty, 1 - if
*  FIFO not full, 4 - if FIFO full. In another case function return size of TX
*  Software Buffer.
*
* Parameters:
*  None.
*
* Return:
*  Integer count of the number of bytes/words in the TX buffer.
*
* Global variables:
*  LIS331HH_SPI_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer.
*  LIS331HH_SPI_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer.
*
* Side Effects:
*  Clear status register of the component.
*
*******************************************************************************/
uint8  LIS331HH_SPI_GetTxBufferSize(void) 
{
    uint8 size;

    #if(LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED)
        /* Disable TX interrupt to protect global veriables */
        LIS331HH_SPI_DisableTxInt();

        if(LIS331HH_SPI_txBufferRead == LIS331HH_SPI_txBufferWrite)
        {
            size = 0u;
        }
        else if(LIS331HH_SPI_txBufferRead < LIS331HH_SPI_txBufferWrite)
        {
            size = (LIS331HH_SPI_txBufferWrite - LIS331HH_SPI_txBufferRead);
        }
        else
        {
            size = (LIS331HH_SPI_TX_BUFFER_SIZE - LIS331HH_SPI_txBufferRead) + LIS331HH_SPI_txBufferWrite;
        }

        LIS331HH_SPI_EnableTxInt();

    #else

        size = LIS331HH_SPI_TX_STATUS_REG;

        if(0u != (size & LIS331HH_SPI_STS_TX_FIFO_EMPTY))
        {
            size = 0u;
        }
        else if(0u != (size & LIS331HH_SPI_STS_TX_FIFO_NOT_FULL))
        {
            size = 1u;
        }
        else
        {
            size = LIS331HH_SPI_FIFO_SIZE;
        }

    #endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) */

    return(size);
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_ClearRxBuffer
********************************************************************************
*
* Summary:
*  Clear the RX RAM buffer by setting the read and write pointers both to zero.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  LIS331HH_SPI_rxBufferWrite - used for the account of the bytes which
*  have been written down in the RX software buffer, modified every function
*  call - resets to zero.
*  LIS331HH_SPI_rxBufferRead - used for the account of the bytes which
*  have been read from the RX software buffer, modified every function call -
*  resets to zero.
*
* Theory:
*  Setting the pointers to zero makes the system believe there is no data to
*  read and writing will resume at address 0 overwriting any data that may have
*  remained in the RAM.
*
* Side Effects:
*  Any received data not read from the RAM buffer will be lost when overwritten.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_ClearRxBuffer(void) 
{
    /* Clear Hardware RX FIFO */
    while(0u !=(LIS331HH_SPI_RX_STATUS_REG & LIS331HH_SPI_STS_RX_FIFO_NOT_EMPTY))
    {
        (void) CY_GET_REG8(LIS331HH_SPI_RXDATA_PTR);
    }

    #if(LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED)
        /* Disable RX interrupt to protect global veriables */
        LIS331HH_SPI_DisableRxInt();

        LIS331HH_SPI_rxBufferFull  = 0u;
        LIS331HH_SPI_rxBufferRead  = 0u;
        LIS331HH_SPI_rxBufferWrite = 0u;

        LIS331HH_SPI_EnableRxInt();
    #endif /* (LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED) */
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_ClearTxBuffer
********************************************************************************
*
* Summary:
*  Clear the TX RAM buffer by setting the read and write pointers both to zero.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Global variables:
*  LIS331HH_SPI_txBufferWrite - used for the account of the bytes which
*  have been written down in the TX software buffer, modified every function
*  call - resets to zero.
*  LIS331HH_SPI_txBufferRead - used for the account of the bytes which
*  have been read from the TX software buffer, modified every function call -
*  resets to zero.
*
* Theory:
*  Setting the pointers to zero makes the system believe there is no data to
*  read and writing will resume at address 0 overwriting any data that may have
*  remained in the RAM.
*
* Side Effects:
*  Any data not yet transmitted from the RAM buffer will be lost when
*  overwritten.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_ClearTxBuffer(void) 
{
    uint8 enableInterrupts;

    enableInterrupts = CyEnterCriticalSection();
    /* Clear TX FIFO */
    LIS331HH_SPI_AUX_CONTROL_DP0_REG |= ((uint8)  LIS331HH_SPI_TX_FIFO_CLR);
    LIS331HH_SPI_AUX_CONTROL_DP0_REG &= ((uint8) ~LIS331HH_SPI_TX_FIFO_CLR);

    #if(LIS331HH_SPI_USE_SECOND_DATAPATH)
        /* Clear TX FIFO for 2nd Datapath */
        LIS331HH_SPI_AUX_CONTROL_DP1_REG |= ((uint8)  LIS331HH_SPI_TX_FIFO_CLR);
        LIS331HH_SPI_AUX_CONTROL_DP1_REG &= ((uint8) ~LIS331HH_SPI_TX_FIFO_CLR);
    #endif /* (LIS331HH_SPI_USE_SECOND_DATAPATH) */
    CyExitCriticalSection(enableInterrupts);

    #if(LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED)
        /* Disable TX interrupt to protect global veriables */
        LIS331HH_SPI_DisableTxInt();

        LIS331HH_SPI_txBufferFull  = 0u;
        LIS331HH_SPI_txBufferRead  = 0u;
        LIS331HH_SPI_txBufferWrite = 0u;

        /* Buffer is EMPTY: disable TX FIFO NOT FULL interrupt */
        LIS331HH_SPI_TX_STATUS_MASK_REG &= ((uint8) ~LIS331HH_SPI_STS_TX_FIFO_NOT_FULL);

        LIS331HH_SPI_EnableTxInt();
    #endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) */
}


#if(0u != LIS331HH_SPI_BIDIRECTIONAL_MODE)
    /*******************************************************************************
    * Function Name: LIS331HH_SPI_TxEnable
    ********************************************************************************
    *
    * Summary:
    *  If the SPI master is configured to use a single bi-directional pin then this
    *  will set the bi-directional pin to transmit.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void LIS331HH_SPI_TxEnable(void) 
    {
        LIS331HH_SPI_CONTROL_REG |= LIS331HH_SPI_CTRL_TX_SIGNAL_EN;
    }


    /*******************************************************************************
    * Function Name: LIS331HH_SPI_TxDisable
    ********************************************************************************
    *
    * Summary:
    *  If the SPI master is configured to use a single bi-directional pin then this
    *  will set the bi-directional pin to receive.
    *
    * Parameters:
    *  None.
    *
    * Return:
    *  None.
    *
    *******************************************************************************/
    void LIS331HH_SPI_TxDisable(void) 
    {
        LIS331HH_SPI_CONTROL_REG &= ((uint8) ~LIS331HH_SPI_CTRL_TX_SIGNAL_EN);
    }

#endif /* (0u != LIS331HH_SPI_BIDIRECTIONAL_MODE) */


/*******************************************************************************
* Function Name: LIS331HH_SPI_PutArray
********************************************************************************
*
* Summary:
*  Write available data from ROM/RAM to the TX buffer while space is available
*  in the TX buffer. Keep trying until all data is passed to the TX buffer.
*
* Parameters:
*  *buffer: Pointer to the location in RAM containing the data to send
*  byteCount: The number of bytes to move to the transmit buffer.
*
* Return:
*  None.
*
* Side Effects:
*  Will stay in this routine until all data has been sent.  May get locked in
*  this loop if data is not being initiated by the master if there is not
*  enough room in the TX FIFO.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void LIS331HH_SPI_PutArray(const uint8 buffer[], uint8 byteCount)
                                                                          
{
    uint8 bufIndex;

    bufIndex = 0u;

    while(byteCount > 0u)
    {
        LIS331HH_SPI_WriteTxData(buffer[bufIndex]);
        bufIndex++;
        byteCount--;
    }
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_ClearFIFO
********************************************************************************
*
* Summary:
*  Clear the RX and TX FIFO's of all data for a fresh start.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Side Effects:
*  Clear status register of the component.
*
*******************************************************************************/
void LIS331HH_SPI_ClearFIFO(void) 
{
    uint8 enableInterrupts;

    /* Clear Hardware RX FIFO */
    while(0u !=(LIS331HH_SPI_RX_STATUS_REG & LIS331HH_SPI_STS_RX_FIFO_NOT_EMPTY))
    {
        (void) CY_GET_REG8(LIS331HH_SPI_RXDATA_PTR);
    }

    enableInterrupts = CyEnterCriticalSection();
    /* Clear TX FIFO */
    LIS331HH_SPI_AUX_CONTROL_DP0_REG |= ((uint8)  LIS331HH_SPI_TX_FIFO_CLR);
    LIS331HH_SPI_AUX_CONTROL_DP0_REG &= ((uint8) ~LIS331HH_SPI_TX_FIFO_CLR);

    #if(LIS331HH_SPI_USE_SECOND_DATAPATH)
        /* Clear TX FIFO for 2nd Datapath */
        LIS331HH_SPI_AUX_CONTROL_DP1_REG |= ((uint8)  LIS331HH_SPI_TX_FIFO_CLR);
        LIS331HH_SPI_AUX_CONTROL_DP1_REG &= ((uint8) ~LIS331HH_SPI_TX_FIFO_CLR);
    #endif /* (LIS331HH_SPI_USE_SECOND_DATAPATH) */
    CyExitCriticalSection(enableInterrupts);
}


/* Following functions are for version Compatibility, they are obsolete.
*  Please do not use it in new projects.
*/


/*******************************************************************************
* Function Name: LIS331HH_SPI_EnableInt
********************************************************************************
*
* Summary:
*  Enable internal interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Enable the internal interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void LIS331HH_SPI_EnableInt(void) 
{
    LIS331HH_SPI_EnableRxInt();
    LIS331HH_SPI_EnableTxInt();
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_DisableInt
********************************************************************************
*
* Summary:
*  Disable internal interrupt generation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
* Theory:
*  Disable the internal interrupt output -or- the interrupt component itself.
*
*******************************************************************************/
void LIS331HH_SPI_DisableInt(void) 
{
    LIS331HH_SPI_DisableTxInt();
    LIS331HH_SPI_DisableRxInt();
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_SetInterruptMode
********************************************************************************
*
* Summary:
*  Configure which status bits trigger an interrupt event.
*
* Parameters:
*  intSrc: An or'd combination of the desired status bit masks (defined in the
*  header file).
*
* Return:
*  None.
*
* Theory:
*  Enables the output of specific status bits to the interrupt controller.
*
*******************************************************************************/
void LIS331HH_SPI_SetInterruptMode(uint8 intSrc) 
{
    LIS331HH_SPI_TX_STATUS_MASK_REG  = (intSrc & ((uint8) ~LIS331HH_SPI_STS_SPI_IDLE));
    LIS331HH_SPI_RX_STATUS_MASK_REG  =  intSrc;
}


/*******************************************************************************
* Function Name: LIS331HH_SPI_ReadStatus
********************************************************************************
*
* Summary:
*  Read the status register for the component.
*
* Parameters:
*  None.
*
* Return:
*  Contents of the status register.
*
* Global variables:
*  LIS331HH_SPI_swStatus - used to store in software status register,
*  modified every function call - resets to zero.
*
* Theory:
*  Allows the user and the API to read the status register for error detection
*  and flow control.
*
* Side Effects:
*  Clear status register of the component.
*
* Reentrant:
*  No.
*
*******************************************************************************/
uint8 LIS331HH_SPI_ReadStatus(void) 
{
    uint8 tmpStatus;

    #if(LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED || LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED)

        LIS331HH_SPI_DisableInt();

        tmpStatus  = LIS331HH_SPI_GET_STATUS_RX(LIS331HH_SPI_swStatusRx);
        tmpStatus |= LIS331HH_SPI_GET_STATUS_TX(LIS331HH_SPI_swStatusTx);
        tmpStatus &= ((uint8) ~LIS331HH_SPI_STS_SPI_IDLE);

        LIS331HH_SPI_swStatusTx = 0u;
        LIS331HH_SPI_swStatusRx = 0u;

        LIS331HH_SPI_EnableInt();

    #else

        tmpStatus  = LIS331HH_SPI_RX_STATUS_REG;
        tmpStatus |= LIS331HH_SPI_TX_STATUS_REG;
        tmpStatus &= ((uint8) ~LIS331HH_SPI_STS_SPI_IDLE);

    #endif /* (LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED || LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED) */

    return(tmpStatus);
}


/* [] END OF FILE */
