/*******************************************************************************
* File Name: DATA_COM.h
* Version 2.30
*
* Description:
*  Contains the function prototypes and constants available to the UART
*  user module.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#if !defined(CY_UART_DATA_COM_H)
#define CY_UART_DATA_COM_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define DATA_COM_RX_ENABLED                     (1u)
#define DATA_COM_TX_ENABLED                     (1u)
#define DATA_COM_HD_ENABLED                     (0u)
#define DATA_COM_RX_INTERRUPT_ENABLED           (0u)
#define DATA_COM_TX_INTERRUPT_ENABLED           (0u)
#define DATA_COM_INTERNAL_CLOCK_USED            (1u)
#define DATA_COM_RXHW_ADDRESS_ENABLED           (0u)
#define DATA_COM_OVER_SAMPLE_COUNT              (8u)
#define DATA_COM_PARITY_TYPE                    (0u)
#define DATA_COM_PARITY_TYPE_SW                 (0u)
#define DATA_COM_BREAK_DETECT                   (0u)
#define DATA_COM_BREAK_BITS_TX                  (13u)
#define DATA_COM_BREAK_BITS_RX                  (13u)
#define DATA_COM_TXCLKGEN_DP                    (1u)
#define DATA_COM_USE23POLLING                   (1u)
#define DATA_COM_FLOW_CONTROL                   (0u)
#define DATA_COM_CLK_FREQ                       (0u)
#define DATA_COM_TXBUFFERSIZE                   (4u)
#define DATA_COM_RXBUFFERSIZE                   (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define DATA_COM_CONTROL_REG_REMOVED            (0u)
#else
    #define DATA_COM_CONTROL_REG_REMOVED            (1u)
#endif /* End DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct DATA_COM_backupStruct_
{
    uint8 enableState;

    #if(DATA_COM_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End DATA_COM_CONTROL_REG_REMOVED */
    #if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (DATA_COM_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End DATA_COM_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED)*/

    #if(DATA_COM_TX_ENABLED)
        #if(DATA_COM_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End DATA_COM_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End DATA_COM_TX_ENABLED */
} DATA_COM_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void DATA_COM_Start(void) ;
void DATA_COM_Stop(void) ;
uint8 DATA_COM_ReadControlRegister(void) ;
void DATA_COM_WriteControlRegister(uint8 control) ;

void DATA_COM_Init(void) ;
void DATA_COM_Enable(void) ;
void DATA_COM_SaveConfig(void) ;
void DATA_COM_RestoreConfig(void) ;
void DATA_COM_Sleep(void) ;
void DATA_COM_Wakeup(void) ;

/* Only if RX is enabled */
#if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )

    #if(DATA_COM_RX_INTERRUPT_ENABLED)
        void  DATA_COM_EnableRxInt(void) ;
        void  DATA_COM_DisableRxInt(void) ;
        CY_ISR_PROTO(DATA_COM_RXISR);
    #endif /* DATA_COM_RX_INTERRUPT_ENABLED */

    void DATA_COM_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void DATA_COM_SetRxAddress1(uint8 address) ;
    void DATA_COM_SetRxAddress2(uint8 address) ;

    void  DATA_COM_SetRxInterruptMode(uint8 intSrc) ;
    uint8 DATA_COM_ReadRxData(void) ;
    uint8 DATA_COM_ReadRxStatus(void) ;
    uint8 DATA_COM_GetChar(void) ;
    uint16 DATA_COM_GetByte(void) ;
    uint8 DATA_COM_GetRxBufferSize(void)
                                                            ;
    void DATA_COM_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define DATA_COM_GetRxInterruptSource   DATA_COM_ReadRxStatus

#endif /* End (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) */

/* Only if TX is enabled */
#if(DATA_COM_TX_ENABLED || DATA_COM_HD_ENABLED)

    #if(DATA_COM_TX_INTERRUPT_ENABLED)
        void DATA_COM_EnableTxInt(void) ;
        void DATA_COM_DisableTxInt(void) ;
        CY_ISR_PROTO(DATA_COM_TXISR);
    #endif /* DATA_COM_TX_INTERRUPT_ENABLED */

    void DATA_COM_SetTxInterruptMode(uint8 intSrc) ;
    void DATA_COM_WriteTxData(uint8 txDataByte) ;
    uint8 DATA_COM_ReadTxStatus(void) ;
    void DATA_COM_PutChar(uint8 txDataByte) ;
    void DATA_COM_PutString(const char8 string[]) ;
    void DATA_COM_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void DATA_COM_PutCRLF(uint8 txDataByte) ;
    void DATA_COM_ClearTxBuffer(void) ;
    void DATA_COM_SetTxAddressMode(uint8 addressMode) ;
    void DATA_COM_SendBreak(uint8 retMode) ;
    uint8 DATA_COM_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define DATA_COM_PutStringConst         DATA_COM_PutString
    #define DATA_COM_PutArrayConst          DATA_COM_PutArray
    #define DATA_COM_GetTxInterruptSource   DATA_COM_ReadTxStatus

#endif /* End DATA_COM_TX_ENABLED || DATA_COM_HD_ENABLED */

#if(DATA_COM_HD_ENABLED)
    void DATA_COM_LoadRxConfig(void) ;
    void DATA_COM_LoadTxConfig(void) ;
#endif /* End DATA_COM_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_DATA_COM) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    DATA_COM_CyBtldrCommStart(void) CYSMALL ;
    void    DATA_COM_CyBtldrCommStop(void) CYSMALL ;
    void    DATA_COM_CyBtldrCommReset(void) CYSMALL ;
    cystatus DATA_COM_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus DATA_COM_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_DATA_COM)
        #define CyBtldrCommStart    DATA_COM_CyBtldrCommStart
        #define CyBtldrCommStop     DATA_COM_CyBtldrCommStop
        #define CyBtldrCommReset    DATA_COM_CyBtldrCommReset
        #define CyBtldrCommWrite    DATA_COM_CyBtldrCommWrite
        #define CyBtldrCommRead     DATA_COM_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_DATA_COM) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define DATA_COM_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define DATA_COM_SET_SPACE                              (0x00u)
#define DATA_COM_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (DATA_COM_TX_ENABLED) || (DATA_COM_HD_ENABLED) )
    #if(DATA_COM_TX_INTERRUPT_ENABLED)
        #define DATA_COM_TX_VECT_NUM            (uint8)DATA_COM_TXInternalInterrupt__INTC_NUMBER
        #define DATA_COM_TX_PRIOR_NUM           (uint8)DATA_COM_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* DATA_COM_TX_INTERRUPT_ENABLED */
    #if(DATA_COM_TX_ENABLED)
        #define DATA_COM_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define DATA_COM_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define DATA_COM_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define DATA_COM_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* DATA_COM_TX_ENABLED */
    #if(DATA_COM_HD_ENABLED)
        #define DATA_COM_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define DATA_COM_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define DATA_COM_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define DATA_COM_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* DATA_COM_HD_ENABLED */
    #define DATA_COM_TX_STS_COMPLETE            (uint8)(0x01u << DATA_COM_TX_STS_COMPLETE_SHIFT)
    #define DATA_COM_TX_STS_FIFO_EMPTY          (uint8)(0x01u << DATA_COM_TX_STS_FIFO_EMPTY_SHIFT)
    #define DATA_COM_TX_STS_FIFO_FULL           (uint8)(0x01u << DATA_COM_TX_STS_FIFO_FULL_SHIFT)
    #define DATA_COM_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << DATA_COM_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (DATA_COM_TX_ENABLED) || (DATA_COM_HD_ENABLED)*/

#if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
    #if(DATA_COM_RX_INTERRUPT_ENABLED)
        #define DATA_COM_RX_VECT_NUM            (uint8)DATA_COM_RXInternalInterrupt__INTC_NUMBER
        #define DATA_COM_RX_PRIOR_NUM           (uint8)DATA_COM_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* DATA_COM_RX_INTERRUPT_ENABLED */
    #define DATA_COM_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define DATA_COM_RX_STS_BREAK_SHIFT             (0x01u)
    #define DATA_COM_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define DATA_COM_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define DATA_COM_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define DATA_COM_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define DATA_COM_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define DATA_COM_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define DATA_COM_RX_STS_MRKSPC           (uint8)(0x01u << DATA_COM_RX_STS_MRKSPC_SHIFT)
    #define DATA_COM_RX_STS_BREAK            (uint8)(0x01u << DATA_COM_RX_STS_BREAK_SHIFT)
    #define DATA_COM_RX_STS_PAR_ERROR        (uint8)(0x01u << DATA_COM_RX_STS_PAR_ERROR_SHIFT)
    #define DATA_COM_RX_STS_STOP_ERROR       (uint8)(0x01u << DATA_COM_RX_STS_STOP_ERROR_SHIFT)
    #define DATA_COM_RX_STS_OVERRUN          (uint8)(0x01u << DATA_COM_RX_STS_OVERRUN_SHIFT)
    #define DATA_COM_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << DATA_COM_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define DATA_COM_RX_STS_ADDR_MATCH       (uint8)(0x01u << DATA_COM_RX_STS_ADDR_MATCH_SHIFT)
    #define DATA_COM_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << DATA_COM_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define DATA_COM_RX_HW_MASK                     (0x7Fu)
#endif /* End (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) */

/* Control Register definitions */
#define DATA_COM_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define DATA_COM_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define DATA_COM_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define DATA_COM_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define DATA_COM_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define DATA_COM_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define DATA_COM_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define DATA_COM_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define DATA_COM_CTRL_HD_SEND               (uint8)(0x01u << DATA_COM_CTRL_HD_SEND_SHIFT)
#define DATA_COM_CTRL_HD_SEND_BREAK         (uint8)(0x01u << DATA_COM_CTRL_HD_SEND_BREAK_SHIFT)
#define DATA_COM_CTRL_MARK                  (uint8)(0x01u << DATA_COM_CTRL_MARK_SHIFT)
#define DATA_COM_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << DATA_COM_CTRL_PARITY_TYPE0_SHIFT)
#define DATA_COM_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << DATA_COM_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define DATA_COM_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define DATA_COM_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define DATA_COM_SEND_BREAK                         (0x00u)
#define DATA_COM_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define DATA_COM_REINIT                             (0x02u)
#define DATA_COM_SEND_WAIT_REINIT                   (0x03u)

#define DATA_COM_OVER_SAMPLE_8                      (8u)
#define DATA_COM_OVER_SAMPLE_16                     (16u)

#define DATA_COM_BIT_CENTER                         (DATA_COM_OVER_SAMPLE_COUNT - 1u)

#define DATA_COM_FIFO_LENGTH                        (4u)
#define DATA_COM_NUMBER_OF_START_BIT                (1u)
#define DATA_COM_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define DATA_COM_TXBITCTR_BREAKBITS8X   ((DATA_COM_BREAK_BITS_TX * DATA_COM_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define DATA_COM_TXBITCTR_BREAKBITS ((DATA_COM_BREAK_BITS_TX * DATA_COM_OVER_SAMPLE_COUNT) - 1u)

#define DATA_COM_HALF_BIT_COUNT   \
                            (((DATA_COM_OVER_SAMPLE_COUNT / 2u) + (DATA_COM_USE23POLLING * 1u)) - 2u)
#if (DATA_COM_OVER_SAMPLE_COUNT == DATA_COM_OVER_SAMPLE_8)
    #define DATA_COM_HD_TXBITCTR_INIT   (((DATA_COM_BREAK_BITS_TX + \
                            DATA_COM_NUMBER_OF_START_BIT) * DATA_COM_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define DATA_COM_RXBITCTR_INIT  ((((DATA_COM_BREAK_BITS_RX + DATA_COM_NUMBER_OF_START_BIT) \
                            * DATA_COM_OVER_SAMPLE_COUNT) + DATA_COM_HALF_BIT_COUNT) - 1u)


#else /* DATA_COM_OVER_SAMPLE_COUNT == DATA_COM_OVER_SAMPLE_16 */
    #define DATA_COM_HD_TXBITCTR_INIT   ((8u * DATA_COM_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define DATA_COM_RXBITCTR_INIT      (((7u * DATA_COM_OVER_SAMPLE_COUNT) - 1u) + \
                                                      DATA_COM_HALF_BIT_COUNT)
#endif /* End DATA_COM_OVER_SAMPLE_COUNT */
#define DATA_COM_HD_RXBITCTR_INIT                   DATA_COM_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 DATA_COM_initVar;
#if( DATA_COM_TX_ENABLED && (DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
    extern volatile uint8 DATA_COM_txBuffer[DATA_COM_TXBUFFERSIZE];
    extern volatile uint8 DATA_COM_txBufferRead;
    extern uint8 DATA_COM_txBufferWrite;
#endif /* End DATA_COM_TX_ENABLED */
#if( ( DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED ) && \
     (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH) )
    extern volatile uint8 DATA_COM_rxBuffer[DATA_COM_RXBUFFERSIZE];
    extern volatile uint8 DATA_COM_rxBufferRead;
    extern volatile uint8 DATA_COM_rxBufferWrite;
    extern volatile uint8 DATA_COM_rxBufferLoopDetect;
    extern volatile uint8 DATA_COM_rxBufferOverflow;
    #if (DATA_COM_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 DATA_COM_rxAddressMode;
        extern volatile uint8 DATA_COM_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End DATA_COM_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define DATA_COM__B_UART__AM_SW_BYTE_BYTE 1
#define DATA_COM__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define DATA_COM__B_UART__AM_HW_BYTE_BY_BYTE 3
#define DATA_COM__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define DATA_COM__B_UART__AM_NONE 0

#define DATA_COM__B_UART__NONE_REVB 0
#define DATA_COM__B_UART__EVEN_REVB 1
#define DATA_COM__B_UART__ODD_REVB 2
#define DATA_COM__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define DATA_COM_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define DATA_COM_NUMBER_OF_STOP_BITS    (1u)

#if (DATA_COM_RXHW_ADDRESS_ENABLED)
    #define DATA_COM_RXADDRESSMODE      (0u)
    #define DATA_COM_RXHWADDRESS1       (0u)
    #define DATA_COM_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define DATA_COM_RXAddressMode      DATA_COM_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define DATA_COM_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << DATA_COM_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << DATA_COM_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << DATA_COM_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << DATA_COM_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << DATA_COM_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << DATA_COM_RX_STS_BREAK_SHIFT) \
                                        | (0 << DATA_COM_RX_STS_OVERRUN_SHIFT))

#define DATA_COM_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << DATA_COM_TX_STS_COMPLETE_SHIFT) \
                                        | (1 << DATA_COM_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << DATA_COM_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << DATA_COM_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define DATA_COM_CONTROL_REG \
                            (* (reg8 *) DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define DATA_COM_CONTROL_PTR \
                            (  (reg8 *) DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(DATA_COM_TX_ENABLED)
    #define DATA_COM_TXDATA_REG          (* (reg8 *) DATA_COM_BUART_sTX_TxShifter_u0__F0_REG)
    #define DATA_COM_TXDATA_PTR          (  (reg8 *) DATA_COM_BUART_sTX_TxShifter_u0__F0_REG)
    #define DATA_COM_TXDATA_AUX_CTL_REG  (* (reg8 *) DATA_COM_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define DATA_COM_TXDATA_AUX_CTL_PTR  (  (reg8 *) DATA_COM_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define DATA_COM_TXSTATUS_REG        (* (reg8 *) DATA_COM_BUART_sTX_TxSts__STATUS_REG)
    #define DATA_COM_TXSTATUS_PTR        (  (reg8 *) DATA_COM_BUART_sTX_TxSts__STATUS_REG)
    #define DATA_COM_TXSTATUS_MASK_REG   (* (reg8 *) DATA_COM_BUART_sTX_TxSts__MASK_REG)
    #define DATA_COM_TXSTATUS_MASK_PTR   (  (reg8 *) DATA_COM_BUART_sTX_TxSts__MASK_REG)
    #define DATA_COM_TXSTATUS_ACTL_REG   (* (reg8 *) DATA_COM_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define DATA_COM_TXSTATUS_ACTL_PTR   (  (reg8 *) DATA_COM_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(DATA_COM_TXCLKGEN_DP)
        #define DATA_COM_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define DATA_COM_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define DATA_COM_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define DATA_COM_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define DATA_COM_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define DATA_COM_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define DATA_COM_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define DATA_COM_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define DATA_COM_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define DATA_COM_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) DATA_COM_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* DATA_COM_TXCLKGEN_DP */

#endif /* End DATA_COM_TX_ENABLED */

#if(DATA_COM_HD_ENABLED)

    #define DATA_COM_TXDATA_REG             (* (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__F1_REG )
    #define DATA_COM_TXDATA_PTR             (  (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__F1_REG )
    #define DATA_COM_TXDATA_AUX_CTL_REG     (* (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define DATA_COM_TXDATA_AUX_CTL_PTR     (  (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define DATA_COM_TXSTATUS_REG           (* (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_REG )
    #define DATA_COM_TXSTATUS_PTR           (  (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_REG )
    #define DATA_COM_TXSTATUS_MASK_REG      (* (reg8 *) DATA_COM_BUART_sRX_RxSts__MASK_REG )
    #define DATA_COM_TXSTATUS_MASK_PTR      (  (reg8 *) DATA_COM_BUART_sRX_RxSts__MASK_REG )
    #define DATA_COM_TXSTATUS_ACTL_REG      (* (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define DATA_COM_TXSTATUS_ACTL_PTR      (  (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End DATA_COM_HD_ENABLED */

#if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
    #define DATA_COM_RXDATA_REG             (* (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__F0_REG )
    #define DATA_COM_RXDATA_PTR             (  (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__F0_REG )
    #define DATA_COM_RXADDRESS1_REG         (* (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__D0_REG )
    #define DATA_COM_RXADDRESS1_PTR         (  (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__D0_REG )
    #define DATA_COM_RXADDRESS2_REG         (* (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__D1_REG )
    #define DATA_COM_RXADDRESS2_PTR         (  (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__D1_REG )
    #define DATA_COM_RXDATA_AUX_CTL_REG     (* (reg8 *) DATA_COM_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define DATA_COM_RXBITCTR_PERIOD_REG    (* (reg8 *) DATA_COM_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define DATA_COM_RXBITCTR_PERIOD_PTR    (  (reg8 *) DATA_COM_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define DATA_COM_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) DATA_COM_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define DATA_COM_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) DATA_COM_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define DATA_COM_RXBITCTR_COUNTER_REG   (* (reg8 *) DATA_COM_BUART_sRX_RxBitCounter__COUNT_REG )
    #define DATA_COM_RXBITCTR_COUNTER_PTR   (  (reg8 *) DATA_COM_BUART_sRX_RxBitCounter__COUNT_REG )

    #define DATA_COM_RXSTATUS_REG           (* (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_REG )
    #define DATA_COM_RXSTATUS_PTR           (  (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_REG )
    #define DATA_COM_RXSTATUS_MASK_REG      (* (reg8 *) DATA_COM_BUART_sRX_RxSts__MASK_REG )
    #define DATA_COM_RXSTATUS_MASK_PTR      (  (reg8 *) DATA_COM_BUART_sRX_RxSts__MASK_REG )
    #define DATA_COM_RXSTATUS_ACTL_REG      (* (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define DATA_COM_RXSTATUS_ACTL_PTR      (  (reg8 *) DATA_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) */

#if(DATA_COM_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define DATA_COM_INTCLOCK_CLKEN_REG     (* (reg8 *) DATA_COM_IntClock__PM_ACT_CFG)
    #define DATA_COM_INTCLOCK_CLKEN_PTR     (  (reg8 *) DATA_COM_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define DATA_COM_INTCLOCK_CLKEN_MASK    DATA_COM_IntClock__PM_ACT_MSK
#endif /* End DATA_COM_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(DATA_COM_TX_ENABLED)
    #define DATA_COM_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End DATA_COM_TX_ENABLED */

#if(DATA_COM_HD_ENABLED)
    #define DATA_COM_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End DATA_COM_HD_ENABLED */

#if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
    #define DATA_COM_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define DATA_COM_initvar                    DATA_COM_initVar

#define DATA_COM_RX_Enabled                 DATA_COM_RX_ENABLED
#define DATA_COM_TX_Enabled                 DATA_COM_TX_ENABLED
#define DATA_COM_HD_Enabled                 DATA_COM_HD_ENABLED
#define DATA_COM_RX_IntInterruptEnabled     DATA_COM_RX_INTERRUPT_ENABLED
#define DATA_COM_TX_IntInterruptEnabled     DATA_COM_TX_INTERRUPT_ENABLED
#define DATA_COM_InternalClockUsed          DATA_COM_INTERNAL_CLOCK_USED
#define DATA_COM_RXHW_Address_Enabled       DATA_COM_RXHW_ADDRESS_ENABLED
#define DATA_COM_OverSampleCount            DATA_COM_OVER_SAMPLE_COUNT
#define DATA_COM_ParityType                 DATA_COM_PARITY_TYPE

#if( DATA_COM_TX_ENABLED && (DATA_COM_TXBUFFERSIZE > DATA_COM_FIFO_LENGTH))
    #define DATA_COM_TXBUFFER               DATA_COM_txBuffer
    #define DATA_COM_TXBUFFERREAD           DATA_COM_txBufferRead
    #define DATA_COM_TXBUFFERWRITE          DATA_COM_txBufferWrite
#endif /* End DATA_COM_TX_ENABLED */
#if( ( DATA_COM_RX_ENABLED || DATA_COM_HD_ENABLED ) && \
     (DATA_COM_RXBUFFERSIZE > DATA_COM_FIFO_LENGTH) )
    #define DATA_COM_RXBUFFER               DATA_COM_rxBuffer
    #define DATA_COM_RXBUFFERREAD           DATA_COM_rxBufferRead
    #define DATA_COM_RXBUFFERWRITE          DATA_COM_rxBufferWrite
    #define DATA_COM_RXBUFFERLOOPDETECT     DATA_COM_rxBufferLoopDetect
    #define DATA_COM_RXBUFFER_OVERFLOW      DATA_COM_rxBufferOverflow
#endif /* End DATA_COM_RX_ENABLED */

#ifdef DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define DATA_COM_CONTROL                DATA_COM_CONTROL_REG
#endif /* End DATA_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(DATA_COM_TX_ENABLED)
    #define DATA_COM_TXDATA                 DATA_COM_TXDATA_REG
    #define DATA_COM_TXSTATUS               DATA_COM_TXSTATUS_REG
    #define DATA_COM_TXSTATUS_MASK          DATA_COM_TXSTATUS_MASK_REG
    #define DATA_COM_TXSTATUS_ACTL          DATA_COM_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(DATA_COM_TXCLKGEN_DP)
        #define DATA_COM_TXBITCLKGEN_CTR        DATA_COM_TXBITCLKGEN_CTR_REG
        #define DATA_COM_TXBITCLKTX_COMPLETE    DATA_COM_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define DATA_COM_TXBITCTR_PERIOD        DATA_COM_TXBITCTR_PERIOD_REG
        #define DATA_COM_TXBITCTR_CONTROL       DATA_COM_TXBITCTR_CONTROL_REG
        #define DATA_COM_TXBITCTR_COUNTER       DATA_COM_TXBITCTR_COUNTER_REG
    #endif /* DATA_COM_TXCLKGEN_DP */
#endif /* End DATA_COM_TX_ENABLED */

#if(DATA_COM_HD_ENABLED)
    #define DATA_COM_TXDATA                 DATA_COM_TXDATA_REG
    #define DATA_COM_TXSTATUS               DATA_COM_TXSTATUS_REG
    #define DATA_COM_TXSTATUS_MASK          DATA_COM_TXSTATUS_MASK_REG
    #define DATA_COM_TXSTATUS_ACTL          DATA_COM_TXSTATUS_ACTL_REG
#endif /* End DATA_COM_HD_ENABLED */

#if( (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) )
    #define DATA_COM_RXDATA                 DATA_COM_RXDATA_REG
    #define DATA_COM_RXADDRESS1             DATA_COM_RXADDRESS1_REG
    #define DATA_COM_RXADDRESS2             DATA_COM_RXADDRESS2_REG
    #define DATA_COM_RXBITCTR_PERIOD        DATA_COM_RXBITCTR_PERIOD_REG
    #define DATA_COM_RXBITCTR_CONTROL       DATA_COM_RXBITCTR_CONTROL_REG
    #define DATA_COM_RXBITCTR_COUNTER       DATA_COM_RXBITCTR_COUNTER_REG
    #define DATA_COM_RXSTATUS               DATA_COM_RXSTATUS_REG
    #define DATA_COM_RXSTATUS_MASK          DATA_COM_RXSTATUS_MASK_REG
    #define DATA_COM_RXSTATUS_ACTL          DATA_COM_RXSTATUS_ACTL_REG
#endif /* End  (DATA_COM_RX_ENABLED) || (DATA_COM_HD_ENABLED) */

#if(DATA_COM_INTERNAL_CLOCK_USED)
    #define DATA_COM_INTCLOCK_CLKEN         DATA_COM_INTCLOCK_CLKEN_REG
#endif /* End DATA_COM_INTERNAL_CLOCK_USED */

#define DATA_COM_WAIT_FOR_COMLETE_REINIT    DATA_COM_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_DATA_COM_H */


/* [] END OF FILE */
