/*******************************************************************************
* File Name: CONFIG_COM.h
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


#if !defined(CY_UART_CONFIG_COM_H)
#define CY_UART_CONFIG_COM_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"


/***************************************
* Conditional Compilation Parameters
***************************************/

#define CONFIG_COM_RX_ENABLED                     (1u)
#define CONFIG_COM_TX_ENABLED                     (1u)
#define CONFIG_COM_HD_ENABLED                     (0u)
#define CONFIG_COM_RX_INTERRUPT_ENABLED           (0u)
#define CONFIG_COM_TX_INTERRUPT_ENABLED           (0u)
#define CONFIG_COM_INTERNAL_CLOCK_USED            (1u)
#define CONFIG_COM_RXHW_ADDRESS_ENABLED           (0u)
#define CONFIG_COM_OVER_SAMPLE_COUNT              (8u)
#define CONFIG_COM_PARITY_TYPE                    (0u)
#define CONFIG_COM_PARITY_TYPE_SW                 (0u)
#define CONFIG_COM_BREAK_DETECT                   (0u)
#define CONFIG_COM_BREAK_BITS_TX                  (13u)
#define CONFIG_COM_BREAK_BITS_RX                  (13u)
#define CONFIG_COM_TXCLKGEN_DP                    (1u)
#define CONFIG_COM_USE23POLLING                   (1u)
#define CONFIG_COM_FLOW_CONTROL                   (0u)
#define CONFIG_COM_CLK_FREQ                       (0u)
#define CONFIG_COM_TXBUFFERSIZE                   (4u)
#define CONFIG_COM_RXBUFFERSIZE                   (4u)

/* Check to see if required defines such as CY_PSOC5LP are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5LP)
    #error Component UART_v2_30 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5LP) */

#ifdef CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define CONFIG_COM_CONTROL_REG_REMOVED            (0u)
#else
    #define CONFIG_COM_CONTROL_REG_REMOVED            (1u)
#endif /* End CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */


/***************************************
*      Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct CONFIG_COM_backupStruct_
{
    uint8 enableState;

    #if(CONFIG_COM_CONTROL_REG_REMOVED == 0u)
        uint8 cr;
    #endif /* End CONFIG_COM_CONTROL_REG_REMOVED */
    #if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
        uint8 rx_period;
        #if (CY_UDB_V0)
            uint8 rx_mask;
            #if (CONFIG_COM_RXHW_ADDRESS_ENABLED)
                uint8 rx_addr1;
                uint8 rx_addr2;
            #endif /* End CONFIG_COM_RXHW_ADDRESS_ENABLED */
        #endif /* End CY_UDB_V0 */
    #endif  /* End (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED)*/

    #if(CONFIG_COM_TX_ENABLED)
        #if(CONFIG_COM_TXCLKGEN_DP)
            uint8 tx_clk_ctr;
            #if (CY_UDB_V0)
                uint8 tx_clk_compl;
            #endif  /* End CY_UDB_V0 */
        #else
            uint8 tx_period;
        #endif /*End CONFIG_COM_TXCLKGEN_DP */
        #if (CY_UDB_V0)
            uint8 tx_mask;
        #endif  /* End CY_UDB_V0 */
    #endif /*End CONFIG_COM_TX_ENABLED */
} CONFIG_COM_BACKUP_STRUCT;


/***************************************
*       Function Prototypes
***************************************/

void CONFIG_COM_Start(void) ;
void CONFIG_COM_Stop(void) ;
uint8 CONFIG_COM_ReadControlRegister(void) ;
void CONFIG_COM_WriteControlRegister(uint8 control) ;

void CONFIG_COM_Init(void) ;
void CONFIG_COM_Enable(void) ;
void CONFIG_COM_SaveConfig(void) ;
void CONFIG_COM_RestoreConfig(void) ;
void CONFIG_COM_Sleep(void) ;
void CONFIG_COM_Wakeup(void) ;

/* Only if RX is enabled */
#if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )

    #if(CONFIG_COM_RX_INTERRUPT_ENABLED)
        void  CONFIG_COM_EnableRxInt(void) ;
        void  CONFIG_COM_DisableRxInt(void) ;
        CY_ISR_PROTO(CONFIG_COM_RXISR);
    #endif /* CONFIG_COM_RX_INTERRUPT_ENABLED */

    void CONFIG_COM_SetRxAddressMode(uint8 addressMode)
                                                           ;
    void CONFIG_COM_SetRxAddress1(uint8 address) ;
    void CONFIG_COM_SetRxAddress2(uint8 address) ;

    void  CONFIG_COM_SetRxInterruptMode(uint8 intSrc) ;
    uint8 CONFIG_COM_ReadRxData(void) ;
    uint8 CONFIG_COM_ReadRxStatus(void) ;
    uint8 CONFIG_COM_GetChar(void) ;
    uint16 CONFIG_COM_GetByte(void) ;
    uint8 CONFIG_COM_GetRxBufferSize(void)
                                                            ;
    void CONFIG_COM_ClearRxBuffer(void) ;

    /* Obsolete functions, defines for backward compatible */
    #define CONFIG_COM_GetRxInterruptSource   CONFIG_COM_ReadRxStatus

#endif /* End (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) */

/* Only if TX is enabled */
#if(CONFIG_COM_TX_ENABLED || CONFIG_COM_HD_ENABLED)

    #if(CONFIG_COM_TX_INTERRUPT_ENABLED)
        void CONFIG_COM_EnableTxInt(void) ;
        void CONFIG_COM_DisableTxInt(void) ;
        CY_ISR_PROTO(CONFIG_COM_TXISR);
    #endif /* CONFIG_COM_TX_INTERRUPT_ENABLED */

    void CONFIG_COM_SetTxInterruptMode(uint8 intSrc) ;
    void CONFIG_COM_WriteTxData(uint8 txDataByte) ;
    uint8 CONFIG_COM_ReadTxStatus(void) ;
    void CONFIG_COM_PutChar(uint8 txDataByte) ;
    void CONFIG_COM_PutString(const char8 string[]) ;
    void CONFIG_COM_PutArray(const uint8 string[], uint8 byteCount)
                                                            ;
    void CONFIG_COM_PutCRLF(uint8 txDataByte) ;
    void CONFIG_COM_ClearTxBuffer(void) ;
    void CONFIG_COM_SetTxAddressMode(uint8 addressMode) ;
    void CONFIG_COM_SendBreak(uint8 retMode) ;
    uint8 CONFIG_COM_GetTxBufferSize(void)
                                                            ;
    /* Obsolete functions, defines for backward compatible */
    #define CONFIG_COM_PutStringConst         CONFIG_COM_PutString
    #define CONFIG_COM_PutArrayConst          CONFIG_COM_PutArray
    #define CONFIG_COM_GetTxInterruptSource   CONFIG_COM_ReadTxStatus

#endif /* End CONFIG_COM_TX_ENABLED || CONFIG_COM_HD_ENABLED */

#if(CONFIG_COM_HD_ENABLED)
    void CONFIG_COM_LoadRxConfig(void) ;
    void CONFIG_COM_LoadTxConfig(void) ;
#endif /* End CONFIG_COM_HD_ENABLED */


/* Communication bootloader APIs */
#if defined(CYDEV_BOOTLOADER_IO_COMP) && ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_CONFIG_COM) || \
                                          (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface))
    /* Physical layer functions */
    void    CONFIG_COM_CyBtldrCommStart(void) CYSMALL ;
    void    CONFIG_COM_CyBtldrCommStop(void) CYSMALL ;
    void    CONFIG_COM_CyBtldrCommReset(void) CYSMALL ;
    cystatus CONFIG_COM_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;
    cystatus CONFIG_COM_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) CYSMALL
             ;

    #if (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_CONFIG_COM)
        #define CyBtldrCommStart    CONFIG_COM_CyBtldrCommStart
        #define CyBtldrCommStop     CONFIG_COM_CyBtldrCommStop
        #define CyBtldrCommReset    CONFIG_COM_CyBtldrCommReset
        #define CyBtldrCommWrite    CONFIG_COM_CyBtldrCommWrite
        #define CyBtldrCommRead     CONFIG_COM_CyBtldrCommRead
    #endif  /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_CONFIG_COM) */

    /* Byte to Byte time out for detecting end of block data from host */
    #define CONFIG_COM_BYTE2BYTE_TIME_OUT (25u)

#endif /* CYDEV_BOOTLOADER_IO_COMP */


/***************************************
*          API Constants
***************************************/
/* Parameters for SetTxAddressMode API*/
#define CONFIG_COM_SET_SPACE                              (0x00u)
#define CONFIG_COM_SET_MARK                               (0x01u)

/* Status Register definitions */
#if( (CONFIG_COM_TX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
    #if(CONFIG_COM_TX_INTERRUPT_ENABLED)
        #define CONFIG_COM_TX_VECT_NUM            (uint8)CONFIG_COM_TXInternalInterrupt__INTC_NUMBER
        #define CONFIG_COM_TX_PRIOR_NUM           (uint8)CONFIG_COM_TXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* CONFIG_COM_TX_INTERRUPT_ENABLED */
    #if(CONFIG_COM_TX_ENABLED)
        #define CONFIG_COM_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define CONFIG_COM_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define CONFIG_COM_TX_STS_FIFO_FULL_SHIFT         (0x02u)
        #define CONFIG_COM_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* CONFIG_COM_TX_ENABLED */
    #if(CONFIG_COM_HD_ENABLED)
        #define CONFIG_COM_TX_STS_COMPLETE_SHIFT          (0x00u)
        #define CONFIG_COM_TX_STS_FIFO_EMPTY_SHIFT        (0x01u)
        #define CONFIG_COM_TX_STS_FIFO_FULL_SHIFT         (0x05u)  /*needs MD=0*/
        #define CONFIG_COM_TX_STS_FIFO_NOT_FULL_SHIFT     (0x03u)
    #endif /* CONFIG_COM_HD_ENABLED */
    #define CONFIG_COM_TX_STS_COMPLETE            (uint8)(0x01u << CONFIG_COM_TX_STS_COMPLETE_SHIFT)
    #define CONFIG_COM_TX_STS_FIFO_EMPTY          (uint8)(0x01u << CONFIG_COM_TX_STS_FIFO_EMPTY_SHIFT)
    #define CONFIG_COM_TX_STS_FIFO_FULL           (uint8)(0x01u << CONFIG_COM_TX_STS_FIFO_FULL_SHIFT)
    #define CONFIG_COM_TX_STS_FIFO_NOT_FULL       (uint8)(0x01u << CONFIG_COM_TX_STS_FIFO_NOT_FULL_SHIFT)
#endif /* End (CONFIG_COM_TX_ENABLED) || (CONFIG_COM_HD_ENABLED)*/

#if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
    #if(CONFIG_COM_RX_INTERRUPT_ENABLED)
        #define CONFIG_COM_RX_VECT_NUM            (uint8)CONFIG_COM_RXInternalInterrupt__INTC_NUMBER
        #define CONFIG_COM_RX_PRIOR_NUM           (uint8)CONFIG_COM_RXInternalInterrupt__INTC_PRIOR_NUM
    #endif /* CONFIG_COM_RX_INTERRUPT_ENABLED */
    #define CONFIG_COM_RX_STS_MRKSPC_SHIFT            (0x00u)
    #define CONFIG_COM_RX_STS_BREAK_SHIFT             (0x01u)
    #define CONFIG_COM_RX_STS_PAR_ERROR_SHIFT         (0x02u)
    #define CONFIG_COM_RX_STS_STOP_ERROR_SHIFT        (0x03u)
    #define CONFIG_COM_RX_STS_OVERRUN_SHIFT           (0x04u)
    #define CONFIG_COM_RX_STS_FIFO_NOTEMPTY_SHIFT     (0x05u)
    #define CONFIG_COM_RX_STS_ADDR_MATCH_SHIFT        (0x06u)
    #define CONFIG_COM_RX_STS_SOFT_BUFF_OVER_SHIFT    (0x07u)

    #define CONFIG_COM_RX_STS_MRKSPC           (uint8)(0x01u << CONFIG_COM_RX_STS_MRKSPC_SHIFT)
    #define CONFIG_COM_RX_STS_BREAK            (uint8)(0x01u << CONFIG_COM_RX_STS_BREAK_SHIFT)
    #define CONFIG_COM_RX_STS_PAR_ERROR        (uint8)(0x01u << CONFIG_COM_RX_STS_PAR_ERROR_SHIFT)
    #define CONFIG_COM_RX_STS_STOP_ERROR       (uint8)(0x01u << CONFIG_COM_RX_STS_STOP_ERROR_SHIFT)
    #define CONFIG_COM_RX_STS_OVERRUN          (uint8)(0x01u << CONFIG_COM_RX_STS_OVERRUN_SHIFT)
    #define CONFIG_COM_RX_STS_FIFO_NOTEMPTY    (uint8)(0x01u << CONFIG_COM_RX_STS_FIFO_NOTEMPTY_SHIFT)
    #define CONFIG_COM_RX_STS_ADDR_MATCH       (uint8)(0x01u << CONFIG_COM_RX_STS_ADDR_MATCH_SHIFT)
    #define CONFIG_COM_RX_STS_SOFT_BUFF_OVER   (uint8)(0x01u << CONFIG_COM_RX_STS_SOFT_BUFF_OVER_SHIFT)
    #define CONFIG_COM_RX_HW_MASK                     (0x7Fu)
#endif /* End (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) */

/* Control Register definitions */
#define CONFIG_COM_CTRL_HD_SEND_SHIFT                 (0x00u) /* 1 enable TX part in Half Duplex mode */
#define CONFIG_COM_CTRL_HD_SEND_BREAK_SHIFT           (0x01u) /* 1 send BREAK signal in Half Duplez mode */
#define CONFIG_COM_CTRL_MARK_SHIFT                    (0x02u) /* 1 sets mark, 0 sets space */
#define CONFIG_COM_CTRL_PARITY_TYPE0_SHIFT            (0x03u) /* Defines the type of parity implemented */
#define CONFIG_COM_CTRL_PARITY_TYPE1_SHIFT            (0x04u) /* Defines the type of parity implemented */
#define CONFIG_COM_CTRL_RXADDR_MODE0_SHIFT            (0x05u)
#define CONFIG_COM_CTRL_RXADDR_MODE1_SHIFT            (0x06u)
#define CONFIG_COM_CTRL_RXADDR_MODE2_SHIFT            (0x07u)

#define CONFIG_COM_CTRL_HD_SEND               (uint8)(0x01u << CONFIG_COM_CTRL_HD_SEND_SHIFT)
#define CONFIG_COM_CTRL_HD_SEND_BREAK         (uint8)(0x01u << CONFIG_COM_CTRL_HD_SEND_BREAK_SHIFT)
#define CONFIG_COM_CTRL_MARK                  (uint8)(0x01u << CONFIG_COM_CTRL_MARK_SHIFT)
#define CONFIG_COM_CTRL_PARITY_TYPE_MASK      (uint8)(0x03u << CONFIG_COM_CTRL_PARITY_TYPE0_SHIFT)
#define CONFIG_COM_CTRL_RXADDR_MODE_MASK      (uint8)(0x07u << CONFIG_COM_CTRL_RXADDR_MODE0_SHIFT)

/* StatusI Register Interrupt Enable Control Bits. As defined by the Register map for the AUX Control Register */
#define CONFIG_COM_INT_ENABLE                         (0x10u)

/* Bit Counter (7-bit) Control Register Bit Definitions. As defined by the Register map for the AUX Control Register */
#define CONFIG_COM_CNTR_ENABLE                        (0x20u)

/*   Constants for SendBreak() "retMode" parameter  */
#define CONFIG_COM_SEND_BREAK                         (0x00u)
#define CONFIG_COM_WAIT_FOR_COMPLETE_REINIT           (0x01u)
#define CONFIG_COM_REINIT                             (0x02u)
#define CONFIG_COM_SEND_WAIT_REINIT                   (0x03u)

#define CONFIG_COM_OVER_SAMPLE_8                      (8u)
#define CONFIG_COM_OVER_SAMPLE_16                     (16u)

#define CONFIG_COM_BIT_CENTER                         (CONFIG_COM_OVER_SAMPLE_COUNT - 1u)

#define CONFIG_COM_FIFO_LENGTH                        (4u)
#define CONFIG_COM_NUMBER_OF_START_BIT                (1u)
#define CONFIG_COM_MAX_BYTE_VALUE                     (0xFFu)

/* 8X always for count7 implementation*/
#define CONFIG_COM_TXBITCTR_BREAKBITS8X   ((CONFIG_COM_BREAK_BITS_TX * CONFIG_COM_OVER_SAMPLE_8) - 1u)
/* 8X or 16X for DP implementation*/
#define CONFIG_COM_TXBITCTR_BREAKBITS ((CONFIG_COM_BREAK_BITS_TX * CONFIG_COM_OVER_SAMPLE_COUNT) - 1u)

#define CONFIG_COM_HALF_BIT_COUNT   \
                            (((CONFIG_COM_OVER_SAMPLE_COUNT / 2u) + (CONFIG_COM_USE23POLLING * 1u)) - 2u)
#if (CONFIG_COM_OVER_SAMPLE_COUNT == CONFIG_COM_OVER_SAMPLE_8)
    #define CONFIG_COM_HD_TXBITCTR_INIT   (((CONFIG_COM_BREAK_BITS_TX + \
                            CONFIG_COM_NUMBER_OF_START_BIT) * CONFIG_COM_OVER_SAMPLE_COUNT) - 1u)

    /* This parameter is increased on the 2 in 2 out of 3 mode to sample voting in the middle */
    #define CONFIG_COM_RXBITCTR_INIT  ((((CONFIG_COM_BREAK_BITS_RX + CONFIG_COM_NUMBER_OF_START_BIT) \
                            * CONFIG_COM_OVER_SAMPLE_COUNT) + CONFIG_COM_HALF_BIT_COUNT) - 1u)


#else /* CONFIG_COM_OVER_SAMPLE_COUNT == CONFIG_COM_OVER_SAMPLE_16 */
    #define CONFIG_COM_HD_TXBITCTR_INIT   ((8u * CONFIG_COM_OVER_SAMPLE_COUNT) - 1u)
    /* 7bit counter need one more bit for OverSampleCount=16 */
    #define CONFIG_COM_RXBITCTR_INIT      (((7u * CONFIG_COM_OVER_SAMPLE_COUNT) - 1u) + \
                                                      CONFIG_COM_HALF_BIT_COUNT)
#endif /* End CONFIG_COM_OVER_SAMPLE_COUNT */
#define CONFIG_COM_HD_RXBITCTR_INIT                   CONFIG_COM_RXBITCTR_INIT


/***************************************
* Global variables external identifier
***************************************/

extern uint8 CONFIG_COM_initVar;
#if( CONFIG_COM_TX_ENABLED && (CONFIG_COM_TXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH))
    extern volatile uint8 CONFIG_COM_txBuffer[CONFIG_COM_TXBUFFERSIZE];
    extern volatile uint8 CONFIG_COM_txBufferRead;
    extern uint8 CONFIG_COM_txBufferWrite;
#endif /* End CONFIG_COM_TX_ENABLED */
#if( ( CONFIG_COM_RX_ENABLED || CONFIG_COM_HD_ENABLED ) && \
     (CONFIG_COM_RXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH) )
    extern volatile uint8 CONFIG_COM_rxBuffer[CONFIG_COM_RXBUFFERSIZE];
    extern volatile uint8 CONFIG_COM_rxBufferRead;
    extern volatile uint8 CONFIG_COM_rxBufferWrite;
    extern volatile uint8 CONFIG_COM_rxBufferLoopDetect;
    extern volatile uint8 CONFIG_COM_rxBufferOverflow;
    #if (CONFIG_COM_RXHW_ADDRESS_ENABLED)
        extern volatile uint8 CONFIG_COM_rxAddressMode;
        extern volatile uint8 CONFIG_COM_rxAddressDetected;
    #endif /* End EnableHWAddress */
#endif /* End CONFIG_COM_RX_ENABLED */


/***************************************
* Enumerated Types and Parameters
***************************************/

#define CONFIG_COM__B_UART__AM_SW_BYTE_BYTE 1
#define CONFIG_COM__B_UART__AM_SW_DETECT_TO_BUFFER 2
#define CONFIG_COM__B_UART__AM_HW_BYTE_BY_BYTE 3
#define CONFIG_COM__B_UART__AM_HW_DETECT_TO_BUFFER 4
#define CONFIG_COM__B_UART__AM_NONE 0

#define CONFIG_COM__B_UART__NONE_REVB 0
#define CONFIG_COM__B_UART__EVEN_REVB 1
#define CONFIG_COM__B_UART__ODD_REVB 2
#define CONFIG_COM__B_UART__MARK_SPACE_REVB 3



/***************************************
*    Initial Parameter Constants
***************************************/

/* UART shifts max 8 bits, Mark/Space functionality working if 9 selected */
#define CONFIG_COM_NUMBER_OF_DATA_BITS    ((8u > 8u) ? 8u : 8u)
#define CONFIG_COM_NUMBER_OF_STOP_BITS    (1u)

#if (CONFIG_COM_RXHW_ADDRESS_ENABLED)
    #define CONFIG_COM_RXADDRESSMODE      (0u)
    #define CONFIG_COM_RXHWADDRESS1       (0u)
    #define CONFIG_COM_RXHWADDRESS2       (0u)
    /* Backward compatible define */
    #define CONFIG_COM_RXAddressMode      CONFIG_COM_RXADDRESSMODE
#endif /* End EnableHWAddress */

#define CONFIG_COM_INIT_RX_INTERRUPTS_MASK \
                                  (uint8)((1 << CONFIG_COM_RX_STS_FIFO_NOTEMPTY_SHIFT) \
                                        | (0 << CONFIG_COM_RX_STS_MRKSPC_SHIFT) \
                                        | (0 << CONFIG_COM_RX_STS_ADDR_MATCH_SHIFT) \
                                        | (0 << CONFIG_COM_RX_STS_PAR_ERROR_SHIFT) \
                                        | (0 << CONFIG_COM_RX_STS_STOP_ERROR_SHIFT) \
                                        | (0 << CONFIG_COM_RX_STS_BREAK_SHIFT) \
                                        | (0 << CONFIG_COM_RX_STS_OVERRUN_SHIFT))

#define CONFIG_COM_INIT_TX_INTERRUPTS_MASK \
                                  (uint8)((0 << CONFIG_COM_TX_STS_COMPLETE_SHIFT) \
                                        | (0 << CONFIG_COM_TX_STS_FIFO_EMPTY_SHIFT) \
                                        | (0 << CONFIG_COM_TX_STS_FIFO_FULL_SHIFT) \
                                        | (0 << CONFIG_COM_TX_STS_FIFO_NOT_FULL_SHIFT))


/***************************************
*              Registers
***************************************/

#ifdef CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define CONFIG_COM_CONTROL_REG \
                            (* (reg8 *) CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
    #define CONFIG_COM_CONTROL_PTR \
                            (  (reg8 *) CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG )
#endif /* End CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(CONFIG_COM_TX_ENABLED)
    #define CONFIG_COM_TXDATA_REG          (* (reg8 *) CONFIG_COM_BUART_sTX_TxShifter_u0__F0_REG)
    #define CONFIG_COM_TXDATA_PTR          (  (reg8 *) CONFIG_COM_BUART_sTX_TxShifter_u0__F0_REG)
    #define CONFIG_COM_TXDATA_AUX_CTL_REG  (* (reg8 *) CONFIG_COM_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define CONFIG_COM_TXDATA_AUX_CTL_PTR  (  (reg8 *) CONFIG_COM_BUART_sTX_TxShifter_u0__DP_AUX_CTL_REG)
    #define CONFIG_COM_TXSTATUS_REG        (* (reg8 *) CONFIG_COM_BUART_sTX_TxSts__STATUS_REG)
    #define CONFIG_COM_TXSTATUS_PTR        (  (reg8 *) CONFIG_COM_BUART_sTX_TxSts__STATUS_REG)
    #define CONFIG_COM_TXSTATUS_MASK_REG   (* (reg8 *) CONFIG_COM_BUART_sTX_TxSts__MASK_REG)
    #define CONFIG_COM_TXSTATUS_MASK_PTR   (  (reg8 *) CONFIG_COM_BUART_sTX_TxSts__MASK_REG)
    #define CONFIG_COM_TXSTATUS_ACTL_REG   (* (reg8 *) CONFIG_COM_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)
    #define CONFIG_COM_TXSTATUS_ACTL_PTR   (  (reg8 *) CONFIG_COM_BUART_sTX_TxSts__STATUS_AUX_CTL_REG)

    /* DP clock */
    #if(CONFIG_COM_TXCLKGEN_DP)
        #define CONFIG_COM_TXBITCLKGEN_CTR_REG        \
                                        (* (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define CONFIG_COM_TXBITCLKGEN_CTR_PTR        \
                                        (  (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitClkGen__D0_REG)
        #define CONFIG_COM_TXBITCLKTX_COMPLETE_REG    \
                                        (* (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
        #define CONFIG_COM_TXBITCLKTX_COMPLETE_PTR    \
                                        (  (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitClkGen__D1_REG)
    #else     /* Count7 clock*/
        #define CONFIG_COM_TXBITCTR_PERIOD_REG    \
                                        (* (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define CONFIG_COM_TXBITCTR_PERIOD_PTR    \
                                        (  (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitCounter__PERIOD_REG)
        #define CONFIG_COM_TXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define CONFIG_COM_TXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitCounter__CONTROL_AUX_CTL_REG)
        #define CONFIG_COM_TXBITCTR_COUNTER_REG   \
                                        (* (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
        #define CONFIG_COM_TXBITCTR_COUNTER_PTR   \
                                        (  (reg8 *) CONFIG_COM_BUART_sTX_sCLOCK_TxBitCounter__COUNT_REG)
    #endif /* CONFIG_COM_TXCLKGEN_DP */

#endif /* End CONFIG_COM_TX_ENABLED */

#if(CONFIG_COM_HD_ENABLED)

    #define CONFIG_COM_TXDATA_REG             (* (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__F1_REG )
    #define CONFIG_COM_TXDATA_PTR             (  (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__F1_REG )
    #define CONFIG_COM_TXDATA_AUX_CTL_REG     (* (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)
    #define CONFIG_COM_TXDATA_AUX_CTL_PTR     (  (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define CONFIG_COM_TXSTATUS_REG           (* (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_REG )
    #define CONFIG_COM_TXSTATUS_PTR           (  (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_REG )
    #define CONFIG_COM_TXSTATUS_MASK_REG      (* (reg8 *) CONFIG_COM_BUART_sRX_RxSts__MASK_REG )
    #define CONFIG_COM_TXSTATUS_MASK_PTR      (  (reg8 *) CONFIG_COM_BUART_sRX_RxSts__MASK_REG )
    #define CONFIG_COM_TXSTATUS_ACTL_REG      (* (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define CONFIG_COM_TXSTATUS_ACTL_PTR      (  (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End CONFIG_COM_HD_ENABLED */

#if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
    #define CONFIG_COM_RXDATA_REG             (* (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__F0_REG )
    #define CONFIG_COM_RXDATA_PTR             (  (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__F0_REG )
    #define CONFIG_COM_RXADDRESS1_REG         (* (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__D0_REG )
    #define CONFIG_COM_RXADDRESS1_PTR         (  (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__D0_REG )
    #define CONFIG_COM_RXADDRESS2_REG         (* (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__D1_REG )
    #define CONFIG_COM_RXADDRESS2_PTR         (  (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__D1_REG )
    #define CONFIG_COM_RXDATA_AUX_CTL_REG     (* (reg8 *) CONFIG_COM_BUART_sRX_RxShifter_u0__DP_AUX_CTL_REG)

    #define CONFIG_COM_RXBITCTR_PERIOD_REG    (* (reg8 *) CONFIG_COM_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define CONFIG_COM_RXBITCTR_PERIOD_PTR    (  (reg8 *) CONFIG_COM_BUART_sRX_RxBitCounter__PERIOD_REG )
    #define CONFIG_COM_RXBITCTR_CONTROL_REG   \
                                        (* (reg8 *) CONFIG_COM_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define CONFIG_COM_RXBITCTR_CONTROL_PTR   \
                                        (  (reg8 *) CONFIG_COM_BUART_sRX_RxBitCounter__CONTROL_AUX_CTL_REG )
    #define CONFIG_COM_RXBITCTR_COUNTER_REG   (* (reg8 *) CONFIG_COM_BUART_sRX_RxBitCounter__COUNT_REG )
    #define CONFIG_COM_RXBITCTR_COUNTER_PTR   (  (reg8 *) CONFIG_COM_BUART_sRX_RxBitCounter__COUNT_REG )

    #define CONFIG_COM_RXSTATUS_REG           (* (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_REG )
    #define CONFIG_COM_RXSTATUS_PTR           (  (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_REG )
    #define CONFIG_COM_RXSTATUS_MASK_REG      (* (reg8 *) CONFIG_COM_BUART_sRX_RxSts__MASK_REG )
    #define CONFIG_COM_RXSTATUS_MASK_PTR      (  (reg8 *) CONFIG_COM_BUART_sRX_RxSts__MASK_REG )
    #define CONFIG_COM_RXSTATUS_ACTL_REG      (* (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
    #define CONFIG_COM_RXSTATUS_ACTL_PTR      (  (reg8 *) CONFIG_COM_BUART_sRX_RxSts__STATUS_AUX_CTL_REG )
#endif /* End  (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) */

#if(CONFIG_COM_INTERNAL_CLOCK_USED)
    /* Register to enable or disable the digital clocks */
    #define CONFIG_COM_INTCLOCK_CLKEN_REG     (* (reg8 *) CONFIG_COM_IntClock__PM_ACT_CFG)
    #define CONFIG_COM_INTCLOCK_CLKEN_PTR     (  (reg8 *) CONFIG_COM_IntClock__PM_ACT_CFG)

    /* Clock mask for this clock. */
    #define CONFIG_COM_INTCLOCK_CLKEN_MASK    CONFIG_COM_IntClock__PM_ACT_MSK
#endif /* End CONFIG_COM_INTERNAL_CLOCK_USED */


/***************************************
*       Register Constants
***************************************/

#if(CONFIG_COM_TX_ENABLED)
    #define CONFIG_COM_TX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End CONFIG_COM_TX_ENABLED */

#if(CONFIG_COM_HD_ENABLED)
    #define CONFIG_COM_TX_FIFO_CLR            (0x02u) /* FIFO1 CLR */
#endif /* End CONFIG_COM_HD_ENABLED */

#if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
    #define CONFIG_COM_RX_FIFO_CLR            (0x01u) /* FIFO0 CLR */
#endif /* End  (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) */


/***************************************
* Renamed global variables or defines
* for backward compatible
***************************************/

#define CONFIG_COM_initvar                    CONFIG_COM_initVar

#define CONFIG_COM_RX_Enabled                 CONFIG_COM_RX_ENABLED
#define CONFIG_COM_TX_Enabled                 CONFIG_COM_TX_ENABLED
#define CONFIG_COM_HD_Enabled                 CONFIG_COM_HD_ENABLED
#define CONFIG_COM_RX_IntInterruptEnabled     CONFIG_COM_RX_INTERRUPT_ENABLED
#define CONFIG_COM_TX_IntInterruptEnabled     CONFIG_COM_TX_INTERRUPT_ENABLED
#define CONFIG_COM_InternalClockUsed          CONFIG_COM_INTERNAL_CLOCK_USED
#define CONFIG_COM_RXHW_Address_Enabled       CONFIG_COM_RXHW_ADDRESS_ENABLED
#define CONFIG_COM_OverSampleCount            CONFIG_COM_OVER_SAMPLE_COUNT
#define CONFIG_COM_ParityType                 CONFIG_COM_PARITY_TYPE

#if( CONFIG_COM_TX_ENABLED && (CONFIG_COM_TXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH))
    #define CONFIG_COM_TXBUFFER               CONFIG_COM_txBuffer
    #define CONFIG_COM_TXBUFFERREAD           CONFIG_COM_txBufferRead
    #define CONFIG_COM_TXBUFFERWRITE          CONFIG_COM_txBufferWrite
#endif /* End CONFIG_COM_TX_ENABLED */
#if( ( CONFIG_COM_RX_ENABLED || CONFIG_COM_HD_ENABLED ) && \
     (CONFIG_COM_RXBUFFERSIZE > CONFIG_COM_FIFO_LENGTH) )
    #define CONFIG_COM_RXBUFFER               CONFIG_COM_rxBuffer
    #define CONFIG_COM_RXBUFFERREAD           CONFIG_COM_rxBufferRead
    #define CONFIG_COM_RXBUFFERWRITE          CONFIG_COM_rxBufferWrite
    #define CONFIG_COM_RXBUFFERLOOPDETECT     CONFIG_COM_rxBufferLoopDetect
    #define CONFIG_COM_RXBUFFER_OVERFLOW      CONFIG_COM_rxBufferOverflow
#endif /* End CONFIG_COM_RX_ENABLED */

#ifdef CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG
    #define CONFIG_COM_CONTROL                CONFIG_COM_CONTROL_REG
#endif /* End CONFIG_COM_BUART_sCR_SyncCtl_CtrlReg__CONTROL_REG */

#if(CONFIG_COM_TX_ENABLED)
    #define CONFIG_COM_TXDATA                 CONFIG_COM_TXDATA_REG
    #define CONFIG_COM_TXSTATUS               CONFIG_COM_TXSTATUS_REG
    #define CONFIG_COM_TXSTATUS_MASK          CONFIG_COM_TXSTATUS_MASK_REG
    #define CONFIG_COM_TXSTATUS_ACTL          CONFIG_COM_TXSTATUS_ACTL_REG
    /* DP clock */
    #if(CONFIG_COM_TXCLKGEN_DP)
        #define CONFIG_COM_TXBITCLKGEN_CTR        CONFIG_COM_TXBITCLKGEN_CTR_REG
        #define CONFIG_COM_TXBITCLKTX_COMPLETE    CONFIG_COM_TXBITCLKTX_COMPLETE_REG
    #else     /* Count7 clock*/
        #define CONFIG_COM_TXBITCTR_PERIOD        CONFIG_COM_TXBITCTR_PERIOD_REG
        #define CONFIG_COM_TXBITCTR_CONTROL       CONFIG_COM_TXBITCTR_CONTROL_REG
        #define CONFIG_COM_TXBITCTR_COUNTER       CONFIG_COM_TXBITCTR_COUNTER_REG
    #endif /* CONFIG_COM_TXCLKGEN_DP */
#endif /* End CONFIG_COM_TX_ENABLED */

#if(CONFIG_COM_HD_ENABLED)
    #define CONFIG_COM_TXDATA                 CONFIG_COM_TXDATA_REG
    #define CONFIG_COM_TXSTATUS               CONFIG_COM_TXSTATUS_REG
    #define CONFIG_COM_TXSTATUS_MASK          CONFIG_COM_TXSTATUS_MASK_REG
    #define CONFIG_COM_TXSTATUS_ACTL          CONFIG_COM_TXSTATUS_ACTL_REG
#endif /* End CONFIG_COM_HD_ENABLED */

#if( (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) )
    #define CONFIG_COM_RXDATA                 CONFIG_COM_RXDATA_REG
    #define CONFIG_COM_RXADDRESS1             CONFIG_COM_RXADDRESS1_REG
    #define CONFIG_COM_RXADDRESS2             CONFIG_COM_RXADDRESS2_REG
    #define CONFIG_COM_RXBITCTR_PERIOD        CONFIG_COM_RXBITCTR_PERIOD_REG
    #define CONFIG_COM_RXBITCTR_CONTROL       CONFIG_COM_RXBITCTR_CONTROL_REG
    #define CONFIG_COM_RXBITCTR_COUNTER       CONFIG_COM_RXBITCTR_COUNTER_REG
    #define CONFIG_COM_RXSTATUS               CONFIG_COM_RXSTATUS_REG
    #define CONFIG_COM_RXSTATUS_MASK          CONFIG_COM_RXSTATUS_MASK_REG
    #define CONFIG_COM_RXSTATUS_ACTL          CONFIG_COM_RXSTATUS_ACTL_REG
#endif /* End  (CONFIG_COM_RX_ENABLED) || (CONFIG_COM_HD_ENABLED) */

#if(CONFIG_COM_INTERNAL_CLOCK_USED)
    #define CONFIG_COM_INTCLOCK_CLKEN         CONFIG_COM_INTCLOCK_CLKEN_REG
#endif /* End CONFIG_COM_INTERNAL_CLOCK_USED */

#define CONFIG_COM_WAIT_FOR_COMLETE_REINIT    CONFIG_COM_WAIT_FOR_COMPLETE_REINIT

#endif  /* CY_UART_CONFIG_COM_H */


/* [] END OF FILE */
