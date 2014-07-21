/*******************************************************************************
* File Name: LIS331HH_SPI.h
* Version 2.40
*
* Description:
*  Contains the function prototypes, constants and register definition
*  of the SPI Master Component.
*
* Note:
*  None
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_SPIM_LIS331HH_SPI_H)
#define CY_SPIM_LIS331HH_SPI_H

#include "cytypes.h"
#include "cyfitter.h"
#include "CyLib.h"

/* Check to see if required defines such as CY_PSOC5A are available */
/* They are defined starting with cy_boot v3.0 */
#if !defined (CY_PSOC5A)
    #error Component SPI_Master_v2_40 requires cy_boot v3.0 or later
#endif /* (CY_PSOC5A) */


/***************************************
*   Conditional Compilation Parameters
***************************************/

#define LIS331HH_SPI_INTERNAL_CLOCK             (1u)

#if(0u != LIS331HH_SPI_INTERNAL_CLOCK)
    #include "LIS331HH_SPI_IntClock.h"
#endif /* (0u != LIS331HH_SPI_INTERNAL_CLOCK) */

#define LIS331HH_SPI_MODE                       (1u)
#define LIS331HH_SPI_DATA_WIDTH                 (8u)
#define LIS331HH_SPI_MODE_USE_ZERO              (1u)
#define LIS331HH_SPI_BIDIRECTIONAL_MODE         (0u)

/* Internal interrupt handling */
#define LIS331HH_SPI_TX_BUFFER_SIZE             (4u)
#define LIS331HH_SPI_RX_BUFFER_SIZE             (4u)
#define LIS331HH_SPI_INTERNAL_TX_INT_ENABLED    (0u)
#define LIS331HH_SPI_INTERNAL_RX_INT_ENABLED    (0u)

#define LIS331HH_SPI_SINGLE_REG_SIZE            (8u)
#define LIS331HH_SPI_USE_SECOND_DATAPATH        (LIS331HH_SPI_DATA_WIDTH > LIS331HH_SPI_SINGLE_REG_SIZE)

#define LIS331HH_SPI_FIFO_SIZE                  (4u)
#define LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED    ((0u != LIS331HH_SPI_INTERNAL_TX_INT_ENABLED) && \
                                                     (LIS331HH_SPI_TX_BUFFER_SIZE > LIS331HH_SPI_FIFO_SIZE))

#define LIS331HH_SPI_RX_SOFTWARE_BUF_ENABLED    ((0u != LIS331HH_SPI_INTERNAL_RX_INT_ENABLED) && \
                                                     (LIS331HH_SPI_RX_BUFFER_SIZE > LIS331HH_SPI_FIFO_SIZE))


/***************************************
*        Data Struct Definition
***************************************/

/* Sleep Mode API Support */
typedef struct
{
    uint8 enableState;
    uint8 cntrPeriod;
    #if(CY_UDB_V0)
        uint8 saveSrTxIntMask;
        uint8 saveSrRxIntMask;
    #endif /* (CY_UDB_V0) */

} LIS331HH_SPI_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void  LIS331HH_SPI_Init(void)                           ;
void  LIS331HH_SPI_Enable(void)                         ;
void  LIS331HH_SPI_Start(void)                          ;
void  LIS331HH_SPI_Stop(void)                           ;

void  LIS331HH_SPI_EnableTxInt(void)                    ;
void  LIS331HH_SPI_EnableRxInt(void)                    ;
void  LIS331HH_SPI_DisableTxInt(void)                   ;
void  LIS331HH_SPI_DisableRxInt(void)                   ;

void  LIS331HH_SPI_Sleep(void)                          ;
void  LIS331HH_SPI_Wakeup(void)                         ;
void  LIS331HH_SPI_SaveConfig(void)                     ;
void  LIS331HH_SPI_RestoreConfig(void)                  ;

void  LIS331HH_SPI_SetTxInterruptMode(uint8 intSrc)     ;
void  LIS331HH_SPI_SetRxInterruptMode(uint8 intSrc)     ;
uint8 LIS331HH_SPI_ReadTxStatus(void)                   ;
uint8 LIS331HH_SPI_ReadRxStatus(void)                   ;
void  LIS331HH_SPI_WriteTxData(uint8 txData)  \
                                                            ;
uint8 LIS331HH_SPI_ReadRxData(void) \
                                                            ;
uint8 LIS331HH_SPI_GetRxBufferSize(void)                ;
uint8 LIS331HH_SPI_GetTxBufferSize(void)                ;
void  LIS331HH_SPI_ClearRxBuffer(void)                  ;
void  LIS331HH_SPI_ClearTxBuffer(void)                  ;
void  LIS331HH_SPI_ClearFIFO(void)                              ;
void  LIS331HH_SPI_PutArray(const uint8 buffer[], uint8 byteCount) \
                                                            ;

#if(0u != LIS331HH_SPI_BIDIRECTIONAL_MODE)
    void  LIS331HH_SPI_TxEnable(void)                   ;
    void  LIS331HH_SPI_TxDisable(void)                  ;
#endif /* (0u != LIS331HH_SPI_BIDIRECTIONAL_MODE) */

CY_ISR_PROTO(LIS331HH_SPI_TX_ISR);
CY_ISR_PROTO(LIS331HH_SPI_RX_ISR);


/**********************************
*   Variable with external linkage
**********************************/

extern uint8 LIS331HH_SPI_initVar;


/***************************************
*           API Constants
***************************************/

#define LIS331HH_SPI_TX_ISR_NUMBER     ((uint8) (LIS331HH_SPI_TxInternalInterrupt__INTC_NUMBER))
#define LIS331HH_SPI_RX_ISR_NUMBER     ((uint8) (LIS331HH_SPI_RxInternalInterrupt__INTC_NUMBER))

#define LIS331HH_SPI_TX_ISR_PRIORITY   ((uint8) (LIS331HH_SPI_TxInternalInterrupt__INTC_PRIOR_NUM))
#define LIS331HH_SPI_RX_ISR_PRIORITY   ((uint8) (LIS331HH_SPI_RxInternalInterrupt__INTC_PRIOR_NUM))


/***************************************
*    Initial Parameter Constants
***************************************/

#define LIS331HH_SPI_INT_ON_SPI_DONE    ((uint8) (0u   << LIS331HH_SPI_STS_SPI_DONE_SHIFT))
#define LIS331HH_SPI_INT_ON_TX_EMPTY    ((uint8) (0u   << LIS331HH_SPI_STS_TX_FIFO_EMPTY_SHIFT))
#define LIS331HH_SPI_INT_ON_TX_NOT_FULL ((uint8) (0u << \
                                                                           LIS331HH_SPI_STS_TX_FIFO_NOT_FULL_SHIFT))
#define LIS331HH_SPI_INT_ON_BYTE_COMP   ((uint8) (0u  << LIS331HH_SPI_STS_BYTE_COMPLETE_SHIFT))
#define LIS331HH_SPI_INT_ON_SPI_IDLE    ((uint8) (0u   << LIS331HH_SPI_STS_SPI_IDLE_SHIFT))

/* Disable TX_NOT_FULL if software buffer is used */
#define LIS331HH_SPI_INT_ON_TX_NOT_FULL_DEF ((LIS331HH_SPI_TX_SOFTWARE_BUF_ENABLED) ? \
                                                                        (0u) : (LIS331HH_SPI_INT_ON_TX_NOT_FULL))

/* TX interrupt mask */
#define LIS331HH_SPI_TX_INIT_INTERRUPTS_MASK    (LIS331HH_SPI_INT_ON_SPI_DONE  | \
                                                     LIS331HH_SPI_INT_ON_TX_EMPTY  | \
                                                     LIS331HH_SPI_INT_ON_TX_NOT_FULL_DEF | \
                                                     LIS331HH_SPI_INT_ON_BYTE_COMP | \
                                                     LIS331HH_SPI_INT_ON_SPI_IDLE)

#define LIS331HH_SPI_INT_ON_RX_FULL         ((uint8) (0u << \
                                                                          LIS331HH_SPI_STS_RX_FIFO_FULL_SHIFT))
#define LIS331HH_SPI_INT_ON_RX_NOT_EMPTY    ((uint8) (0u << \
                                                                          LIS331HH_SPI_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define LIS331HH_SPI_INT_ON_RX_OVER         ((uint8) (0u << \
                                                                          LIS331HH_SPI_STS_RX_FIFO_OVERRUN_SHIFT))

/* RX interrupt mask */
#define LIS331HH_SPI_RX_INIT_INTERRUPTS_MASK    (LIS331HH_SPI_INT_ON_RX_FULL      | \
                                                     LIS331HH_SPI_INT_ON_RX_NOT_EMPTY | \
                                                     LIS331HH_SPI_INT_ON_RX_OVER)
/* Nubmer of bits to receive/transmit */
#define LIS331HH_SPI_BITCTR_INIT            (((uint8) (LIS331HH_SPI_DATA_WIDTH << 1u)) - 1u)


/***************************************
*             Registers
***************************************/

#if(CY_PSOC3 || CY_PSOC5)
    #define LIS331HH_SPI_TXDATA_REG (* (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F0_REG)
    #define LIS331HH_SPI_TXDATA_PTR (  (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F0_REG)
    #define LIS331HH_SPI_RXDATA_REG (* (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F1_REG)
    #define LIS331HH_SPI_RXDATA_PTR (  (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F1_REG)
#else   /* PSOC4 */
    #if(LIS331HH_SPI_USE_SECOND_DATAPATH)
        #define LIS331HH_SPI_TXDATA_REG (* (reg16 *) \
                                          LIS331HH_SPI_BSPIM_sR8_Dp_u0__16BIT_F0_REG)
        #define LIS331HH_SPI_TXDATA_PTR (  (reg16 *) \
                                          LIS331HH_SPI_BSPIM_sR8_Dp_u0__16BIT_F0_REG)
        #define LIS331HH_SPI_RXDATA_REG (* (reg16 *) \
                                          LIS331HH_SPI_BSPIM_sR8_Dp_u0__16BIT_F1_REG)
        #define LIS331HH_SPI_RXDATA_PTR         (  (reg16 *) \
                                          LIS331HH_SPI_BSPIM_sR8_Dp_u0__16BIT_F1_REG)
    #else
        #define LIS331HH_SPI_TXDATA_REG (* (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F0_REG)
        #define LIS331HH_SPI_TXDATA_PTR (  (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F0_REG)
        #define LIS331HH_SPI_RXDATA_REG (* (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F1_REG)
        #define LIS331HH_SPI_RXDATA_PTR (  (reg8 *) \
                                                LIS331HH_SPI_BSPIM_sR8_Dp_u0__F1_REG)
    #endif /* (LIS331HH_SPI_USE_SECOND_DATAPATH) */
#endif     /* (CY_PSOC3 || CY_PSOC5) */

#define LIS331HH_SPI_AUX_CONTROL_DP0_REG (* (reg8 *) \
                                        LIS331HH_SPI_BSPIM_sR8_Dp_u0__DP_AUX_CTL_REG)
#define LIS331HH_SPI_AUX_CONTROL_DP0_PTR (  (reg8 *) \
                                        LIS331HH_SPI_BSPIM_sR8_Dp_u0__DP_AUX_CTL_REG)

#if(LIS331HH_SPI_USE_SECOND_DATAPATH)
    #define LIS331HH_SPI_AUX_CONTROL_DP1_REG  (* (reg8 *) \
                                        LIS331HH_SPI_BSPIM_sR8_Dp_u1__DP_AUX_CTL_REG)
    #define LIS331HH_SPI_AUX_CONTROL_DP1_PTR  (  (reg8 *) \
                                        LIS331HH_SPI_BSPIM_sR8_Dp_u1__DP_AUX_CTL_REG)
#endif /* (LIS331HH_SPI_USE_SECOND_DATAPATH) */

#define LIS331HH_SPI_COUNTER_PERIOD_REG     (* (reg8 *) LIS331HH_SPI_BSPIM_BitCounter__PERIOD_REG)
#define LIS331HH_SPI_COUNTER_PERIOD_PTR     (  (reg8 *) LIS331HH_SPI_BSPIM_BitCounter__PERIOD_REG)
#define LIS331HH_SPI_COUNTER_CONTROL_REG    (* (reg8 *) LIS331HH_SPI_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)
#define LIS331HH_SPI_COUNTER_CONTROL_PTR    (  (reg8 *) LIS331HH_SPI_BSPIM_BitCounter__CONTROL_AUX_CTL_REG)

#define LIS331HH_SPI_TX_STATUS_REG          (* (reg8 *) LIS331HH_SPI_BSPIM_TxStsReg__STATUS_REG)
#define LIS331HH_SPI_TX_STATUS_PTR          (  (reg8 *) LIS331HH_SPI_BSPIM_TxStsReg__STATUS_REG)
#define LIS331HH_SPI_RX_STATUS_REG          (* (reg8 *) LIS331HH_SPI_BSPIM_RxStsReg__STATUS_REG)
#define LIS331HH_SPI_RX_STATUS_PTR          (  (reg8 *) LIS331HH_SPI_BSPIM_RxStsReg__STATUS_REG)

#define LIS331HH_SPI_CONTROL_REG            (* (reg8 *) \
                                      LIS331HH_SPI_BSPIM_BidirMode_SyncCtl_CtrlReg__CONTROL_REG)
#define LIS331HH_SPI_CONTROL_PTR            (  (reg8 *) \
                                      LIS331HH_SPI_BSPIM_BidirMode_SyncCtl_CtrlReg__CONTROL_REG)

#define LIS331HH_SPI_TX_STATUS_MASK_REG     (* (reg8 *) LIS331HH_SPI_BSPIM_TxStsReg__MASK_REG)
#define LIS331HH_SPI_TX_STATUS_MASK_PTR     (  (reg8 *) LIS331HH_SPI_BSPIM_TxStsReg__MASK_REG)
#define LIS331HH_SPI_RX_STATUS_MASK_REG     (* (reg8 *) LIS331HH_SPI_BSPIM_RxStsReg__MASK_REG)
#define LIS331HH_SPI_RX_STATUS_MASK_PTR     (  (reg8 *) LIS331HH_SPI_BSPIM_RxStsReg__MASK_REG)

#define LIS331HH_SPI_TX_STATUS_ACTL_REG     (* (reg8 *) LIS331HH_SPI_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define LIS331HH_SPI_TX_STATUS_ACTL_PTR     (  (reg8 *) LIS331HH_SPI_BSPIM_TxStsReg__STATUS_AUX_CTL_REG)
#define LIS331HH_SPI_RX_STATUS_ACTL_REG     (* (reg8 *) LIS331HH_SPI_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)
#define LIS331HH_SPI_RX_STATUS_ACTL_PTR     (  (reg8 *) LIS331HH_SPI_BSPIM_RxStsReg__STATUS_AUX_CTL_REG)

#if(LIS331HH_SPI_USE_SECOND_DATAPATH)
    #define LIS331HH_SPI_AUX_CONTROLDP1     (LIS331HH_SPI_AUX_CONTROL_DP1_REG)
#endif /* (LIS331HH_SPI_USE_SECOND_DATAPATH) */


/***************************************
*       Register Constants
***************************************/

/* Status Register Definitions */
#define LIS331HH_SPI_STS_SPI_DONE_SHIFT             (0x00u)
#define LIS331HH_SPI_STS_TX_FIFO_EMPTY_SHIFT        (0x01u)
#define LIS331HH_SPI_STS_TX_FIFO_NOT_FULL_SHIFT     (0x02u)
#define LIS331HH_SPI_STS_BYTE_COMPLETE_SHIFT        (0x03u)
#define LIS331HH_SPI_STS_SPI_IDLE_SHIFT             (0x04u)
#define LIS331HH_SPI_STS_RX_FIFO_FULL_SHIFT         (0x04u)
#define LIS331HH_SPI_STS_RX_FIFO_NOT_EMPTY_SHIFT    (0x05u)
#define LIS331HH_SPI_STS_RX_FIFO_OVERRUN_SHIFT      (0x06u)

#define LIS331HH_SPI_STS_SPI_DONE           ((uint8) (0x01u << LIS331HH_SPI_STS_SPI_DONE_SHIFT))
#define LIS331HH_SPI_STS_TX_FIFO_EMPTY      ((uint8) (0x01u << LIS331HH_SPI_STS_TX_FIFO_EMPTY_SHIFT))
#define LIS331HH_SPI_STS_TX_FIFO_NOT_FULL   ((uint8) (0x01u << LIS331HH_SPI_STS_TX_FIFO_NOT_FULL_SHIFT))
#define LIS331HH_SPI_STS_BYTE_COMPLETE      ((uint8) (0x01u << LIS331HH_SPI_STS_BYTE_COMPLETE_SHIFT))
#define LIS331HH_SPI_STS_SPI_IDLE           ((uint8) (0x01u << LIS331HH_SPI_STS_SPI_IDLE_SHIFT))
#define LIS331HH_SPI_STS_RX_FIFO_FULL       ((uint8) (0x01u << LIS331HH_SPI_STS_RX_FIFO_FULL_SHIFT))
#define LIS331HH_SPI_STS_RX_FIFO_NOT_EMPTY  ((uint8) (0x01u << LIS331HH_SPI_STS_RX_FIFO_NOT_EMPTY_SHIFT))
#define LIS331HH_SPI_STS_RX_FIFO_OVERRUN    ((uint8) (0x01u << LIS331HH_SPI_STS_RX_FIFO_OVERRUN_SHIFT))

/* TX and RX masks for clear on read bits */
#define LIS331HH_SPI_TX_STS_CLR_ON_RD_BYTES_MASK    (0x09u)
#define LIS331HH_SPI_RX_STS_CLR_ON_RD_BYTES_MASK    (0x40u)

/* StatusI Register Interrupt Enable Control Bits */
/* As defined by the Register map for the AUX Control Register */
#define LIS331HH_SPI_INT_ENABLE     (0x10u) /* Enable interrupt from statusi */
#define LIS331HH_SPI_TX_FIFO_CLR    (0x01u) /* F0 - TX FIFO */
#define LIS331HH_SPI_RX_FIFO_CLR    (0x02u) /* F1 - RX FIFO */
#define LIS331HH_SPI_FIFO_CLR       (LIS331HH_SPI_TX_FIFO_CLR | LIS331HH_SPI_RX_FIFO_CLR)

/* Bit Counter (7-bit) Control Register Bit Definitions */
/* As defined by the Register map for the AUX Control Register */
#define LIS331HH_SPI_CNTR_ENABLE    (0x20u) /* Enable CNT7 */

/* Bi-Directional mode control bit */
#define LIS331HH_SPI_CTRL_TX_SIGNAL_EN  (0x01u)

/* Datapath Auxillary Control Register definitions */
#define LIS331HH_SPI_AUX_CTRL_FIFO0_CLR         (0x01u)
#define LIS331HH_SPI_AUX_CTRL_FIFO1_CLR         (0x02u)
#define LIS331HH_SPI_AUX_CTRL_FIFO0_LVL         (0x04u)
#define LIS331HH_SPI_AUX_CTRL_FIFO1_LVL         (0x08u)
#define LIS331HH_SPI_STATUS_ACTL_INT_EN_MASK    (0x10u)

/* Component disabled */
#define LIS331HH_SPI_DISABLED   (0u)


/***************************************
*       Macros
***************************************/

/* Returns true if componentn enabled */
#define LIS331HH_SPI_IS_ENABLED (0u != (LIS331HH_SPI_TX_STATUS_ACTL_REG & LIS331HH_SPI_INT_ENABLE))

/* Retuns TX status register */
#define LIS331HH_SPI_GET_STATUS_TX(swTxSts) ( (uint8)(LIS331HH_SPI_TX_STATUS_REG | \
                                                          ((swTxSts) & LIS331HH_SPI_TX_STS_CLR_ON_RD_BYTES_MASK)) )
/* Retuns RX status register */
#define LIS331HH_SPI_GET_STATUS_RX(swRxSts) ( (uint8)(LIS331HH_SPI_RX_STATUS_REG | \
                                                          ((swRxSts) & LIS331HH_SPI_RX_STS_CLR_ON_RD_BYTES_MASK)) )


/***************************************
*       Obsolete definitions
***************************************/

/* Following definitions are for version compatibility.
*  They are obsolete in SPIM v2_30.
*  Please do not use it in new projects
*/

#define LIS331HH_SPI_WriteByte   LIS331HH_SPI_WriteTxData
#define LIS331HH_SPI_ReadByte    LIS331HH_SPI_ReadRxData
void  LIS331HH_SPI_SetInterruptMode(uint8 intSrc)       ;
uint8 LIS331HH_SPI_ReadStatus(void)                     ;
void  LIS331HH_SPI_EnableInt(void)                      ;
void  LIS331HH_SPI_DisableInt(void)                     ;

/* Obsolete register names. Not to be used in new designs */
#define LIS331HH_SPI_TXDATA                 (LIS331HH_SPI_TXDATA_REG)
#define LIS331HH_SPI_RXDATA                 (LIS331HH_SPI_RXDATA_REG)
#define LIS331HH_SPI_AUX_CONTROLDP0         (LIS331HH_SPI_AUX_CONTROL_DP0_REG)
#define LIS331HH_SPI_TXBUFFERREAD           (LIS331HH_SPI_txBufferRead)
#define LIS331HH_SPI_TXBUFFERWRITE          (LIS331HH_SPI_txBufferWrite)
#define LIS331HH_SPI_RXBUFFERREAD           (LIS331HH_SPI_rxBufferRead)
#define LIS331HH_SPI_RXBUFFERWRITE          (LIS331HH_SPI_rxBufferWrite)

#define LIS331HH_SPI_COUNTER_PERIOD         (LIS331HH_SPI_COUNTER_PERIOD_REG)
#define LIS331HH_SPI_COUNTER_CONTROL        (LIS331HH_SPI_COUNTER_CONTROL_REG)
#define LIS331HH_SPI_STATUS                 (LIS331HH_SPI_TX_STATUS_REG)
#define LIS331HH_SPI_CONTROL                (LIS331HH_SPI_CONTROL_REG)
#define LIS331HH_SPI_STATUS_MASK            (LIS331HH_SPI_TX_STATUS_MASK_REG)
#define LIS331HH_SPI_STATUS_ACTL            (LIS331HH_SPI_TX_STATUS_ACTL_REG)

#define LIS331HH_SPI_INIT_INTERRUPTS_MASK  (LIS331HH_SPI_INT_ON_SPI_DONE     | \
                                                LIS331HH_SPI_INT_ON_TX_EMPTY     | \
                                                LIS331HH_SPI_INT_ON_TX_NOT_FULL_DEF  | \
                                                LIS331HH_SPI_INT_ON_RX_FULL      | \
                                                LIS331HH_SPI_INT_ON_RX_NOT_EMPTY | \
                                                LIS331HH_SPI_INT_ON_RX_OVER      | \
                                                LIS331HH_SPI_INT_ON_BYTE_COMP)
                                                
/* Following definitions are for version Compatibility.
*  They are obsolete in SPIM v2_40.
*  Please do not use it in new projects
*/

#define LIS331HH_SPI_DataWidth                  (LIS331HH_SPI_DATA_WIDTH)
#define LIS331HH_SPI_InternalClockUsed          (LIS331HH_SPI_INTERNAL_CLOCK)
#define LIS331HH_SPI_InternalTxInterruptEnabled (LIS331HH_SPI_INTERNAL_TX_INT_ENABLED)
#define LIS331HH_SPI_InternalRxInterruptEnabled (LIS331HH_SPI_INTERNAL_RX_INT_ENABLED)
#define LIS331HH_SPI_ModeUseZero                (LIS331HH_SPI_MODE_USE_ZERO)
#define LIS331HH_SPI_BidirectionalMode          (LIS331HH_SPI_BIDIRECTIONAL_MODE)
#define LIS331HH_SPI_Mode                       (LIS331HH_SPI_MODE)
#define LIS331HH_SPI_DATAWIDHT                  (LIS331HH_SPI_DATA_WIDTH)
#define LIS331HH_SPI_InternalInterruptEnabled   (0u)

#define LIS331HH_SPI_TXBUFFERSIZE   (LIS331HH_SPI_TX_BUFFER_SIZE)
#define LIS331HH_SPI_RXBUFFERSIZE   (LIS331HH_SPI_RX_BUFFER_SIZE)

#define LIS331HH_SPI_TXBUFFER       LIS331HH_SPI_txBuffer
#define LIS331HH_SPI_RXBUFFER       LIS331HH_SPI_rxBuffer

#endif /* (CY_SPIM_LIS331HH_SPI_H) */


/* [] END OF FILE */
