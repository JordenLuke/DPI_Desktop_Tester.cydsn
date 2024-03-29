/*******************************************************************************
* File Name: MAG_I2C.h
* Version 3.30
*
* Description:
*  This file provides constants and parameter values for the I2C component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_I2C_MAG_I2C_H)
#define CY_I2C_MAG_I2C_H

#include "cyfitter.h"
#include "cytypes.h"
#include "CyLib.h"

/* Check if required defines such as CY_PSOC5LP are available in cy_boot */
#if !defined (CY_PSOC5LP)
    #error Component I2C_v3_30 requires cy_boot v3.10 or later
#endif /* (CY_PSOC5LP) */


/***************************************
*   Conditional Compilation Parameters
****************************************/

#define MAG_I2C_IMPLEMENTATION     (0u)
#define MAG_I2C_MODE               (2u)
#define MAG_I2C_ENABLE_WAKEUP      (0u)
#define MAG_I2C_ADDR_DECODE        (1u)
#define MAG_I2C_UDB_INTRN_CLOCK    (0u)


/* I2C implementation enum */
#define MAG_I2C_UDB    (0x00u)
#define MAG_I2C_FF     (0x01u)

#define MAG_I2C_FF_IMPLEMENTED     (MAG_I2C_FF  == MAG_I2C_IMPLEMENTATION)
#define MAG_I2C_UDB_IMPLEMENTED    (MAG_I2C_UDB == MAG_I2C_IMPLEMENTATION)

#define MAG_I2C_UDB_INTRN_CLOCK_ENABLED    (MAG_I2C_UDB_IMPLEMENTED && \
                                                     (0u != MAG_I2C_UDB_INTRN_CLOCK))
/* I2C modes enum */
#define MAG_I2C_MODE_SLAVE                 (0x01u)
#define MAG_I2C_MODE_MASTER                (0x02u)
#define MAG_I2C_MODE_MULTI_MASTER          (0x06u)
#define MAG_I2C_MODE_MULTI_MASTER_SLAVE    (0x07u)
#define MAG_I2C_MODE_MULTI_MASTER_MASK     (0x04u)

#define MAG_I2C_MODE_SLAVE_ENABLED         (0u != (MAG_I2C_MODE_SLAVE  & MAG_I2C_MODE))
#define MAG_I2C_MODE_MASTER_ENABLED        (0u != (MAG_I2C_MODE_MASTER & MAG_I2C_MODE))
#define MAG_I2C_MODE_MULTI_MASTER_ENABLED  (0u != (MAG_I2C_MODE_MULTI_MASTER_MASK & \
                                                            MAG_I2C_MODE))
#define MAG_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED    (MAG_I2C_MODE_MULTI_MASTER_SLAVE == \
                                                             MAG_I2C_MODE)

/* Address detection enum */
#define MAG_I2C_SW_DECODE      (0x00u)
#define MAG_I2C_HW_DECODE      (0x01u)

#define MAG_I2C_SW_ADRR_DECODE             (MAG_I2C_SW_DECODE == MAG_I2C_ADDR_DECODE)
#define MAG_I2C_HW_ADRR_DECODE             (MAG_I2C_HW_DECODE == MAG_I2C_ADDR_DECODE)

/* Wakeup enabled */
#define MAG_I2C_WAKEUP_ENABLED             (0u != MAG_I2C_ENABLE_WAKEUP)

/* Adds bootloader APIs to component */
#define MAG_I2C_BOOTLOADER_INTERFACE_ENABLED   (MAG_I2C_MODE_SLAVE_ENABLED && \
                                                            ((CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MAG_I2C) || \
                                                             (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_Custom_Interface)))

/* Timeout functionality */
#define MAG_I2C_TIMEOUT_ENABLE             (0u)
#define MAG_I2C_TIMEOUT_SCL_TMOUT_ENABLE   (0u)
#define MAG_I2C_TIMEOUT_SDA_TMOUT_ENABLE   (0u)
#define MAG_I2C_TIMEOUT_PRESCALER_ENABLE   (0u)
#define MAG_I2C_TIMEOUT_IMPLEMENTATION     (0u)

/* Convert to boolean */
#define MAG_I2C_TIMEOUT_ENABLED            (0u != MAG_I2C_TIMEOUT_ENABLE)
#define MAG_I2C_TIMEOUT_SCL_TMOUT_ENABLED  (0u != MAG_I2C_TIMEOUT_SCL_TMOUT_ENABLE)
#define MAG_I2C_TIMEOUT_SDA_TMOUT_ENABLED  (0u != MAG_I2C_TIMEOUT_SDA_TMOUT_ENABLE)
#define MAG_I2C_TIMEOUT_PRESCALER_ENABLED  (0u != MAG_I2C_TIMEOUT_PRESCALER_ENABLE)

/* Timeout implementation enum. */
#define MAG_I2C_TIMEOUT_UDB    (0x00u)
#define MAG_I2C_TIMEOUT_FF     (0x01u)

#define MAG_I2C_TIMEOUT_FF_IMPLEMENTED     (MAG_I2C_TIMEOUT_FF  == \
                                                        MAG_I2C_TIMEOUT_IMPLEMENTATION)
#define MAG_I2C_TIMEOUT_UDB_IMPLEMENTED    (MAG_I2C_TIMEOUT_UDB == \
                                                        MAG_I2C_TIMEOUT_IMPLEMENTATION)

#define MAG_I2C_TIMEOUT_FF_ENABLED         (MAG_I2C_TIMEOUT_ENABLED && \
                                                     MAG_I2C_TIMEOUT_FF_IMPLEMENTED && \
                                                     CY_PSOC5LP)

#define MAG_I2C_TIMEOUT_UDB_ENABLED        (MAG_I2C_TIMEOUT_ENABLED && \
                                                     MAG_I2C_TIMEOUT_UDB_IMPLEMENTED)

#define MAG_I2C_EXTERN_I2C_INTR_HANDLER    (0u)
#define MAG_I2C_EXTERN_TMOUT_INTR_HANDLER  (0u)

#define MAG_I2C_INTERN_I2C_INTR_HANDLER    (0u == MAG_I2C_EXTERN_I2C_INTR_HANDLER)
#define MAG_I2C_INTERN_TMOUT_INTR_HANDLER  (0u == MAG_I2C_EXTERN_TMOUT_INTR_HANDLER)


/***************************************
*       Type defines
***************************************/

/* Structure to save registers before go to sleep */
typedef struct
{
    uint8 enableState;

    #if(MAG_I2C_FF_IMPLEMENTED)
        uint8 xcfg;
        uint8 cfg;

        #if(MAG_I2C_MODE_SLAVE_ENABLED)
            uint8 addr;
        #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */

        #if(CY_PSOC5A)
            uint8 clkDiv;
        #else
            uint8 clkDiv1;
            uint8 clkDiv2;
        #endif /* (CY_PSOC5A) */

    #else
        uint8 control;

        #if(CY_UDB_V0)
            uint8 intMask;

            #if(MAG_I2C_MODE_SLAVE_ENABLED)
                uint8 addr;
            #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */
        #endif     /* (CY_UDB_V0) */

    #endif /* (MAG_I2C_FF_IMPLEMENTED) */

    #if(MAG_I2C_TIMEOUT_ENABLED)
        uint16 tmoutCfg;
        uint8  tmoutIntr;

        #if(MAG_I2C_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0)
            uint8 tmoutPrd;
        #endif /* (MAG_I2C_TIMEOUT_PRESCALER_ENABLED && CY_UDB_V0) */

    #endif /* (MAG_I2C_TIMEOUT_ENABLED) */

} MAG_I2C_BACKUP_STRUCT;


/***************************************
*        Function Prototypes
***************************************/

void MAG_I2C_Init(void)                            ;
void MAG_I2C_Enable(void)                          ;

void MAG_I2C_Start(void)                           ;
void MAG_I2C_Stop(void)                            ;

#define MAG_I2C_EnableInt()        CyIntEnable      (MAG_I2C_ISR_NUMBER)
#define MAG_I2C_DisableInt()       CyIntDisable     (MAG_I2C_ISR_NUMBER)
#define MAG_I2C_ClearPendingInt()  CyIntClearPending(MAG_I2C_ISR_NUMBER)
#define MAG_I2C_SetPendingInt()    CyIntSetPending  (MAG_I2C_ISR_NUMBER)

void MAG_I2C_SaveConfig(void)                      ;
void MAG_I2C_Sleep(void)                           ;
void MAG_I2C_RestoreConfig(void)                   ;
void MAG_I2C_Wakeup(void)                          ;

/* I2C Master functions prototypes */
#if(MAG_I2C_MODE_MASTER_ENABLED)
    /* Read and Clear status functions */
    uint8 MAG_I2C_MasterStatus(void)                ;
    uint8 MAG_I2C_MasterClearStatus(void)           ;

    /* Interrupt based operation functions */
    uint8 MAG_I2C_MasterWriteBuf(uint8 slaveAddress, uint8 * wrData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 MAG_I2C_MasterReadBuf(uint8 slaveAddress, uint8 * rdData, uint8 cnt, uint8 mode) \
                                                            ;
    uint8 MAG_I2C_MasterGetReadBufSize(void)       ;
    uint8 MAG_I2C_MasterGetWriteBufSize(void)      ;
    void  MAG_I2C_MasterClearReadBuf(void)         ;
    void  MAG_I2C_MasterClearWriteBuf(void)        ;

    /* Manual operation functions */
    uint8 MAG_I2C_MasterSendStart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 MAG_I2C_MasterSendRestart(uint8 slaveAddress, uint8 R_nW) \
                                                            ;
    uint8 MAG_I2C_MasterSendStop(void)             ;
    uint8 MAG_I2C_MasterWriteByte(uint8 theByte)   ;
    uint8 MAG_I2C_MasterReadByte(uint8 acknNak)    ;

    /* This fake function use as workaround */
    void  MAG_I2C_Workaround(void)                 ;

#endif /* (MAG_I2C_MODE_MASTER_ENABLED) */

/* I2C Slave functions prototypes */
#if(MAG_I2C_MODE_SLAVE_ENABLED)
    /* Read and Clear status functions */
    uint8 MAG_I2C_SlaveStatus(void)                ;
    uint8 MAG_I2C_SlaveClearReadStatus(void)       ;
    uint8 MAG_I2C_SlaveClearWriteStatus(void)      ;

    void  MAG_I2C_SlaveSetAddress(uint8 address)   ;

    /* Interrupt based operation functions */
    void  MAG_I2C_SlaveInitReadBuf(uint8 * rdBuf, uint8 bufSize) \
                                                            ;
    void  MAG_I2C_SlaveInitWriteBuf(uint8 * wrBuf, uint8 bufSize) \
                                                            ;
    uint8 MAG_I2C_SlaveGetReadBufSize(void)        ;
    uint8 MAG_I2C_SlaveGetWriteBufSize(void)       ;
    void  MAG_I2C_SlaveClearReadBuf(void)          ;
    void  MAG_I2C_SlaveClearWriteBuf(void)         ;

    /* Communication bootloader I2C Slave APIs */
    #if defined(CYDEV_BOOTLOADER_IO_COMP) && (MAG_I2C_BOOTLOADER_INTERFACE_ENABLED)
        /* Physical layer functions */
        void     MAG_I2C_CyBtldrCommStart(void) CYSMALL \
                                                            ;
        void     MAG_I2C_CyBtldrCommStop(void)  CYSMALL \
                                                            ;
        void     MAG_I2C_CyBtldrCommReset(void) CYSMALL \
                                                            ;
        cystatus MAG_I2C_CyBtldrCommWrite(const uint8 pData[], uint16 size, uint16 * count, uint8 timeOut) \
                                                        CYSMALL ;
        cystatus MAG_I2C_CyBtldrCommRead(uint8 pData[], uint16 size, uint16 * count, uint8 timeOut)  CYSMALL \
                                                            ;

        #if(CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MAG_I2C)
            #define CyBtldrCommStart    MAG_I2C_CyBtldrCommStart
            #define CyBtldrCommStop     MAG_I2C_CyBtldrCommStop
            #define CyBtldrCommReset    MAG_I2C_CyBtldrCommReset
            #define CyBtldrCommWrite    MAG_I2C_CyBtldrCommWrite
            #define CyBtldrCommRead     MAG_I2C_CyBtldrCommRead
        #endif /* (CYDEV_BOOTLOADER_IO_COMP == CyBtldr_MAG_I2C) */

        /* Size of Read/Write buffers for I2C bootloader  */
        #define MAG_I2C_BTLDR_SIZEOF_READ_BUFFER   (0x80u)
        #define MAG_I2C_BTLDR_SIZEOF_WRITE_BUFFER  (0x80u)
        #define MAG_I2C_MIN_UNT16(a, b)            ( ((uint16)(a) < (b)) ? ((uint16) (a)) : ((uint16) (b)) )
        #define MAG_I2C_WAIT_1_MS                  (1u)

    #endif /* defined(CYDEV_BOOTLOADER_IO_COMP) && (MAG_I2C_BOOTLOADER_INTERFACE_ENABLED) */

#endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */

/* I2C interrupt handler */
CY_ISR_PROTO(MAG_I2C_ISR);
#if((MAG_I2C_FF_IMPLEMENTED) || (MAG_I2C_WAKEUP_ENABLED))
    CY_ISR_PROTO(MAG_I2C_WAKEUP_ISR);
#endif /* ((MAG_I2C_FF_IMPLEMENTED) || (MAG_I2C_WAKEUP_ENABLED)) */


/**********************************
*   Variable with external linkage
**********************************/

extern uint8 MAG_I2C_initVar;


/***************************************
*   Initial Parameter Constants
***************************************/

#define MAG_I2C_DATA_RATE          (100u)
#define MAG_I2C_DEFAULT_ADDR       (8u)
#define MAG_I2C_I2C_PAIR_SELECTED  (0u)

/* I2C pair enum */
#define MAG_I2C_I2C_PAIR_ANY   (0x01u) /* Any pins for I2C */
#define MAG_I2C_I2C_PAIR0      (0x01u) /* I2C0: (SCL = P12[4]) && (SCL = P12[5]) */
#define MAG_I2C_I2C_PAIR1      (0x02u) /* I2C1: (SCL = P12[0]) && (SDA = P12[1]) */

#define MAG_I2C_I2C1_SIO_PAIR  (MAG_I2C_I2C_PAIR1 == MAG_I2C_I2C_PAIR_SELECTED)
#define MAG_I2C_I2C0_SIO_PAIR  (MAG_I2C_I2C_PAIR0 == MAG_I2C_I2C_PAIR_SELECTED)


/***************************************
*            API Constants
***************************************/

/* Master/Slave control constants */
#define MAG_I2C_READ_XFER_MODE     (0x01u) /* Read */
#define MAG_I2C_WRITE_XFER_MODE    (0x00u) /* Write */
#define MAG_I2C_ACK_DATA           (0x01u) /* Send ACK */
#define MAG_I2C_NAK_DATA           (0x00u) /* Send NAK */
#define MAG_I2C_OVERFLOW_RETURN    (0xFFu) /* Senf on bus in case of overflow */

#if(MAG_I2C_MODE_MASTER_ENABLED)
    /* "Mode" constants for MasterWriteBuf() or MasterReadBuf() function */
    #define MAG_I2C_MODE_COMPLETE_XFER     (0x00u) /* Full transfer with Start and Stop */
    #define MAG_I2C_MODE_REPEAT_START      (0x01u) /* Begin with a ReStart instead of a Start */
    #define MAG_I2C_MODE_NO_STOP           (0x02u) /* Complete the transfer without a Stop */

    /* Master status */
    #define MAG_I2C_MSTAT_CLEAR            (0x00u) /* Clear (init) status value */

    #define MAG_I2C_MSTAT_RD_CMPLT         (0x01u) /* Read complete */
    #define MAG_I2C_MSTAT_WR_CMPLT         (0x02u) /* Write complete */
    #define MAG_I2C_MSTAT_XFER_INP         (0x04u) /* Master transfer in progress */
    #define MAG_I2C_MSTAT_XFER_HALT        (0x08u) /* Transfer is halted */

    #define MAG_I2C_MSTAT_ERR_MASK         (0xF0u) /* Mask for all errors */
    #define MAG_I2C_MSTAT_ERR_SHORT_XFER   (0x10u) /* Master NAKed before end of packet */
    #define MAG_I2C_MSTAT_ERR_ADDR_NAK     (0x20u) /* Slave did not ACK */
    #define MAG_I2C_MSTAT_ERR_ARB_LOST     (0x40u) /* Master lost arbitration during communication */
    #define MAG_I2C_MSTAT_ERR_XFER         (0x80u) /* Error during transfer */

    /* Master API returns */
    #define MAG_I2C_MSTR_NO_ERROR          (0x00u) /* Function complete without error */
    #define MAG_I2C_MSTR_BUS_BUSY          (0x01u) /* Bus is busy, process not started */
    #define MAG_I2C_MSTR_NOT_READY         (0x02u) /* Master not Master on the bus or */
                                                            /*  Slave operation in progress */
    #define MAG_I2C_MSTR_ERR_LB_NAK        (0x03u) /* Last Byte Naked */
    #define MAG_I2C_MSTR_ERR_ARB_LOST      (0x04u) /* Master lost arbitration during communication */
    #define MAG_I2C_MSTR_ERR_ABORT_START_GEN  (0x05u) /* Master did not generate Start, the Slave */
                                                               /* was addressed before */

#endif /* (MAG_I2C_MODE_MASTER_ENABLED) */

#if(MAG_I2C_MODE_SLAVE_ENABLED)
    /* Slave Status Constants */
    #define MAG_I2C_SSTAT_RD_CMPLT     (0x01u) /* Read transfer complete */
    #define MAG_I2C_SSTAT_RD_BUSY      (0x02u) /* Read transfer in progress */
    #define MAG_I2C_SSTAT_RD_ERR_OVFL  (0x04u) /* Read overflow Error */
    #define MAG_I2C_SSTAT_RD_MASK      (0x0Fu) /* Read Status Mask */
    #define MAG_I2C_SSTAT_RD_NO_ERR    (0x00u) /* Read no Error */

    #define MAG_I2C_SSTAT_WR_CMPLT     (0x10u) /* Write transfer complete */
    #define MAG_I2C_SSTAT_WR_BUSY      (0x20u) /* Write transfer in progress */
    #define MAG_I2C_SSTAT_WR_ERR_OVFL  (0x40u) /* Write overflow Error */
    #define MAG_I2C_SSTAT_WR_MASK      (0xF0u) /* Write Status Mask  */
    #define MAG_I2C_SSTAT_WR_NO_ERR    (0x00u) /* Write no Error */

    #define MAG_I2C_SSTAT_RD_CLEAR     (0x0Du) /* Read Status clear */
    #define MAG_I2C_SSTAT_WR_CLEAR     (0xD0u) /* Write Status Clear */

#endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */


/***************************************
*       I2C state machine constants
***************************************/

/* Default slave address states */
#define  MAG_I2C_SM_IDLE           (0x10u) /* Default state - IDLE */
#define  MAG_I2C_SM_EXIT_IDLE      (0x00u) /* Pass master and slave processing and go to IDLE */

/* Slave mode states */
#define  MAG_I2C_SM_SLAVE          (MAG_I2C_SM_IDLE) /* Any Slave state */
#define  MAG_I2C_SM_SL_WR_DATA     (0x11u) /* Master writes data to slzve  */
#define  MAG_I2C_SM_SL_RD_DATA     (0x12u) /* Master reads data from slave */

/* Master mode states */
#define  MAG_I2C_SM_MASTER         (0x40u) /* Any master state */

#define  MAG_I2C_SM_MSTR_RD        (0x08u) /* Any master read state          */
#define  MAG_I2C_SM_MSTR_RD_ADDR   (0x49u) /* Master sends address with read */
#define  MAG_I2C_SM_MSTR_RD_DATA   (0x4Au) /* Master reads data              */

#define  MAG_I2C_SM_MSTR_WR        (0x04u) /* Any master read state           */
#define  MAG_I2C_SM_MSTR_WR_ADDR   (0x45u) /* Master sends address with write */
#define  MAG_I2C_SM_MSTR_WR_DATA   (0x46u) /* Master writes data              */

#define  MAG_I2C_SM_MSTR_HALT      (0x60u) /* Master waits for ReStart */

#define MAG_I2C_CHECK_SM_MASTER    (0u != (MAG_I2C_SM_MASTER & MAG_I2C_state))
#define MAG_I2C_CHECK_SM_SLAVE     (0u != (MAG_I2C_SM_SLAVE  & MAG_I2C_state))


/***************************************
*              Registers
***************************************/

#if(MAG_I2C_FF_IMPLEMENTED)
    /* Fixed Function registers */
    #define MAG_I2C_XCFG_REG           (* (reg8 *) MAG_I2C_I2C_FF__XCFG)
    #define MAG_I2C_XCFG_PTR           (  (reg8 *) MAG_I2C_I2C_FF__XCFG)

    #define MAG_I2C_ADDR_REG           (* (reg8 *) MAG_I2C_I2C_FF__ADR)
    #define MAG_I2C_ADDR_PTR           (  (reg8 *) MAG_I2C_I2C_FF__ADR)

    #define MAG_I2C_CFG_REG            (* (reg8 *) MAG_I2C_I2C_FF__CFG)
    #define MAG_I2C_CFG_PTR            (  (reg8 *) MAG_I2C_I2C_FF__CFG)

    #define MAG_I2C_CSR_REG            (* (reg8 *) MAG_I2C_I2C_FF__CSR)
    #define MAG_I2C_CSR_PTR            (  (reg8 *) MAG_I2C_I2C_FF__CSR)

    #define MAG_I2C_DATA_REG           (* (reg8 *) MAG_I2C_I2C_FF__D)
    #define MAG_I2C_DATA_PTR           (  (reg8 *) MAG_I2C_I2C_FF__D)

    #define MAG_I2C_MCSR_REG           (* (reg8 *) MAG_I2C_I2C_FF__MCSR)
    #define MAG_I2C_MCSR_PTR           (  (reg8 *) MAG_I2C_I2C_FF__MCSR)

    #define MAG_I2C_ACT_PWRMGR_REG     (* (reg8 *) MAG_I2C_I2C_FF__PM_ACT_CFG)
    #define MAG_I2C_ACT_PWRMGR_PTR     (  (reg8 *) MAG_I2C_I2C_FF__PM_ACT_CFG)
    #define MAG_I2C_ACT_PWR_EN         (  (uint8)  MAG_I2C_I2C_FF__PM_ACT_MSK)

    #define MAG_I2C_STBY_PWRMGR_REG    (* (reg8 *) MAG_I2C_I2C_FF__PM_STBY_CFG)
    #define MAG_I2C_STBY_PWRMGR_PTR    (  (reg8 *) MAG_I2C_I2C_FF__PM_STBY_CFG)
    #define MAG_I2C_STBY_PWR_EN        (  (uint8)  MAG_I2C_I2C_FF__PM_STBY_MSK)

    #define MAG_I2C_PWRSYS_CR1_REG     (* (reg8 *) CYREG_PWRSYS_CR1)
    #define MAG_I2C_PWRSYS_CR1_PTR     (  (reg8 *) CYREG_PWRSYS_CR1)

    /* Clock divider register depends on silicon */
    #if(CY_PSOC5A)
        #define MAG_I2C_CLKDIV_REG     (* (reg8 *) MAG_I2C_I2C_FF__CLK_DIV)
        #define MAG_I2C_CLKDIV_PTR     (  (reg8 *) MAG_I2C_I2C_FF__CLK_DIV)

    #else
        #define MAG_I2C_CLKDIV1_REG    (* (reg8 *) MAG_I2C_I2C_FF__CLK_DIV1)
        #define MAG_I2C_CLKDIV1_PTR    (  (reg8 *) MAG_I2C_I2C_FF__CLK_DIV1)

        #define MAG_I2C_CLKDIV2_REG    (* (reg8 *) MAG_I2C_I2C_FF__CLK_DIV2)
        #define MAG_I2C_CLKDIV2_PTR    (  (reg8 *) MAG_I2C_I2C_FF__CLK_DIV2)

    #endif /* (CY_PSOC5A) */

#else
    /* UDB implementation registers */
    #define MAG_I2C_CFG_REG    (* (reg8 *) \
                                           MAG_I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)
    #define MAG_I2C_CFG_PTR    (  (reg8 *) \
                                           MAG_I2C_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)

    #define MAG_I2C_CSR_REG        (* (reg8 *) MAG_I2C_bI2C_UDB_StsReg__STATUS_REG)
    #define MAG_I2C_CSR_PTR        (  (reg8 *) MAG_I2C_bI2C_UDB_StsReg__STATUS_REG)

    #define MAG_I2C_INT_MASK_REG   (* (reg8 *) MAG_I2C_bI2C_UDB_StsReg__MASK_REG)
    #define MAG_I2C_INT_MASK_PTR   (  (reg8 *) MAG_I2C_bI2C_UDB_StsReg__MASK_REG)

    #define MAG_I2C_INT_ENABLE_REG (* (reg8 *) MAG_I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)
    #define MAG_I2C_INT_ENABLE_PTR (  (reg8 *) MAG_I2C_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)

    #define MAG_I2C_DATA_REG       (* (reg8 *) MAG_I2C_bI2C_UDB_Shifter_u0__A0_REG)
    #define MAG_I2C_DATA_PTR       (  (reg8 *) MAG_I2C_bI2C_UDB_Shifter_u0__A0_REG)

    #define MAG_I2C_GO_REG         (* (reg8 *) MAG_I2C_bI2C_UDB_Shifter_u0__F1_REG)
    #define MAG_I2C_GO_PTR         (  (reg8 *) MAG_I2C_bI2C_UDB_Shifter_u0__F1_REG)

    #define MAG_I2C_MCLK_PRD_REG   (* (reg8 *) MAG_I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG)
    #define MAG_I2C_MCLK_PRD_PTR   (  (reg8 *) MAG_I2C_bI2C_UDB_Master_ClkGen_u0__D0_REG)

    #define MAG_I2C_MCLK_CMP_REG   (* (reg8 *) MAG_I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG)
    #define MAG_I2C_MCLK_CMP_PTR   (  (reg8 *) MAG_I2C_bI2C_UDB_Master_ClkGen_u0__D1_REG)

    #if(MAG_I2C_MODE_SLAVE_ENABLED)
        #define MAG_I2C_ADDR_REG       (* (reg8 *) MAG_I2C_bI2C_UDB_Shifter_u0__D0_REG)
        #define MAG_I2C_ADDR_PTR       (  (reg8 *) MAG_I2C_bI2C_UDB_Shifter_u0__D0_REG)

        #define MAG_I2C_PERIOD_REG     (* (reg8 *) MAG_I2C_bI2C_UDB_Slave_BitCounter__PERIOD_REG)
        #define MAG_I2C_PERIOD_PTR     (  (reg8 *) MAG_I2C_bI2C_UDB_Slave_BitCounter__PERIOD_REG)

        #define MAG_I2C_COUNTER_REG    (* (reg8 *) MAG_I2C_bI2C_UDB_Slave_BitCounter__COUNT_REG)
        #define MAG_I2C_COUNTER_PTR    (  (reg8 *) MAG_I2C_bI2C_UDB_Slave_BitCounter__COUNT_REG)

        #define MAG_I2C_COUNTER_AUX_CTL_REG  (* (reg8 *) \
                                                        MAG_I2C_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)
        #define MAG_I2C_COUNTER_AUX_CTL_PTR  (  (reg8 *) \
                                                        MAG_I2C_bI2C_UDB_Slave_BitCounter__CONTROL_AUX_CTL_REG)

    #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */

#endif /* (MAG_I2C_FF_IMPLEMENTED) */


/***************************************
*        Registers Constants
***************************************/

/* MAG_I2C_I2C_IRQ */
#define MAG_I2C_ISR_NUMBER     ((uint8) MAG_I2C_I2C_IRQ__INTC_NUMBER)
#define MAG_I2C_ISR_PRIORITY   ((uint8) MAG_I2C_I2C_IRQ__INTC_PRIOR_NUM)

/* I2C Slave Data Register */
#define MAG_I2C_SLAVE_ADDR_MASK    (0x7Fu)
#define MAG_I2C_SLAVE_ADDR_SHIFT   (0x01u)
#define MAG_I2C_DATA_MASK          (0xFFu)
#define MAG_I2C_READ_FLAG          (0x01u)

#define MAG_I2C_FF_RESET_DELAY     (0x02u)

#if(MAG_I2C_FF_IMPLEMENTED)
    /* XCFG I2C Extended Configuration Register */
    #define MAG_I2C_XCFG_CLK_EN        (0x80u) /* Enable gated clock to block */
    #define MAG_I2C_XCFG_I2C_ON        (0x40u) /* Enable I2C as wake up source*/
    #define MAG_I2C_XCFG_RDY_TO_SLEEP  (0x20u) /* I2C ready go to sleep */
    #define MAG_I2C_XCFG_FORCE_NACK    (0x10u) /* Force NACK all incomming transactions */
    #define MAG_I2C_XCFG_NO_BC_INT     (0x08u) /* No interrupt on byte complete */
    #define MAG_I2C_XCFG_BUF_MODE      (0x02u) /* Enable buffer mode */
    #define MAG_I2C_XCFG_HDWR_ADDR_EN  (0x01u) /* Enable Hardware address match */

    /* CFG I2C Configuration Register */
    #define MAG_I2C_CFG_SIO_SELECT     (0x80u) /* Pin Select for SCL/SDA lines */
    #define MAG_I2C_CFG_PSELECT        (0x40u) /* Pin Select */
    #define MAG_I2C_CFG_BUS_ERR_IE     (0x20u) /* Bus Error Interrupt Enable */
    #define MAG_I2C_CFG_STOP_IE        (0x10u) /* Enable Interrupt on STOP condition */
    #define MAG_I2C_CFG_CLK_RATE_MSK   (0x0Cu) /* Clock rate select  **CHECK**  */
    #define MAG_I2C_CFG_CLK_RATE_100   (0x00u) /* Clock rate select 100K */
    #define MAG_I2C_CFG_CLK_RATE_400   (0x04u) /* Clock rate select 400K */
    #define MAG_I2C_CFG_CLK_RATE_050   (0x08u) /* Clock rate select 50K  */
    #define MAG_I2C_CFG_CLK_RATE_RSVD  (0x0Cu) /* Clock rate select Invalid */
    #define MAG_I2C_CFG_EN_MSTR        (0x02u) /* Enable Master operation */
    #define MAG_I2C_CFG_EN_SLAVE       (0x01u) /* Enable Slave operation */

    #define MAG_I2C_CFG_CLK_RATE_LESS_EQUAL_50 (0x04u) /* Clock rate select <= 50kHz */
    #define MAG_I2C_CFG_CLK_RATE_GRATER_50     (0x00u) /* Clock rate select > 50kHz */

    /* CSR I2C Control and Status Register */
    #define MAG_I2C_CSR_BUS_ERROR      (0x80u) /* Active high when bus error has occured */
    #define MAG_I2C_CSR_LOST_ARB       (0x40u) /* Set to 1 if lost arbitration in host mode */
    #define MAG_I2C_CSR_STOP_STATUS    (0x20u) /* Set if Stop has been detected */
    #define MAG_I2C_CSR_ACK            (0x10u) /* ACK response */
    #define MAG_I2C_CSR_NAK            (0x00u) /* NAK response */
    #define MAG_I2C_CSR_ADDRESS        (0x08u) /* Set in firmware 0 = status bit, 1 Address is slave */
    #define MAG_I2C_CSR_TRANSMIT       (0x04u) /* Set in firmware 1 = transmit, 0 = receive */
    #define MAG_I2C_CSR_LRB            (0x02u) /* Last received bit */
    #define MAG_I2C_CSR_LRB_ACK        (0x00u) /* Last received bit was an ACK */
    #define MAG_I2C_CSR_LRB_NAK        (0x02u) /* Last received bit was an NAK */
    #define MAG_I2C_CSR_BYTE_COMPLETE  (0x01u) /* Informs that last byte has been sent */
    #define MAG_I2C_CSR_STOP_GEN       (0x00u) /* Generate a stop condition */
    #define MAG_I2C_CSR_RDY_TO_RD      (0x00u) /* Set to recieve mode */

    /* MCSR I2C Master Control and Status Register */
    #define MAG_I2C_MCSR_STOP_GEN      (0x10u) /* Firmware sets this bit to initiate a Stop condition */
    #define MAG_I2C_MCSR_BUS_BUSY      (0x08u) /* Status bit, Set at Start and cleared at Stop condition */
    #define MAG_I2C_MCSR_MSTR_MODE     (0x04u) /* Status bit, Set at Start and cleared at Stop condition */
    #define MAG_I2C_MCSR_RESTART_GEN   (0x02u) /* Firmware sets this bit to initiate a ReStart condition */
    #define MAG_I2C_MCSR_START_GEN     (0x01u) /* Firmware sets this bit to initiate a Start condition */

    /* CLK_DIV I2C Clock Divide Factor Register */
    #define MAG_I2C_CLK_DIV_MSK    (0x07u) /* Status bit, Set at Start and cleared at Stop condition */
    #define MAG_I2C_CLK_DIV_1      (0x00u) /* Divide input clock by  1 */
    #define MAG_I2C_CLK_DIV_2      (0x01u) /* Divide input clock by  2 */
    #define MAG_I2C_CLK_DIV_4      (0x02u) /* Divide input clock by  4 */
    #define MAG_I2C_CLK_DIV_8      (0x03u) /* Divide input clock by  8 */
    #define MAG_I2C_CLK_DIV_16     (0x04u) /* Divide input clock by 16 */
    #define MAG_I2C_CLK_DIV_32     (0x05u) /* Divide input clock by 32 */
    #define MAG_I2C_CLK_DIV_64     (0x06u) /* Divide input clock by 64 */

    /* PWRSYS_CR1 to handle Sleep */
    #define MAG_I2C_PWRSYS_CR1_I2C_REG_BACKUP  (0x04u) /* Enables, power to I2C regs while sleep */

#else
    /* CONTROL REG bits location */
    #define MAG_I2C_CTRL_START_SHIFT           (7u)
    #define MAG_I2C_CTRL_STOP_SHIFT            (6u)
    #define MAG_I2C_CTRL_RESTART_SHIFT         (5u)
    #define MAG_I2C_CTRL_NACK_SHIFT            (4u)
    #define MAG_I2C_CTRL_ANY_ADDRESS_SHIFT     (3u)
    #define MAG_I2C_CTRL_TRANSMIT_SHIFT        (2u)
    #define MAG_I2C_CTRL_ENABLE_MASTER_SHIFT   (1u)
    #define MAG_I2C_CTRL_ENABLE_SLAVE_SHIFT    (0u)
    #define MAG_I2C_CTRL_START_MASK            ((uint8) (0x01u << MAG_I2C_CTRL_START_SHIFT))
    #define MAG_I2C_CTRL_STOP_MASK             ((uint8) (0x01u << MAG_I2C_CTRL_STOP_SHIFT))
    #define MAG_I2C_CTRL_RESTART_MASK          ((uint8) (0x01u << MAG_I2C_CTRL_RESTART_SHIFT))
    #define MAG_I2C_CTRL_NACK_MASK             ((uint8) (0x01u << MAG_I2C_CTRL_NACK_SHIFT))
    #define MAG_I2C_CTRL_ANY_ADDRESS_MASK      ((uint8) (0x01u << MAG_I2C_CTRL_ANY_ADDRESS_SHIFT))
    #define MAG_I2C_CTRL_TRANSMIT_MASK         ((uint8) (0x01u << MAG_I2C_CTRL_TRANSMIT_SHIFT))
    #define MAG_I2C_CTRL_ENABLE_MASTER_MASK    ((uint8) (0x01u << MAG_I2C_CTRL_ENABLE_MASTER_SHIFT))
    #define MAG_I2C_CTRL_ENABLE_SLAVE_MASK     ((uint8) (0x01u << MAG_I2C_CTRL_ENABLE_SLAVE_SHIFT))

    /* STATUS REG bits location */
    #define MAG_I2C_STS_LOST_ARB_SHIFT         (6u)
    #define MAG_I2C_STS_STOP_SHIFT             (5u)
    #define MAG_I2C_STS_BUSY_SHIFT             (4u)
    #define MAG_I2C_STS_ADDR_SHIFT             (3u)
    #define MAG_I2C_STS_MASTER_MODE_SHIFT      (2u)
    #define MAG_I2C_STS_LRB_SHIFT              (1u)
    #define MAG_I2C_STS_BYTE_COMPLETE_SHIFT    (0u)
    #define MAG_I2C_STS_LOST_ARB_MASK          ((uint8) (0x01u << MAG_I2C_STS_LOST_ARB_SHIFT))
    #define MAG_I2C_STS_STOP_MASK              ((uint8) (0x01u << MAG_I2C_STS_STOP_SHIFT))
    #define MAG_I2C_STS_BUSY_MASK              ((uint8) (0x01u << MAG_I2C_STS_BUSY_SHIFT))
    #define MAG_I2C_STS_ADDR_MASK              ((uint8) (0x01u << MAG_I2C_STS_ADDR_SHIFT))
    #define MAG_I2C_STS_MASTER_MODE_MASK       ((uint8) (0x01u << MAG_I2C_STS_MASTER_MODE_SHIFT))
    #define MAG_I2C_STS_LRB_MASK               ((uint8) (0x01u << MAG_I2C_STS_LRB_SHIFT))
    #define MAG_I2C_STS_BYTE_COMPLETE_MASK     ((uint8) (0x01u << MAG_I2C_STS_BYTE_COMPLETE_SHIFT))

    /* AUX_CTL bits definition */
    #define MAG_I2C_COUNTER_ENABLE_MASK        (0x20u) /* Enable 7-bit counter     */
    #define MAG_I2C_INT_ENABLE_MASK            (0x10u) /* Enable intr from statusi */
    #define MAG_I2C_CNT7_ENABLE                (MAG_I2C_COUNTER_ENABLE_MASK)
    #define MAG_I2C_INTR_ENABLE                (MAG_I2C_INT_ENABLE_MASK)

#endif /* (MAG_I2C_FF_IMPLEMENTED) */


/***************************************
*        Marco
***************************************/

/* ACK and NACK for data and address checks */
#define MAG_I2C_CHECK_ADDR_ACK(csr)    ((MAG_I2C_CSR_LRB_ACK | MAG_I2C_CSR_ADDRESS) == \
                                                 ((MAG_I2C_CSR_LRB    | MAG_I2C_CSR_ADDRESS) &  \
                                                  (csr)))


#define MAG_I2C_CHECK_ADDR_NAK(csr)    ((MAG_I2C_CSR_LRB_NAK | MAG_I2C_CSR_ADDRESS) == \
                                                 ((MAG_I2C_CSR_LRB    | MAG_I2C_CSR_ADDRESS) &  \
                                                  (csr)))

#define MAG_I2C_CHECK_DATA_ACK(csr)    (0u == ((csr) & MAG_I2C_CSR_LRB_NAK))

/* MCSR conditions check */
#define MAG_I2C_CHECK_BUS_FREE(mcsr)       (0u == ((mcsr) & MAG_I2C_MCSR_BUS_BUSY))
#define MAG_I2C_CHECK_MASTER_MODE(mcsr)    (0u != ((mcsr) & MAG_I2C_MCSR_MSTR_MODE))

/* CSR conditions check */
#define MAG_I2C_WAIT_BYTE_COMPLETE(csr)    (0u == ((csr) & MAG_I2C_CSR_BYTE_COMPLETE))
#define MAG_I2C_WAIT_STOP_COMPLETE(csr)    (0u == ((csr) & (MAG_I2C_CSR_BYTE_COMPLETE | \
                                                                     MAG_I2C_CSR_STOP_STATUS)))
#define MAG_I2C_CHECK_BYTE_COMPLETE(csr)   (0u != ((csr) & MAG_I2C_CSR_BYTE_COMPLETE))
#define MAG_I2C_CHECK_STOP_STS(csr)        (0u != ((csr) & MAG_I2C_CSR_STOP_STATUS))
#define MAG_I2C_CHECK_LOST_ARB(csr)        (0u != ((csr) & MAG_I2C_CSR_LOST_ARB))
#define MAG_I2C_CHECK_ADDRESS_STS(csr)     (0u != ((csr) & MAG_I2C_CSR_ADDRESS))

/* Software start and end of transaction check */
#define MAG_I2C_CHECK_RESTART(mstrCtrl)    (0u != ((mstrCtrl) & MAG_I2C_MODE_REPEAT_START))
#define MAG_I2C_CHECK_NO_STOP(mstrCtrl)    (0u != ((mstrCtrl) & MAG_I2C_MODE_NO_STOP))

/* Send read or write completion depends on state */
#define MAG_I2C_GET_MSTAT_CMPLT ((0u != (MAG_I2C_state & MAG_I2C_SM_MSTR_RD)) ? \
                                                 (MAG_I2C_MSTAT_RD_CMPLT) : (MAG_I2C_MSTAT_WR_CMPLT))

/* Returns 7-bit slave address and used for software address match */
#define MAG_I2C_GET_SLAVE_ADDR(dataReg)   (((dataReg) >> MAG_I2C_SLAVE_ADDR_SHIFT) & \
                                                                  MAG_I2C_SLAVE_ADDR_MASK)

#if(MAG_I2C_FF_IMPLEMENTED)
    /* Check enable of module */
    #define MAG_I2C_I2C_ENABLE_REG     (MAG_I2C_ACT_PWRMGR_REG)
    #define MAG_I2C_IS_I2C_ENABLE(reg) (0u != ((reg) & MAG_I2C_ACT_PWR_EN))
    #define MAG_I2C_IS_ENABLED         (0u != (MAG_I2C_ACT_PWRMGR_REG & MAG_I2C_ACT_PWR_EN))

    #define MAG_I2C_CHECK_PWRSYS_I2C_BACKUP    (0u != (MAG_I2C_PWRSYS_CR1_I2C_REG_BACKUP & \
                                                                MAG_I2C_PWRSYS_CR1_REG))

    /* Check start condition generation */
    #define MAG_I2C_CHECK_START_GEN(mcsr)  ((0u != ((mcsr) & MAG_I2C_MCSR_START_GEN)) && \
                                                     (0u == ((mcsr) & MAG_I2C_MCSR_MSTR_MODE)))

    #define MAG_I2C_CLEAR_START_GEN        do{ \
                                                        MAG_I2C_MCSR_REG &=                                   \
                                                                           ((uint8) ~MAG_I2C_MCSR_START_GEN); \
                                                    }while(0)

    /* Stop interrupt */
    #define MAG_I2C_ENABLE_INT_ON_STOP     do{ \
                                                        MAG_I2C_CFG_REG |= MAG_I2C_CFG_STOP_IE; \
                                                    }while(0)

    #define MAG_I2C_DISABLE_INT_ON_STOP    do{ \
                                                        MAG_I2C_CFG_REG &=                                 \
                                                                           ((uint8) ~MAG_I2C_CFG_STOP_IE); \
                                                    }while(0)

    /* Transmit data */
    #define MAG_I2C_TRANSMIT_DATA          do{ \
                                                        MAG_I2C_CSR_REG = MAG_I2C_CSR_TRANSMIT; \
                                                    }while(0)

    #define MAG_I2C_ACK_AND_TRANSMIT       do{ \
                                                        MAG_I2C_CSR_REG = (MAG_I2C_CSR_ACK |      \
                                                                                    MAG_I2C_CSR_TRANSMIT); \
                                                    }while(0)

    #define MAG_I2C_NAK_AND_TRANSMIT       do{ \
                                                        MAG_I2C_CSR_REG = MAG_I2C_CSR_NAK; \
                                                    }while(0)

    /* Special case: udb needs to ack, ff needs to nak */
    #define MAG_I2C_ACKNAK_AND_TRANSMIT    do{ \
                                                        MAG_I2C_CSR_REG  = (MAG_I2C_CSR_NAK |      \
                                                                                     MAG_I2C_CSR_TRANSMIT); \
                                                    }while(0)
    /* Receive data */
    #define MAG_I2C_ACK_AND_RECEIVE        do{ \
                                                        MAG_I2C_CSR_REG = MAG_I2C_CSR_ACK; \
                                                    }while(0)

    #define MAG_I2C_NAK_AND_RECEIVE        do{ \
                                                        MAG_I2C_CSR_REG = MAG_I2C_CSR_NAK; \
                                                    }while(0)

    #define MAG_I2C_READY_TO_READ          do{ \
                                                        MAG_I2C_CSR_REG = MAG_I2C_CSR_RDY_TO_RD; \
                                                    }while(0)

    /* Master condition generation */
    #define MAG_I2C_GENERATE_START         do{ \
                                                        MAG_I2C_MCSR_REG = MAG_I2C_MCSR_START_GEN; \
                                                    }while(0)

    #if(CY_PSOC5A)
        #define MAG_I2C_GENERATE_RESTART \
                        do{ \
                            MAG_I2C_MCSR_REG = MAG_I2C_MCSR_RESTART_GEN; \
                            MAG_I2C_CSR_REG  = MAG_I2C_CSR_NAK;          \
                        }while(0)

        #define MAG_I2C_GENERATE_STOP      do{ \
                                                        MAG_I2C_CSR_REG = MAG_I2C_CSR_STOP_GEN; \
                                                    }while(0)

    #else   /* PSoC3 ES3 handlees zero lenght packets */
        #define MAG_I2C_GENERATE_RESTART \
                        do{ \
                            MAG_I2C_MCSR_REG = (MAG_I2C_MCSR_RESTART_GEN | \
                                                         MAG_I2C_MCSR_STOP_GEN);    \
                            MAG_I2C_CSR_REG  = MAG_I2C_CSR_TRANSMIT;       \
                        }while(0)

        #define MAG_I2C_GENERATE_STOP \
                        do{ \
                            MAG_I2C_MCSR_REG = MAG_I2C_MCSR_STOP_GEN; \
                            MAG_I2C_CSR_REG  = MAG_I2C_CSR_TRANSMIT;  \
                        }while(0)
    #endif /* (CY_PSOC5A) */

    /* Master manual APIs compatible defines */
    #define MAG_I2C_GENERATE_RESTART_MANUAL    MAG_I2C_GENERATE_RESTART
    #define MAG_I2C_GENERATE_STOP_MANUAL       MAG_I2C_GENERATE_STOP
    #define MAG_I2C_TRANSMIT_DATA_MANUAL       MAG_I2C_TRANSMIT_DATA
    #define MAG_I2C_READY_TO_READ_MANUAL       MAG_I2C_READY_TO_READ
    #define MAG_I2C_ACK_AND_RECEIVE_MANUAL     MAG_I2C_ACK_AND_RECEIVE

#else

    /* Masks to enalbe interrupts from Status register */
    #define MAG_I2C_STOP_IE_MASK           (MAG_I2C_STS_STOP_MASK)
    #define MAG_I2C_BYTE_COMPLETE_IE_MASK  (MAG_I2C_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: CSR gegisters definitions */
    #define MAG_I2C_CSR_LOST_ARB       (MAG_I2C_STS_LOST_ARB_MASK)
    #define MAG_I2C_CSR_STOP_STATUS    (MAG_I2C_STS_STOP_MASK)
    #define MAG_I2C_CSR_BUS_ERROR      (0x00u)
    #define MAG_I2C_CSR_ADDRESS        (MAG_I2C_STS_ADDR_MASK)
    #define MAG_I2C_CSR_TRANSMIT       (MAG_I2C_CTRL_TRANSMIT_MASK)
    #define MAG_I2C_CSR_LRB            (MAG_I2C_STS_LRB_MASK)
    #define MAG_I2C_CSR_LRB_NAK        (MAG_I2C_STS_LRB_MASK)
    #define MAG_I2C_CSR_LRB_ACK        (0x00u)
    #define MAG_I2C_CSR_BYTE_COMPLETE  (MAG_I2C_STS_BYTE_COMPLETE_MASK)

    /* FF compatibility: MCSR gegisters definitions */
    #define MAG_I2C_MCSR_REG           (MAG_I2C_CSR_REG)   /* UDB incoporates master and slave regs */
    #define MAG_I2C_MCSR_BUS_BUSY      (MAG_I2C_STS_BUSY_MASK)       /* Is bus is busy              */
    #define MAG_I2C_MCSR_START_GEN     (MAG_I2C_CTRL_START_MASK)     /* Generate Sart condition     */
    #define MAG_I2C_MCSR_RESTART_GEN   (MAG_I2C_CTRL_RESTART_MASK)   /* Generates RESTART condition */
    #define MAG_I2C_MCSR_MSTR_MODE     (MAG_I2C_STS_MASTER_MODE_MASK)/* Define if active Master     */

    /* Data to write into TX FIFO to release FSM */
    #define MAG_I2C_RELEASE_FSM         (0x00u)
    
    /* Check enable of module */
    #define MAG_I2C_I2C_ENABLE_REG     (MAG_I2C_CFG_REG)
    #define MAG_I2C_IS_I2C_ENABLE(reg) ((0u != ((reg) & MAG_I2C_ENABLE_MASTER)) || \
                                                 (0u != ((reg) & MAG_I2C_ENABLE_SLAVE)))

    #define MAG_I2C_IS_ENABLED         (0u != (MAG_I2C_CFG_REG & MAG_I2C_ENABLE_MS))

    /* Check start condition generation */
    #define MAG_I2C_CHECK_START_GEN(mcsr)  ((0u != (MAG_I2C_CFG_REG &        \
                                                             MAG_I2C_MCSR_START_GEN)) \
                                                    &&                                         \
                                                    (0u == ((mcsr) & MAG_I2C_MCSR_MSTR_MODE)))

    #define MAG_I2C_CLEAR_START_GEN        do{ \
                                                        MAG_I2C_CFG_REG &=                 \
                                                        ((uint8) ~MAG_I2C_MCSR_START_GEN); \
                                                    }while(0)


    /* Stop interrupt */
    #define MAG_I2C_ENABLE_INT_ON_STOP     do{ \
                                                       MAG_I2C_INT_MASK_REG |= MAG_I2C_STOP_IE_MASK; \
                                                    }while(0)

    #define MAG_I2C_DISABLE_INT_ON_STOP    do{ \
                                                        MAG_I2C_INT_MASK_REG &=                               \
                                                                             ((uint8) ~MAG_I2C_STOP_IE_MASK); \
                                                    }while(0)


    /* Transmit data */
    #define MAG_I2C_TRANSMIT_DATA      do{ \
                                                    MAG_I2C_CFG_REG = (MAG_I2C_CTRL_TRANSMIT_MASK | \
                                                                                MAG_I2C_CTRL_DEFAULT);       \
                                                    MAG_I2C_GO_REG  = MAG_I2C_RELEASE_FSM;          \
                                                }while(0)

    #define MAG_I2C_ACK_AND_TRANSMIT   MAG_I2C_TRANSMIT_DATA


    #define MAG_I2C_NAK_AND_TRANSMIT   do{ \
                                                    MAG_I2C_CFG_REG = (MAG_I2C_CTRL_NACK_MASK     | \
                                                                                MAG_I2C_CTRL_TRANSMIT_MASK | \
                                                                                MAG_I2C_CTRL_DEFAULT);       \
                                                    MAG_I2C_GO_REG  =  MAG_I2C_RELEASE_FSM;         \
                                                }while(0)

    /* Receive data */
    #define MAG_I2C_READY_TO_READ      do{ \
                                                    MAG_I2C_CFG_REG = MAG_I2C_CTRL_DEFAULT; \
                                                    MAG_I2C_GO_REG  =  MAG_I2C_RELEASE_FSM; \
                                                }while(0)

    #define MAG_I2C_ACK_AND_RECEIVE    MAG_I2C_READY_TO_READ

    #define MAG_I2C_NAK_AND_RECEIVE    do{ \
                                                    MAG_I2C_CFG_REG = (MAG_I2C_CTRL_NACK_MASK | \
                                                                                MAG_I2C_CTRL_DEFAULT);   \
                                                    MAG_I2C_GO_REG  =  MAG_I2C_RELEASE_FSM;     \
                                                }while(0)

    /* Master condition generation */
    #define MAG_I2C_GENERATE_START     do{ \
                                                    MAG_I2C_CFG_REG = (MAG_I2C_CTRL_START_MASK | \
                                                                                 MAG_I2C_CTRL_DEFAULT);   \
                                                    MAG_I2C_GO_REG  =  MAG_I2C_RELEASE_FSM;      \
                                                }while(0)

    #define MAG_I2C_GENERATE_RESTART   do{ \
                                                    MAG_I2C_CFG_REG = (MAG_I2C_CTRL_RESTART_MASK | \
                                                                                MAG_I2C_CTRL_NACK_MASK    | \
                                                                                MAG_I2C_CTRL_DEFAULT);      \
                                                    MAG_I2C_GO_REG  =  MAG_I2C_RELEASE_FSM;        \
                                                }while(0)


    #define MAG_I2C_GENERATE_STOP      do{ \
                                                    MAG_I2C_CFG_REG = (MAG_I2C_CTRL_NACK_MASK | \
                                                                                MAG_I2C_CTRL_STOP_MASK | \
                                                                                MAG_I2C_CTRL_DEFAULT);   \
                                                    MAG_I2C_GO_REG  =  MAG_I2C_RELEASE_FSM;     \
                                                }while(0)

    /* Master manual APIs compatible defines */
    /* These defines wait while byte complete is cleared after command issued */
    #define MAG_I2C_GENERATE_RESTART_MANUAL    \
                                        do{             \
                                            MAG_I2C_GENERATE_RESTART;                                    \
                                            while(MAG_I2C_CHECK_BYTE_COMPLETE(MAG_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define MAG_I2C_GENERATE_STOP_MANUAL   \
                                        do{         \
                                            MAG_I2C_GENERATE_STOP;                                       \
                                            while(MAG_I2C_CHECK_BYTE_COMPLETE(MAG_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define MAG_I2C_TRANSMIT_DATA_MANUAL   \
                                        do{         \
                                            MAG_I2C_TRANSMIT_DATA;                                       \
                                            while(MAG_I2C_CHECK_BYTE_COMPLETE(MAG_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define MAG_I2C_READY_TO_READ_MANUAL   \
                                        do{         \
                                            MAG_I2C_READY_TO_READ;      \
                                            while(MAG_I2C_CHECK_BYTE_COMPLETE(MAG_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)

    #define MAG_I2C_ACK_AND_RECEIVE_MANUAL \
                                        do{         \
                                            MAG_I2C_ACK_AND_RECEIVE;                                     \
                                            while(MAG_I2C_CHECK_BYTE_COMPLETE(MAG_I2C_CSR_REG)) \
                                            {                                                                     \
                                                ; /* Wait when byte complete is cleared */                        \
                                            }                                                                     \
                                        }while(0)
#endif /* (MAG_I2C_FF_IMPLEMENTED) */

/* Comon for FF and UDB: used to release bus after lost arb */
#define MAG_I2C_BUS_RELEASE    MAG_I2C_READY_TO_READ


/***************************************
*     Default register init constants
***************************************/

#define MAG_I2C_DISABLE    (0u)
#define MAG_I2C_ENABLE     (1u)

#if(MAG_I2C_FF_IMPLEMENTED)
    /* MAG_I2C_XCFG_REG: bits definition */
    #define MAG_I2C_DEFAULT_XCFG_HW_ADDR_EN ((MAG_I2C_HW_ADRR_DECODE) ? \
                                                        (MAG_I2C_XCFG_HDWR_ADDR_EN) : (0u))

    #define MAG_I2C_DEFAULT_XCFG_I2C_ON    ((MAG_I2C_WAKEUP_ENABLED) ? \
                                                        (MAG_I2C_XCFG_I2C_ON) : (0u))


    #define MAG_I2C_DEFAULT_CFG_SIO_SELECT ((MAG_I2C_I2C1_SIO_PAIR) ? \
                                                        (MAG_I2C_CFG_SIO_SELECT) : (0u))


    /* MAG_I2C_CFG_REG: bits definition */
    #define MAG_I2C_DEFAULT_CFG_PSELECT    ((MAG_I2C_WAKEUP_ENABLED) ? \
                                                        (MAG_I2C_CFG_PSELECT) : (0u))

    #define MAG_I2C_DEFAULT_CLK_RATE0  ((MAG_I2C_DATA_RATE <= 50u) ?        \
                                                    (MAG_I2C_CFG_CLK_RATE_050) :     \
                                                    ((MAG_I2C_DATA_RATE <= 100u) ?   \
                                                        (MAG_I2C_CFG_CLK_RATE_100) : \
                                                        (MAG_I2C_CFG_CLK_RATE_400)))

    #define MAG_I2C_DEFAULT_CLK_RATE1  ((MAG_I2C_DATA_RATE <= 50u) ?           \
                                                 (MAG_I2C_CFG_CLK_RATE_LESS_EQUAL_50) : \
                                                 (MAG_I2C_CFG_CLK_RATE_GRATER_50))

    #define MAG_I2C_DEFAULT_CLK_RATE   ((CY_PSOC5A) ? (MAG_I2C_DEFAULT_CLK_RATE0) : \
                                                               (MAG_I2C_DEFAULT_CLK_RATE1))


    #define MAG_I2C_ENABLE_MASTER      ((MAG_I2C_MODE_MASTER_ENABLED) ? \
                                                 (MAG_I2C_CFG_EN_MSTR) : (0u))

    #define MAG_I2C_ENABLE_SLAVE       ((MAG_I2C_MODE_SLAVE_ENABLED) ? \
                                                 (MAG_I2C_CFG_EN_SLAVE) : (0u))

    #define MAG_I2C_ENABLE_MS      (MAG_I2C_ENABLE_MASTER | MAG_I2C_ENABLE_SLAVE)


    /* MAG_I2C_DEFAULT_XCFG_REG */
    #define MAG_I2C_DEFAULT_XCFG   (MAG_I2C_XCFG_CLK_EN         | \
                                             MAG_I2C_DEFAULT_XCFG_I2C_ON | \
                                             MAG_I2C_DEFAULT_XCFG_HW_ADDR_EN)

    /* MAG_I2C_DEFAULT_CFG_REG */
    #define MAG_I2C_DEFAULT_CFG    (MAG_I2C_DEFAULT_CFG_SIO_SELECT | \
                                             MAG_I2C_DEFAULT_CFG_PSELECT    | \
                                             MAG_I2C_DEFAULT_CLK_RATE       | \
                                             MAG_I2C_ENABLE_MASTER          | \
                                             MAG_I2C_ENABLE_SLAVE)

    /*MAG_I2C_DEFAULT_DIVIDE_FACTOR_REG */
    #define MAG_I2C_DEFAULT_DIVIDE_FACTOR  ((CY_PSOC5A) ? ((uint8) 0u) : ((uint16) 1u))

#else
    /* MAG_I2C_CFG_REG: bits definition  */
    #define MAG_I2C_ENABLE_MASTER  ((MAG_I2C_MODE_MASTER_ENABLED) ? \
                                             (MAG_I2C_CTRL_ENABLE_MASTER_MASK) : (0u))

    #define MAG_I2C_ENABLE_SLAVE   ((MAG_I2C_MODE_SLAVE_ENABLED) ? \
                                             (MAG_I2C_CTRL_ENABLE_SLAVE_MASK) : (0u))

    #define MAG_I2C_ENABLE_MS      (MAG_I2C_ENABLE_MASTER | MAG_I2C_ENABLE_SLAVE)


    #define MAG_I2C_DEFAULT_CTRL_ANY_ADDR   ((MAG_I2C_HW_ADRR_DECODE) ? \
                                                      (0u) : (MAG_I2C_CTRL_ANY_ADDRESS_MASK))

    /* MAG_I2C_DEFAULT_CFG_REG */
    #define MAG_I2C_DEFAULT_CFG    (MAG_I2C_DEFAULT_CTRL_ANY_ADDR)

    /* All CTRL default bits to be used in macro */
    #define MAG_I2C_CTRL_DEFAULT   (MAG_I2C_DEFAULT_CTRL_ANY_ADDR | MAG_I2C_ENABLE_MS)

    /* Master clock generator: d0 and d1 */
    #define MAG_I2C_MCLK_PERIOD_VALUE  (0x0Fu)
    #define MAG_I2C_MCLK_COMPARE_VALUE (0x08u)

    /* Slave bit-counter: contorol period */
    #define MAG_I2C_PERIOD_VALUE       (0x07u)

    /* MAG_I2C_DEFAULT_INT_MASK */
    #define MAG_I2C_DEFAULT_INT_MASK   (MAG_I2C_BYTE_COMPLETE_IE_MASK)

    /* MAG_I2C_DEFAULT_MCLK_PRD_REG */
    #define MAG_I2C_DEFAULT_MCLK_PRD   (MAG_I2C_MCLK_PERIOD_VALUE)

    /* MAG_I2C_DEFAULT_MCLK_CMP_REG */
    #define MAG_I2C_DEFAULT_MCLK_CMP   (MAG_I2C_MCLK_COMPARE_VALUE)

    /* MAG_I2C_DEFAULT_PERIOD_REG */
    #define MAG_I2C_DEFAULT_PERIOD     (MAG_I2C_PERIOD_VALUE)

#endif /* (MAG_I2C_FF_IMPLEMENTED) */


/***************************************
*       Obsolete
***************************************/

/* Following code are OBSOLETE and must not be used 
 * starting from I2C 3.20
 */
 
#define MAG_I2C_SSTAT_RD_ERR       (0x08u)
#define MAG_I2C_SSTAT_WR_ERR       (0x80u)
#define MAG_I2C_MSTR_SLAVE_BUSY    (MAG_I2C_MSTR_NOT_READY)
#define MAG_I2C_MSTAT_ERR_BUF_OVFL (0x80u)
#define MAG_I2C_SSTAT_RD_CMPT      (MAG_I2C_SSTAT_RD_CMPLT)
#define MAG_I2C_SSTAT_WR_CMPT      (MAG_I2C_SSTAT_WR_CMPLT)
#define MAG_I2C_MODE_MULTI_MASTER_ENABLE    (MAG_I2C_MODE_MULTI_MASTER_MASK)
#define MAG_I2C_DATA_RATE_50       (50u)
#define MAG_I2C_DATA_RATE_100      (100u)
#define MAG_I2C_DEV_MASK           (0xF0u)
#define MAG_I2C_SM_SL_STOP         (0x14u)
#define MAG_I2C_SM_MASTER_IDLE     (0x40u)
#define MAG_I2C_HDWR_DECODE        (0x01u)

#endif /* CY_I2C_MAG_I2C_H */


/* [] END OF FILE */
