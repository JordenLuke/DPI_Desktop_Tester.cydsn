/*******************************************************************************
* File Name: HMC5883L_I2C_INT.c
* Version 3.30
*
* Description:
*  This file provides the source code of Interrupt Service Routine (ISR)
*  for I2C component.
*
*  Note:
*
********************************************************************************
* Copyright 2008-2012, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "HMC5883L_I2C_PVT.h"


/*******************************************************************************
*  Place your includes, defines and code here
********************************************************************************/
/* `#START HMC5883L_I2C_ISR_intc` */

/* `#END` */


/*******************************************************************************
* Function Name: HMC5883L_I2C_ISR
********************************************************************************
*
* Summary:
*  Handler for I2C interrupt. The Slave and Master operations are handled here.
*
* Parameters:
*  void
*
* Return:
*  void
*
* Reentrant:
*  No
*
*******************************************************************************/
CY_ISR(HMC5883L_I2C_ISR)
{
    #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
       uint8  tmp8;
    #endif  /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */

    uint8  tmpCsr;

    #if(HMC5883L_I2C_TIMEOUT_FF_ENABLED)
        if(0u != HMC5883L_I2C_TimeoutGetStatus())
        {
            HMC5883L_I2C_TimeoutReset();
            HMC5883L_I2C_state = HMC5883L_I2C_SM_EXIT_IDLE;
            /* HMC5883L_I2C_CSR_REG should be cleared after reset */
        }
    #endif /* (HMC5883L_I2C_TIMEOUT_FF_ENABLED) */


    tmpCsr = HMC5883L_I2C_CSR_REG;      /* Make copy as interrupts clear */

    #if(HMC5883L_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED)
        if(HMC5883L_I2C_CHECK_START_GEN(HMC5883L_I2C_MCSR_REG))
        {
            HMC5883L_I2C_CLEAR_START_GEN;

            /* Set READ complete, but was aborted */
            HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_ERR_XFER |
                                            HMC5883L_I2C_GET_MSTAT_CMPLT);

            /* The slave was addressed */
            HMC5883L_I2C_state = HMC5883L_I2C_SM_SLAVE;
        }
    #endif /* (HMC5883L_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED) */


    #if(HMC5883L_I2C_MODE_MULTI_MASTER_ENABLED)
        if(HMC5883L_I2C_CHECK_LOST_ARB(tmpCsr))
        {
            /* Set errors */
            HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_ERR_XFER     |
                                            HMC5883L_I2C_MSTAT_ERR_ARB_LOST |
                                            HMC5883L_I2C_GET_MSTAT_CMPLT);

            HMC5883L_I2C_DISABLE_INT_ON_STOP; /* Interrupt on Stop is enabled by write */

            #if(HMC5883L_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED)
                if(HMC5883L_I2C_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* The slave was addressed */
                    HMC5883L_I2C_state = HMC5883L_I2C_SM_SLAVE;
                }
                else
                {
                    HMC5883L_I2C_BUS_RELEASE;

                    HMC5883L_I2C_state = HMC5883L_I2C_SM_EXIT_IDLE;
                }
            #else
                HMC5883L_I2C_BUS_RELEASE;

                HMC5883L_I2C_state = HMC5883L_I2C_SM_EXIT_IDLE;

            #endif /* (HMC5883L_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED) */
        }
    #endif /* (HMC5883L_I2C_MODE_MULTI_MASTER_ENABLED) */

    /* Check for Master operation mode */
    if(HMC5883L_I2C_CHECK_SM_MASTER)
    {
        #if(HMC5883L_I2C_MODE_MASTER_ENABLED)
            if(HMC5883L_I2C_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                switch (HMC5883L_I2C_state)
                {
                case HMC5883L_I2C_SM_MSTR_WR_ADDR:  /* After address is sent, WRITE data */
                case HMC5883L_I2C_SM_MSTR_RD_ADDR:  /* After address is sent, READ  data */

                    tmpCsr &= ((uint8) ~HMC5883L_I2C_CSR_STOP_STATUS); /* Clear STOP bit history on address phase */
                    
                    if(HMC5883L_I2C_CHECK_ADDR_ACK(tmpCsr))
                    {
                        /* Setup for transmit or receive of data */
                        if(HMC5883L_I2C_state == HMC5883L_I2C_SM_MSTR_WR_ADDR)   /* TRANSMIT data */
                        {
                            /* Check if at least one byte to transfer */
                            if(HMC5883L_I2C_mstrWrBufSize > 0u)
                            {
                                /* Load the 1st data byte */
                                HMC5883L_I2C_DATA_REG = HMC5883L_I2C_mstrWrBufPtr[0u];
                                HMC5883L_I2C_TRANSMIT_DATA;
                                HMC5883L_I2C_mstrWrBufIndex = 1u;   /* Set index to 2nd element */

                                /* Set transmit state until done */
                                HMC5883L_I2C_state = HMC5883L_I2C_SM_MSTR_WR_DATA;
                            }
                            /* End of buffer: complete writing */
                            else if(HMC5883L_I2C_CHECK_NO_STOP(HMC5883L_I2C_mstrControl))
                            {
                                #if(CY_PSOC5A)
                                    /* Do not handles 0 bytes transfer - HALT is NOT allowed */
                                    HMC5883L_I2C_ENABLE_INT_ON_STOP;
                                    HMC5883L_I2C_GENERATE_STOP;
                                
                                #else
                                    /* Set WRITE complete and Master HALTED */
                                    HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_XFER_HALT |
                                                                    HMC5883L_I2C_MSTAT_WR_CMPLT);

                                    HMC5883L_I2C_state = HMC5883L_I2C_SM_MSTR_HALT; /* Expect RESTART */
                                    HMC5883L_I2C_DisableInt();
                                
                                #endif /* (CY_PSOC5A) */
                            }
                            else
                            {
                                HMC5883L_I2C_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                                HMC5883L_I2C_GENERATE_STOP;
                            }
                        }
                        else  /* Master Receive data */
                        {
                            HMC5883L_I2C_READY_TO_READ; /* Release bus to read data */

                            HMC5883L_I2C_state  = HMC5883L_I2C_SM_MSTR_RD_DATA;
                        }
                    }
                    /* Address is NACKed */
                    else if(HMC5883L_I2C_CHECK_ADDR_NAK(tmpCsr))
                    {
                        /* Set Address NAK error */
                        HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_ERR_XFER |
                                                        HMC5883L_I2C_MSTAT_ERR_ADDR_NAK);
                                                        
                        if(HMC5883L_I2C_CHECK_NO_STOP(HMC5883L_I2C_mstrControl))
                        {
                            HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_XFER_HALT | 
                                                            HMC5883L_I2C_GET_MSTAT_CMPLT);

                            HMC5883L_I2C_state = HMC5883L_I2C_SM_MSTR_HALT; /* Expect RESTART */
                            HMC5883L_I2C_DisableInt();
                        }
                        else  /* Do normal Stop */
                        {
                            HMC5883L_I2C_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                            HMC5883L_I2C_GENERATE_STOP;
                        }
                    }
                    else
                    {
                        /* Address phase is not set for some reason: error */
                        #if(HMC5883L_I2C_TIMEOUT_ENABLED)
                            /* Exit from interrupt to take a chance for timeout timer handle this case */
                            HMC5883L_I2C_DisableInt();
                            HMC5883L_I2C_ClearPendingInt();
                        #else
                            /* Block execution flow: unexpected condition */
                            CYASSERT(0u != 0u);
                        #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */
                    }
                    break;

                case HMC5883L_I2C_SM_MSTR_WR_DATA:

                    if(HMC5883L_I2C_CHECK_DATA_ACK(tmpCsr))
                    {
                        /* Check if end of buffer */
                        if(HMC5883L_I2C_mstrWrBufIndex  < HMC5883L_I2C_mstrWrBufSize)
                        {
                            HMC5883L_I2C_DATA_REG =
                                                     HMC5883L_I2C_mstrWrBufPtr[HMC5883L_I2C_mstrWrBufIndex];
                            HMC5883L_I2C_TRANSMIT_DATA;
                            HMC5883L_I2C_mstrWrBufIndex++;
                        }
                        /* End of buffer: complete writing */
                        else if(HMC5883L_I2C_CHECK_NO_STOP(HMC5883L_I2C_mstrControl))
                        {
                            /* Set WRITE complete and Master HALTED */
                            HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_XFER_HALT |
                                                            HMC5883L_I2C_MSTAT_WR_CMPLT);

                            HMC5883L_I2C_state = HMC5883L_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                            HMC5883L_I2C_DisableInt();
                        }
                        else  /* Do normal STOP */
                        {
                            HMC5883L_I2C_Workaround();          /* Workaround: empty function */
                            HMC5883L_I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                            HMC5883L_I2C_GENERATE_STOP;
                        }
                    }
                    /* Last byte NAKed: end writing */
                    else if(HMC5883L_I2C_CHECK_NO_STOP(HMC5883L_I2C_mstrControl))
                    {
                        /* Set WRITE complete, SHORT transfer and Master HALTED */
                        HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_ERR_XFER       |
                                                        HMC5883L_I2C_MSTAT_ERR_SHORT_XFER |
                                                        HMC5883L_I2C_MSTAT_XFER_HALT      |
                                                        HMC5883L_I2C_MSTAT_WR_CMPLT);

                        HMC5883L_I2C_state = HMC5883L_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                        HMC5883L_I2C_DisableInt();
                    }
                    else  /* Do normal STOP */
                    {
                        HMC5883L_I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                        HMC5883L_I2C_GENERATE_STOP;

                        /* Set SHORT and ERR transfer */
                        HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_ERR_SHORT_XFER |
                                                        HMC5883L_I2C_MSTAT_ERR_XFER);
                    }
                    
                    break;

                case HMC5883L_I2C_SM_MSTR_RD_DATA:

                    HMC5883L_I2C_mstrRdBufPtr[HMC5883L_I2C_mstrRdBufIndex] = HMC5883L_I2C_DATA_REG;
                    HMC5883L_I2C_mstrRdBufIndex++;

                    /* Check if end of buffer */
                    if(HMC5883L_I2C_mstrRdBufIndex < HMC5883L_I2C_mstrRdBufSize)
                    {
                        HMC5883L_I2C_ACK_AND_RECEIVE;       /* ACK and receive byte */
                    }
                    /* End of buffer: complete reading */
                    else if(HMC5883L_I2C_CHECK_NO_STOP(HMC5883L_I2C_mstrControl))
                    {                        
                        /* Set READ complete and Master HALTED */
                        HMC5883L_I2C_mstrStatus |= (HMC5883L_I2C_MSTAT_XFER_HALT |
                                                        HMC5883L_I2C_MSTAT_RD_CMPLT);
                        
                        HMC5883L_I2C_state = HMC5883L_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                        HMC5883L_I2C_DisableInt();
                    }
                    else
                    {
                        HMC5883L_I2C_ENABLE_INT_ON_STOP;
                        HMC5883L_I2C_NAK_AND_RECEIVE;       /* NACK and TRY to generate STOP */
                    }
                    break;

                default: /* This is an invalid state and should not occur */

                    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        HMC5883L_I2C_DisableInt();
                        HMC5883L_I2C_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */

                    break;
                }
            }

            /* Catches the Stop: end of transaction */
            if(HMC5883L_I2C_CHECK_STOP_STS(tmpCsr))
            {
                HMC5883L_I2C_mstrStatus |= HMC5883L_I2C_GET_MSTAT_CMPLT;

                HMC5883L_I2C_DISABLE_INT_ON_STOP;
                HMC5883L_I2C_state = HMC5883L_I2C_SM_IDLE;
            }
        #endif /* (HMC5883L_I2C_MODE_MASTER_ENABLED) */
    }
    else if(HMC5883L_I2C_CHECK_SM_SLAVE)
    {
        #if(HMC5883L_I2C_MODE_SLAVE_ENABLED)
            
            if((HMC5883L_I2C_CHECK_STOP_STS(tmpCsr)) || /* Stop || Restart */
               (HMC5883L_I2C_CHECK_BYTE_COMPLETE(tmpCsr) && HMC5883L_I2C_CHECK_ADDRESS_STS(tmpCsr)))
            {
                /* Catch end of master write transcation: use interrupt on Stop */
                /* The STOP bit history on address phase does not have correct state */
                if(HMC5883L_I2C_SM_SL_WR_DATA == HMC5883L_I2C_state)
                {
                    HMC5883L_I2C_DISABLE_INT_ON_STOP;

                    HMC5883L_I2C_slStatus &= ((uint8) ~HMC5883L_I2C_SSTAT_WR_BUSY);
                    HMC5883L_I2C_slStatus |= ((uint8)  HMC5883L_I2C_SSTAT_WR_CMPLT);

                    HMC5883L_I2C_state = HMC5883L_I2C_SM_IDLE;
                }
            }

            if(HMC5883L_I2C_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                /* The address only issued after Start or ReStart: so check address
                   to catch this events:
                    FF : sets Addr phase with byte_complete interrupt trigger.
                    UDB: sets Addr phase immediately after Start or ReStart. */
                if(HMC5883L_I2C_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* Check for software address detection */
                    #if(HMC5883L_I2C_SW_ADRR_DECODE)
                        tmp8 = HMC5883L_I2C_GET_SLAVE_ADDR(HMC5883L_I2C_DATA_REG);

                        if(tmp8 == HMC5883L_I2C_slAddress)   /* Check for address match */
                        {
                            if(0u != (HMC5883L_I2C_DATA_REG & HMC5883L_I2C_READ_FLAG))
                            {
                                /* Place code to prepare read buffer here                  */
                                /* `#START HMC5883L_I2C_SW_PREPARE_READ_BUF_interrupt` */

                                /* `#END` */

                                /* Prepare next opeation to read, get data and place in data register */
                                if(HMC5883L_I2C_slRdBufIndex < HMC5883L_I2C_slRdBufSize)
                                {
                                    /* Load first data byte from array */
                                    HMC5883L_I2C_DATA_REG = HMC5883L_I2C_slRdBufPtr[HMC5883L_I2C_slRdBufIndex];
                                    HMC5883L_I2C_ACK_AND_TRANSMIT;
                                    HMC5883L_I2C_slRdBufIndex++;

                                    HMC5883L_I2C_slStatus |= HMC5883L_I2C_SSTAT_RD_BUSY;
                                }
                                else    /* Overflow: provide 0xFF on the bus */
                                {
                                    HMC5883L_I2C_DATA_REG = HMC5883L_I2C_OVERFLOW_RETURN;
                                    HMC5883L_I2C_ACK_AND_TRANSMIT;

                                    HMC5883L_I2C_slStatus  |= (HMC5883L_I2C_SSTAT_RD_BUSY |
                                                                   HMC5883L_I2C_SSTAT_RD_ERR_OVFL);
                                }

                                HMC5883L_I2C_state = HMC5883L_I2C_SM_SL_RD_DATA;
                            }
                            else  /* Write transaction: receive 1st byte */
                            {
                                HMC5883L_I2C_ACK_AND_RECEIVE;
                                HMC5883L_I2C_state = HMC5883L_I2C_SM_SL_WR_DATA;

                                HMC5883L_I2C_slStatus |= HMC5883L_I2C_SSTAT_WR_BUSY;
                                HMC5883L_I2C_ENABLE_INT_ON_STOP;
                            }
                        }    
                        else
                        {
                            /*     Place code to compare for additional address here    */
                            /* `#START HMC5883L_I2C_SW_ADDR_COMPARE_interruptStart` */

                            /* `#END` */
                            
                            HMC5883L_I2C_NAK_AND_RECEIVE;   /* NACK address */

                            /* Place code to end of condition for NACK generation here */
                            /* `#START HMC5883L_I2C_SW_ADDR_COMPARE_interruptEnd`  */

                            /* `#END` */
                        }
                        
                    #else /* (HMC5883L_I2C_HW_ADRR_DECODE) */
                        
                        if(0u != (HMC5883L_I2C_DATA_REG & HMC5883L_I2C_READ_FLAG))
                        {
                            /* Place code to prepare read buffer here                  */
                            /* `#START HMC5883L_I2C_HW_PREPARE_READ_BUF_interrupt` */

                            /* `#END` */

                            /* Prepare next opeation to read, get data and place in data register */
                            if(HMC5883L_I2C_slRdBufIndex < HMC5883L_I2C_slRdBufSize)
                            {
                                /* Load first data byte from array */
                                HMC5883L_I2C_DATA_REG = HMC5883L_I2C_slRdBufPtr[HMC5883L_I2C_slRdBufIndex];
                                HMC5883L_I2C_ACK_AND_TRANSMIT;
                                HMC5883L_I2C_slRdBufIndex++;

                                HMC5883L_I2C_slStatus |= HMC5883L_I2C_SSTAT_RD_BUSY;
                            }
                            else    /* Overflow: provide 0xFF on the bus */
                            {
                                HMC5883L_I2C_DATA_REG = HMC5883L_I2C_OVERFLOW_RETURN;
                                HMC5883L_I2C_ACK_AND_TRANSMIT;

                                HMC5883L_I2C_slStatus  |= (HMC5883L_I2C_SSTAT_RD_BUSY |
                                                               HMC5883L_I2C_SSTAT_RD_ERR_OVFL);
                            }

                            HMC5883L_I2C_state = HMC5883L_I2C_SM_SL_RD_DATA;
                        }
                        else  /* Write transaction: receive 1st byte */
                        {
                            HMC5883L_I2C_ACK_AND_RECEIVE;
                            HMC5883L_I2C_state = HMC5883L_I2C_SM_SL_WR_DATA;

                            HMC5883L_I2C_slStatus |= HMC5883L_I2C_SSTAT_WR_BUSY;
                            HMC5883L_I2C_ENABLE_INT_ON_STOP;
                        }
                        
                    #endif /* (HMC5883L_I2C_SW_ADRR_DECODE) */
                }
                /* Data states */
                /* Data master writes into slave */
                else if(HMC5883L_I2C_state == HMC5883L_I2C_SM_SL_WR_DATA)
                {
                    if(HMC5883L_I2C_slWrBufIndex < HMC5883L_I2C_slWrBufSize)
                    {
                        tmp8 = HMC5883L_I2C_DATA_REG;
                        HMC5883L_I2C_ACK_AND_RECEIVE;
                        HMC5883L_I2C_slWrBufPtr[HMC5883L_I2C_slWrBufIndex] = tmp8;
                        HMC5883L_I2C_slWrBufIndex++;
                    }
                    else  /* of array: complete write, send NACK */
                    {
                        HMC5883L_I2C_NAK_AND_RECEIVE;

                        HMC5883L_I2C_slStatus |= HMC5883L_I2C_SSTAT_WR_ERR_OVFL;
                    }
                }
                /* Data master reads from slave */
                else if(HMC5883L_I2C_state == HMC5883L_I2C_SM_SL_RD_DATA)
                {
                    if(HMC5883L_I2C_CHECK_DATA_ACK(tmpCsr))
                    {
                        if(HMC5883L_I2C_slRdBufIndex < HMC5883L_I2C_slRdBufSize)
                        {
                             /* Get data from array */
                            HMC5883L_I2C_DATA_REG = HMC5883L_I2C_slRdBufPtr[HMC5883L_I2C_slRdBufIndex];
                            HMC5883L_I2C_TRANSMIT_DATA;
                            HMC5883L_I2C_slRdBufIndex++;
                        }
                        else   /* Overflow: provide 0xFF on the bus */
                        {
                            HMC5883L_I2C_DATA_REG = HMC5883L_I2C_OVERFLOW_RETURN;
                            HMC5883L_I2C_TRANSMIT_DATA;

                            HMC5883L_I2C_slStatus |= HMC5883L_I2C_SSTAT_RD_ERR_OVFL;
                        }
                    }
                    else  /* Last byte was NACKed: read complete */
                    {
                        /* Only NACK appears on the bus */
                        HMC5883L_I2C_DATA_REG = HMC5883L_I2C_OVERFLOW_RETURN;
                        HMC5883L_I2C_NAK_AND_TRANSMIT;

                        HMC5883L_I2C_slStatus &= ((uint8) ~HMC5883L_I2C_SSTAT_RD_BUSY);
                        HMC5883L_I2C_slStatus |= ((uint8)  HMC5883L_I2C_SSTAT_RD_CMPLT);

                        HMC5883L_I2C_state = HMC5883L_I2C_SM_IDLE;
                    }
                }
                else
                {
                    #if(HMC5883L_I2C_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        HMC5883L_I2C_DisableInt();
                        HMC5883L_I2C_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (HMC5883L_I2C_TIMEOUT_ENABLED) */
                }
            }
        #endif /* (HMC5883L_I2C_MODE_SLAVE_ENABLED) */
    }
    else
    {
        /* The FSM skips master and slave processing: return to IDLE */
        HMC5883L_I2C_state = HMC5883L_I2C_SM_IDLE;
    }
}


#if((HMC5883L_I2C_FF_IMPLEMENTED) && (HMC5883L_I2C_WAKEUP_ENABLED))
    /*******************************************************************************
    * Function Name: HMC5883L_I2C_WAKEUP_ISR
    ********************************************************************************
    *
    * Summary:
    *  Empty interrupt handler to trigger after wakeup.
    *
    * Parameters:
    *  void
    *
    * Return:
    *  void
    *
    *******************************************************************************/
    CY_ISR(HMC5883L_I2C_WAKEUP_ISR)
    {
        HMC5883L_I2C_wakeupSource = 1u;  /* I2C was wakeup source */
        /* The SCL is stretched unitl the I2C_Wake() is called */
    }
#endif /* ((HMC5883L_I2C_FF_IMPLEMENTED) && (HMC5883L_I2C_WAKEUP_ENABLED))*/


/* [] END OF FILE */
