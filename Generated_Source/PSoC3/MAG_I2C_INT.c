/*******************************************************************************
* File Name: MAG_I2C_INT.c
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

#include "MAG_I2C_PVT.h"


/*******************************************************************************
*  Place your includes, defines and code here
********************************************************************************/
/* `#START MAG_I2C_ISR_intc` */

/* `#END` */


/*******************************************************************************
* Function Name: MAG_I2C_ISR
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
CY_ISR(MAG_I2C_ISR)
{
    #if(MAG_I2C_MODE_SLAVE_ENABLED)
       uint8  tmp8;
    #endif  /* (MAG_I2C_MODE_SLAVE_ENABLED) */

    uint8  tmpCsr;

    #if(MAG_I2C_TIMEOUT_FF_ENABLED)
        if(0u != MAG_I2C_TimeoutGetStatus())
        {
            MAG_I2C_TimeoutReset();
            MAG_I2C_state = MAG_I2C_SM_EXIT_IDLE;
            /* MAG_I2C_CSR_REG should be cleared after reset */
        }
    #endif /* (MAG_I2C_TIMEOUT_FF_ENABLED) */


    tmpCsr = MAG_I2C_CSR_REG;      /* Make copy as interrupts clear */

    #if(MAG_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED)
        if(MAG_I2C_CHECK_START_GEN(MAG_I2C_MCSR_REG))
        {
            MAG_I2C_CLEAR_START_GEN;

            /* Set READ complete, but was aborted */
            MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_ERR_XFER |
                                            MAG_I2C_GET_MSTAT_CMPLT);

            /* The slave was addressed */
            MAG_I2C_state = MAG_I2C_SM_SLAVE;
        }
    #endif /* (MAG_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED) */


    #if(MAG_I2C_MODE_MULTI_MASTER_ENABLED)
        if(MAG_I2C_CHECK_LOST_ARB(tmpCsr))
        {
            /* Set errors */
            MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_ERR_XFER     |
                                            MAG_I2C_MSTAT_ERR_ARB_LOST |
                                            MAG_I2C_GET_MSTAT_CMPLT);

            MAG_I2C_DISABLE_INT_ON_STOP; /* Interrupt on Stop is enabled by write */

            #if(MAG_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED)
                if(MAG_I2C_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* The slave was addressed */
                    MAG_I2C_state = MAG_I2C_SM_SLAVE;
                }
                else
                {
                    MAG_I2C_BUS_RELEASE;

                    MAG_I2C_state = MAG_I2C_SM_EXIT_IDLE;
                }
            #else
                MAG_I2C_BUS_RELEASE;

                MAG_I2C_state = MAG_I2C_SM_EXIT_IDLE;

            #endif /* (MAG_I2C_MODE_MULTI_MASTER_SLAVE_ENABLED) */
        }
    #endif /* (MAG_I2C_MODE_MULTI_MASTER_ENABLED) */

    /* Check for Master operation mode */
    if(MAG_I2C_CHECK_SM_MASTER)
    {
        #if(MAG_I2C_MODE_MASTER_ENABLED)
            if(MAG_I2C_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                switch (MAG_I2C_state)
                {
                case MAG_I2C_SM_MSTR_WR_ADDR:  /* After address is sent, WRITE data */
                case MAG_I2C_SM_MSTR_RD_ADDR:  /* After address is sent, READ  data */

                    tmpCsr &= ((uint8) ~MAG_I2C_CSR_STOP_STATUS); /* Clear STOP bit history on address phase */
                    
                    if(MAG_I2C_CHECK_ADDR_ACK(tmpCsr))
                    {
                        /* Setup for transmit or receive of data */
                        if(MAG_I2C_state == MAG_I2C_SM_MSTR_WR_ADDR)   /* TRANSMIT data */
                        {
                            /* Check if at least one byte to transfer */
                            if(MAG_I2C_mstrWrBufSize > 0u)
                            {
                                /* Load the 1st data byte */
                                MAG_I2C_DATA_REG = MAG_I2C_mstrWrBufPtr[0u];
                                MAG_I2C_TRANSMIT_DATA;
                                MAG_I2C_mstrWrBufIndex = 1u;   /* Set index to 2nd element */

                                /* Set transmit state until done */
                                MAG_I2C_state = MAG_I2C_SM_MSTR_WR_DATA;
                            }
                            /* End of buffer: complete writing */
                            else if(MAG_I2C_CHECK_NO_STOP(MAG_I2C_mstrControl))
                            {
                                #if(CY_PSOC5A)
                                    /* Do not handles 0 bytes transfer - HALT is NOT allowed */
                                    MAG_I2C_ENABLE_INT_ON_STOP;
                                    MAG_I2C_GENERATE_STOP;
                                
                                #else
                                    /* Set WRITE complete and Master HALTED */
                                    MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_XFER_HALT |
                                                                    MAG_I2C_MSTAT_WR_CMPLT);

                                    MAG_I2C_state = MAG_I2C_SM_MSTR_HALT; /* Expect RESTART */
                                    MAG_I2C_DisableInt();
                                
                                #endif /* (CY_PSOC5A) */
                            }
                            else
                            {
                                MAG_I2C_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                                MAG_I2C_GENERATE_STOP;
                            }
                        }
                        else  /* Master Receive data */
                        {
                            MAG_I2C_READY_TO_READ; /* Release bus to read data */

                            MAG_I2C_state  = MAG_I2C_SM_MSTR_RD_DATA;
                        }
                    }
                    /* Address is NACKed */
                    else if(MAG_I2C_CHECK_ADDR_NAK(tmpCsr))
                    {
                        /* Set Address NAK error */
                        MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_ERR_XFER |
                                                        MAG_I2C_MSTAT_ERR_ADDR_NAK);
                                                        
                        if(MAG_I2C_CHECK_NO_STOP(MAG_I2C_mstrControl))
                        {
                            MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_XFER_HALT | 
                                                            MAG_I2C_GET_MSTAT_CMPLT);

                            MAG_I2C_state = MAG_I2C_SM_MSTR_HALT; /* Expect RESTART */
                            MAG_I2C_DisableInt();
                        }
                        else  /* Do normal Stop */
                        {
                            MAG_I2C_ENABLE_INT_ON_STOP; /* Enable interrupt on STOP, to catch it */
                            MAG_I2C_GENERATE_STOP;
                        }
                    }
                    else
                    {
                        /* Address phase is not set for some reason: error */
                        #if(MAG_I2C_TIMEOUT_ENABLED)
                            /* Exit from interrupt to take a chance for timeout timer handle this case */
                            MAG_I2C_DisableInt();
                            MAG_I2C_ClearPendingInt();
                        #else
                            /* Block execution flow: unexpected condition */
                            CYASSERT(0u != 0u);
                        #endif /* (MAG_I2C_TIMEOUT_ENABLED) */
                    }
                    break;

                case MAG_I2C_SM_MSTR_WR_DATA:

                    if(MAG_I2C_CHECK_DATA_ACK(tmpCsr))
                    {
                        /* Check if end of buffer */
                        if(MAG_I2C_mstrWrBufIndex  < MAG_I2C_mstrWrBufSize)
                        {
                            MAG_I2C_DATA_REG =
                                                     MAG_I2C_mstrWrBufPtr[MAG_I2C_mstrWrBufIndex];
                            MAG_I2C_TRANSMIT_DATA;
                            MAG_I2C_mstrWrBufIndex++;
                        }
                        /* End of buffer: complete writing */
                        else if(MAG_I2C_CHECK_NO_STOP(MAG_I2C_mstrControl))
                        {
                            /* Set WRITE complete and Master HALTED */
                            MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_XFER_HALT |
                                                            MAG_I2C_MSTAT_WR_CMPLT);

                            MAG_I2C_state = MAG_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                            MAG_I2C_DisableInt();
                        }
                        else  /* Do normal STOP */
                        {
                            MAG_I2C_Workaround();          /* Workaround: empty function */
                            MAG_I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                            MAG_I2C_GENERATE_STOP;
                        }
                    }
                    /* Last byte NAKed: end writing */
                    else if(MAG_I2C_CHECK_NO_STOP(MAG_I2C_mstrControl))
                    {
                        /* Set WRITE complete, SHORT transfer and Master HALTED */
                        MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_ERR_XFER       |
                                                        MAG_I2C_MSTAT_ERR_SHORT_XFER |
                                                        MAG_I2C_MSTAT_XFER_HALT      |
                                                        MAG_I2C_MSTAT_WR_CMPLT);

                        MAG_I2C_state = MAG_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                        MAG_I2C_DisableInt();
                    }
                    else  /* Do normal STOP */
                    {
                        MAG_I2C_ENABLE_INT_ON_STOP;    /* Enable interrupt on STOP, to catch it */
                        MAG_I2C_GENERATE_STOP;

                        /* Set SHORT and ERR transfer */
                        MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_ERR_SHORT_XFER |
                                                        MAG_I2C_MSTAT_ERR_XFER);
                    }
                    
                    break;

                case MAG_I2C_SM_MSTR_RD_DATA:

                    MAG_I2C_mstrRdBufPtr[MAG_I2C_mstrRdBufIndex] = MAG_I2C_DATA_REG;
                    MAG_I2C_mstrRdBufIndex++;

                    /* Check if end of buffer */
                    if(MAG_I2C_mstrRdBufIndex < MAG_I2C_mstrRdBufSize)
                    {
                        MAG_I2C_ACK_AND_RECEIVE;       /* ACK and receive byte */
                    }
                    /* End of buffer: complete reading */
                    else if(MAG_I2C_CHECK_NO_STOP(MAG_I2C_mstrControl))
                    {                        
                        /* Set READ complete and Master HALTED */
                        MAG_I2C_mstrStatus |= (MAG_I2C_MSTAT_XFER_HALT |
                                                        MAG_I2C_MSTAT_RD_CMPLT);
                        
                        MAG_I2C_state = MAG_I2C_SM_MSTR_HALT;    /* Expect RESTART */
                        MAG_I2C_DisableInt();
                    }
                    else
                    {
                        MAG_I2C_ENABLE_INT_ON_STOP;
                        MAG_I2C_NAK_AND_RECEIVE;       /* NACK and TRY to generate STOP */
                    }
                    break;

                default: /* This is an invalid state and should not occur */

                    #if(MAG_I2C_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        MAG_I2C_DisableInt();
                        MAG_I2C_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (MAG_I2C_TIMEOUT_ENABLED) */

                    break;
                }
            }

            /* Catches the Stop: end of transaction */
            if(MAG_I2C_CHECK_STOP_STS(tmpCsr))
            {
                MAG_I2C_mstrStatus |= MAG_I2C_GET_MSTAT_CMPLT;

                MAG_I2C_DISABLE_INT_ON_STOP;
                MAG_I2C_state = MAG_I2C_SM_IDLE;
            }
        #endif /* (MAG_I2C_MODE_MASTER_ENABLED) */
    }
    else if(MAG_I2C_CHECK_SM_SLAVE)
    {
        #if(MAG_I2C_MODE_SLAVE_ENABLED)
            
            if((MAG_I2C_CHECK_STOP_STS(tmpCsr)) || /* Stop || Restart */
               (MAG_I2C_CHECK_BYTE_COMPLETE(tmpCsr) && MAG_I2C_CHECK_ADDRESS_STS(tmpCsr)))
            {
                /* Catch end of master write transcation: use interrupt on Stop */
                /* The STOP bit history on address phase does not have correct state */
                if(MAG_I2C_SM_SL_WR_DATA == MAG_I2C_state)
                {
                    MAG_I2C_DISABLE_INT_ON_STOP;

                    MAG_I2C_slStatus &= ((uint8) ~MAG_I2C_SSTAT_WR_BUSY);
                    MAG_I2C_slStatus |= ((uint8)  MAG_I2C_SSTAT_WR_CMPLT);

                    MAG_I2C_state = MAG_I2C_SM_IDLE;
                }
            }

            if(MAG_I2C_CHECK_BYTE_COMPLETE(tmpCsr))
            {
                /* The address only issued after Start or ReStart: so check address
                   to catch this events:
                    FF : sets Addr phase with byte_complete interrupt trigger.
                    UDB: sets Addr phase immediately after Start or ReStart. */
                if(MAG_I2C_CHECK_ADDRESS_STS(tmpCsr))
                {
                    /* Check for software address detection */
                    #if(MAG_I2C_SW_ADRR_DECODE)
                        tmp8 = MAG_I2C_GET_SLAVE_ADDR(MAG_I2C_DATA_REG);

                        if(tmp8 == MAG_I2C_slAddress)   /* Check for address match */
                        {
                            if(0u != (MAG_I2C_DATA_REG & MAG_I2C_READ_FLAG))
                            {
                                /* Place code to prepare read buffer here                  */
                                /* `#START MAG_I2C_SW_PREPARE_READ_BUF_interrupt` */

                                /* `#END` */

                                /* Prepare next opeation to read, get data and place in data register */
                                if(MAG_I2C_slRdBufIndex < MAG_I2C_slRdBufSize)
                                {
                                    /* Load first data byte from array */
                                    MAG_I2C_DATA_REG = MAG_I2C_slRdBufPtr[MAG_I2C_slRdBufIndex];
                                    MAG_I2C_ACK_AND_TRANSMIT;
                                    MAG_I2C_slRdBufIndex++;

                                    MAG_I2C_slStatus |= MAG_I2C_SSTAT_RD_BUSY;
                                }
                                else    /* Overflow: provide 0xFF on the bus */
                                {
                                    MAG_I2C_DATA_REG = MAG_I2C_OVERFLOW_RETURN;
                                    MAG_I2C_ACK_AND_TRANSMIT;

                                    MAG_I2C_slStatus  |= (MAG_I2C_SSTAT_RD_BUSY |
                                                                   MAG_I2C_SSTAT_RD_ERR_OVFL);
                                }

                                MAG_I2C_state = MAG_I2C_SM_SL_RD_DATA;
                            }
                            else  /* Write transaction: receive 1st byte */
                            {
                                MAG_I2C_ACK_AND_RECEIVE;
                                MAG_I2C_state = MAG_I2C_SM_SL_WR_DATA;

                                MAG_I2C_slStatus |= MAG_I2C_SSTAT_WR_BUSY;
                                MAG_I2C_ENABLE_INT_ON_STOP;
                            }
                        }    
                        else
                        {
                            /*     Place code to compare for additional address here    */
                            /* `#START MAG_I2C_SW_ADDR_COMPARE_interruptStart` */

                            /* `#END` */
                            
                            MAG_I2C_NAK_AND_RECEIVE;   /* NACK address */

                            /* Place code to end of condition for NACK generation here */
                            /* `#START MAG_I2C_SW_ADDR_COMPARE_interruptEnd`  */

                            /* `#END` */
                        }
                        
                    #else /* (MAG_I2C_HW_ADRR_DECODE) */
                        
                        if(0u != (MAG_I2C_DATA_REG & MAG_I2C_READ_FLAG))
                        {
                            /* Place code to prepare read buffer here                  */
                            /* `#START MAG_I2C_HW_PREPARE_READ_BUF_interrupt` */

                            /* `#END` */

                            /* Prepare next opeation to read, get data and place in data register */
                            if(MAG_I2C_slRdBufIndex < MAG_I2C_slRdBufSize)
                            {
                                /* Load first data byte from array */
                                MAG_I2C_DATA_REG = MAG_I2C_slRdBufPtr[MAG_I2C_slRdBufIndex];
                                MAG_I2C_ACK_AND_TRANSMIT;
                                MAG_I2C_slRdBufIndex++;

                                MAG_I2C_slStatus |= MAG_I2C_SSTAT_RD_BUSY;
                            }
                            else    /* Overflow: provide 0xFF on the bus */
                            {
                                MAG_I2C_DATA_REG = MAG_I2C_OVERFLOW_RETURN;
                                MAG_I2C_ACK_AND_TRANSMIT;

                                MAG_I2C_slStatus  |= (MAG_I2C_SSTAT_RD_BUSY |
                                                               MAG_I2C_SSTAT_RD_ERR_OVFL);
                            }

                            MAG_I2C_state = MAG_I2C_SM_SL_RD_DATA;
                        }
                        else  /* Write transaction: receive 1st byte */
                        {
                            MAG_I2C_ACK_AND_RECEIVE;
                            MAG_I2C_state = MAG_I2C_SM_SL_WR_DATA;

                            MAG_I2C_slStatus |= MAG_I2C_SSTAT_WR_BUSY;
                            MAG_I2C_ENABLE_INT_ON_STOP;
                        }
                        
                    #endif /* (MAG_I2C_SW_ADRR_DECODE) */
                }
                /* Data states */
                /* Data master writes into slave */
                else if(MAG_I2C_state == MAG_I2C_SM_SL_WR_DATA)
                {
                    if(MAG_I2C_slWrBufIndex < MAG_I2C_slWrBufSize)
                    {
                        tmp8 = MAG_I2C_DATA_REG;
                        MAG_I2C_ACK_AND_RECEIVE;
                        MAG_I2C_slWrBufPtr[MAG_I2C_slWrBufIndex] = tmp8;
                        MAG_I2C_slWrBufIndex++;
                    }
                    else  /* of array: complete write, send NACK */
                    {
                        MAG_I2C_NAK_AND_RECEIVE;

                        MAG_I2C_slStatus |= MAG_I2C_SSTAT_WR_ERR_OVFL;
                    }
                }
                /* Data master reads from slave */
                else if(MAG_I2C_state == MAG_I2C_SM_SL_RD_DATA)
                {
                    if(MAG_I2C_CHECK_DATA_ACK(tmpCsr))
                    {
                        if(MAG_I2C_slRdBufIndex < MAG_I2C_slRdBufSize)
                        {
                             /* Get data from array */
                            MAG_I2C_DATA_REG = MAG_I2C_slRdBufPtr[MAG_I2C_slRdBufIndex];
                            MAG_I2C_TRANSMIT_DATA;
                            MAG_I2C_slRdBufIndex++;
                        }
                        else   /* Overflow: provide 0xFF on the bus */
                        {
                            MAG_I2C_DATA_REG = MAG_I2C_OVERFLOW_RETURN;
                            MAG_I2C_TRANSMIT_DATA;

                            MAG_I2C_slStatus |= MAG_I2C_SSTAT_RD_ERR_OVFL;
                        }
                    }
                    else  /* Last byte was NACKed: read complete */
                    {
                        /* Only NACK appears on the bus */
                        MAG_I2C_DATA_REG = MAG_I2C_OVERFLOW_RETURN;
                        MAG_I2C_NAK_AND_TRANSMIT;

                        MAG_I2C_slStatus &= ((uint8) ~MAG_I2C_SSTAT_RD_BUSY);
                        MAG_I2C_slStatus |= ((uint8)  MAG_I2C_SSTAT_RD_CMPLT);

                        MAG_I2C_state = MAG_I2C_SM_IDLE;
                    }
                }
                else
                {
                    #if(MAG_I2C_TIMEOUT_ENABLED)
                        /* Exit from interrupt to take a chance for timeout timer handle this case */
                        MAG_I2C_DisableInt();
                        MAG_I2C_ClearPendingInt();
                    #else
                        /* Block execution flow: unexpected condition */
                        CYASSERT(0u != 0u);
                    #endif /* (MAG_I2C_TIMEOUT_ENABLED) */
                }
            }
        #endif /* (MAG_I2C_MODE_SLAVE_ENABLED) */
    }
    else
    {
        /* The FSM skips master and slave processing: return to IDLE */
        MAG_I2C_state = MAG_I2C_SM_IDLE;
    }
}


#if((MAG_I2C_FF_IMPLEMENTED) && (MAG_I2C_WAKEUP_ENABLED))
    /*******************************************************************************
    * Function Name: MAG_I2C_WAKEUP_ISR
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
    CY_ISR(MAG_I2C_WAKEUP_ISR)
    {
        MAG_I2C_wakeupSource = 1u;  /* I2C was wakeup source */
        /* The SCL is stretched unitl the I2C_Wake() is called */
    }
#endif /* ((MAG_I2C_FF_IMPLEMENTED) && (MAG_I2C_WAKEUP_ENABLED))*/


/* [] END OF FILE */
