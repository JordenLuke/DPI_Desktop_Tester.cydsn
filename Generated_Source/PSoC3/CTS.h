/*******************************************************************************
* File Name: CTS.h  
* Version 2.0
*
* Description:
*  This file containts Control Register function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_CTS_H) /* Pins CTS_H */
#define CY_PINS_CTS_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CTS_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    CTS_Write(uint8 value) ;
void    CTS_SetDriveMode(uint8 mode) ;
uint8   CTS_ReadDataReg(void) ;
uint8   CTS_Read(void) ;
uint8   CTS_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define CTS_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define CTS_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define CTS_DM_RES_UP          PIN_DM_RES_UP
#define CTS_DM_RES_DWN         PIN_DM_RES_DWN
#define CTS_DM_OD_LO           PIN_DM_OD_LO
#define CTS_DM_OD_HI           PIN_DM_OD_HI
#define CTS_DM_STRONG          PIN_DM_STRONG
#define CTS_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define CTS_MASK               CTS__MASK
#define CTS_SHIFT              CTS__SHIFT
#define CTS_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CTS_PS                     (* (reg8 *) CTS__PS)
/* Data Register */
#define CTS_DR                     (* (reg8 *) CTS__DR)
/* Port Number */
#define CTS_PRT_NUM                (* (reg8 *) CTS__PRT) 
/* Connect to Analog Globals */                                                  
#define CTS_AG                     (* (reg8 *) CTS__AG)                       
/* Analog MUX bux enable */
#define CTS_AMUX                   (* (reg8 *) CTS__AMUX) 
/* Bidirectional Enable */                                                        
#define CTS_BIE                    (* (reg8 *) CTS__BIE)
/* Bit-mask for Aliased Register Access */
#define CTS_BIT_MASK               (* (reg8 *) CTS__BIT_MASK)
/* Bypass Enable */
#define CTS_BYP                    (* (reg8 *) CTS__BYP)
/* Port wide control signals */                                                   
#define CTS_CTL                    (* (reg8 *) CTS__CTL)
/* Drive Modes */
#define CTS_DM0                    (* (reg8 *) CTS__DM0) 
#define CTS_DM1                    (* (reg8 *) CTS__DM1)
#define CTS_DM2                    (* (reg8 *) CTS__DM2) 
/* Input Buffer Disable Override */
#define CTS_INP_DIS                (* (reg8 *) CTS__INP_DIS)
/* LCD Common or Segment Drive */
#define CTS_LCD_COM_SEG            (* (reg8 *) CTS__LCD_COM_SEG)
/* Enable Segment LCD */
#define CTS_LCD_EN                 (* (reg8 *) CTS__LCD_EN)
/* Slew Rate Control */
#define CTS_SLW                    (* (reg8 *) CTS__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CTS_PRTDSI__CAPS_SEL       (* (reg8 *) CTS__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CTS_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CTS__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CTS_PRTDSI__OE_SEL0        (* (reg8 *) CTS__PRTDSI__OE_SEL0) 
#define CTS_PRTDSI__OE_SEL1        (* (reg8 *) CTS__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CTS_PRTDSI__OUT_SEL0       (* (reg8 *) CTS__PRTDSI__OUT_SEL0) 
#define CTS_PRTDSI__OUT_SEL1       (* (reg8 *) CTS__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CTS_PRTDSI__SYNC_OUT       (* (reg8 *) CTS__PRTDSI__SYNC_OUT) 


#if defined(CTS__INTSTAT)  /* Interrupt Registers */

    #define CTS_INTSTAT                (* (reg8 *) CTS__INTSTAT)
    #define CTS_SNAP                   (* (reg8 *) CTS__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins CTS_H */


/* [] END OF FILE */
