/*******************************************************************************
* File Name: CS.h  
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

#if !defined(CY_PINS_CS_H) /* Pins CS_H */
#define CY_PINS_CS_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "CS_aliases.h"


/***************************************
*        Function Prototypes             
***************************************/    

void    CS_Write(uint8 value) ;
void    CS_SetDriveMode(uint8 mode) ;
uint8   CS_ReadDataReg(void) ;
uint8   CS_Read(void) ;
uint8   CS_ClearInterrupt(void) ;


/***************************************
*           API Constants        
***************************************/

/* Drive Modes */
#define CS_DM_ALG_HIZ         PIN_DM_ALG_HIZ
#define CS_DM_DIG_HIZ         PIN_DM_DIG_HIZ
#define CS_DM_RES_UP          PIN_DM_RES_UP
#define CS_DM_RES_DWN         PIN_DM_RES_DWN
#define CS_DM_OD_LO           PIN_DM_OD_LO
#define CS_DM_OD_HI           PIN_DM_OD_HI
#define CS_DM_STRONG          PIN_DM_STRONG
#define CS_DM_RES_UPDWN       PIN_DM_RES_UPDWN

/* Digital Port Constants */
#define CS_MASK               CS__MASK
#define CS_SHIFT              CS__SHIFT
#define CS_WIDTH              1u


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define CS_PS                     (* (reg8 *) CS__PS)
/* Data Register */
#define CS_DR                     (* (reg8 *) CS__DR)
/* Port Number */
#define CS_PRT_NUM                (* (reg8 *) CS__PRT) 
/* Connect to Analog Globals */                                                  
#define CS_AG                     (* (reg8 *) CS__AG)                       
/* Analog MUX bux enable */
#define CS_AMUX                   (* (reg8 *) CS__AMUX) 
/* Bidirectional Enable */                                                        
#define CS_BIE                    (* (reg8 *) CS__BIE)
/* Bit-mask for Aliased Register Access */
#define CS_BIT_MASK               (* (reg8 *) CS__BIT_MASK)
/* Bypass Enable */
#define CS_BYP                    (* (reg8 *) CS__BYP)
/* Port wide control signals */                                                   
#define CS_CTL                    (* (reg8 *) CS__CTL)
/* Drive Modes */
#define CS_DM0                    (* (reg8 *) CS__DM0) 
#define CS_DM1                    (* (reg8 *) CS__DM1)
#define CS_DM2                    (* (reg8 *) CS__DM2) 
/* Input Buffer Disable Override */
#define CS_INP_DIS                (* (reg8 *) CS__INP_DIS)
/* LCD Common or Segment Drive */
#define CS_LCD_COM_SEG            (* (reg8 *) CS__LCD_COM_SEG)
/* Enable Segment LCD */
#define CS_LCD_EN                 (* (reg8 *) CS__LCD_EN)
/* Slew Rate Control */
#define CS_SLW                    (* (reg8 *) CS__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define CS_PRTDSI__CAPS_SEL       (* (reg8 *) CS__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define CS_PRTDSI__DBL_SYNC_IN    (* (reg8 *) CS__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define CS_PRTDSI__OE_SEL0        (* (reg8 *) CS__PRTDSI__OE_SEL0) 
#define CS_PRTDSI__OE_SEL1        (* (reg8 *) CS__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define CS_PRTDSI__OUT_SEL0       (* (reg8 *) CS__PRTDSI__OUT_SEL0) 
#define CS_PRTDSI__OUT_SEL1       (* (reg8 *) CS__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define CS_PRTDSI__SYNC_OUT       (* (reg8 *) CS__PRTDSI__SYNC_OUT) 


#if defined(CS__INTSTAT)  /* Interrupt Registers */

    #define CS_INTSTAT                (* (reg8 *) CS__INTSTAT)
    #define CS_SNAP                   (* (reg8 *) CS__SNAP)

#endif /* Interrupt Registers */

#endif /* End Pins CS_H */


/* [] END OF FILE */
