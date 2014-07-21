/*******************************************************************************
* File Name: RTS.c  
* Version 2.0
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "RTS.h"


/*******************************************************************************
* Function Name: RTS_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void RTS_Write(uint8 value) 
{
    uint8 staticBits = (RTS_DR & (uint8)(~RTS_MASK));
    RTS_DR = staticBits | ((uint8)(value << RTS_SHIFT) & RTS_MASK);
}


/*******************************************************************************
* Function Name: RTS_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  RTS_DM_STRONG     Strong Drive 
*  RTS_DM_OD_HI      Open Drain, Drives High 
*  RTS_DM_OD_LO      Open Drain, Drives Low 
*  RTS_DM_RES_UP     Resistive Pull Up 
*  RTS_DM_RES_DWN    Resistive Pull Down 
*  RTS_DM_RES_UPDWN  Resistive Pull Up/Down 
*  RTS_DM_DIG_HIZ    High Impedance Digital 
*  RTS_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void RTS_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(RTS_0, mode);
}


/*******************************************************************************
* Function Name: RTS_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro RTS_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 RTS_Read(void) 
{
    return (RTS_PS & RTS_MASK) >> RTS_SHIFT;
}


/*******************************************************************************
* Function Name: RTS_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 RTS_ReadDataReg(void) 
{
    return (RTS_DR & RTS_MASK) >> RTS_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(RTS_INTSTAT) 

    /*******************************************************************************
    * Function Name: RTS_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 RTS_ClearInterrupt(void) 
    {
        return (RTS_INTSTAT & RTS_MASK) >> RTS_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
