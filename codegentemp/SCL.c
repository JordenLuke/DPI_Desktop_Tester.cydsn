/*******************************************************************************
* File Name: SCL.c  
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
#include "SCL.h"


/*******************************************************************************
* Function Name: SCL_Write
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
void SCL_Write(uint8 value) 
{
    uint8 staticBits = (SCL_DR & (uint8)(~SCL_MASK));
    SCL_DR = staticBits | ((uint8)(value << SCL_SHIFT) & SCL_MASK);
}


/*******************************************************************************
* Function Name: SCL_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  SCL_DM_STRONG     Strong Drive 
*  SCL_DM_OD_HI      Open Drain, Drives High 
*  SCL_DM_OD_LO      Open Drain, Drives Low 
*  SCL_DM_RES_UP     Resistive Pull Up 
*  SCL_DM_RES_DWN    Resistive Pull Down 
*  SCL_DM_RES_UPDWN  Resistive Pull Up/Down 
*  SCL_DM_DIG_HIZ    High Impedance Digital 
*  SCL_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void SCL_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(SCL_0, mode);
}


/*******************************************************************************
* Function Name: SCL_Read
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
*  Macro SCL_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 SCL_Read(void) 
{
    return (SCL_PS & SCL_MASK) >> SCL_SHIFT;
}


/*******************************************************************************
* Function Name: SCL_ReadDataReg
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
uint8 SCL_ReadDataReg(void) 
{
    return (SCL_DR & SCL_MASK) >> SCL_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(SCL_INTSTAT) 

    /*******************************************************************************
    * Function Name: SCL_ClearInterrupt
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
    uint8 SCL_ClearInterrupt(void) 
    {
        return (SCL_INTSTAT & SCL_MASK) >> SCL_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
