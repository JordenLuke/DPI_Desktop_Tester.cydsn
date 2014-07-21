/*******************************************************************************
* File Name: CTS.c  
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
#include "CTS.h"


/*******************************************************************************
* Function Name: CTS_Write
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
void CTS_Write(uint8 value) 
{
    uint8 staticBits = (CTS_DR & (uint8)(~CTS_MASK));
    CTS_DR = staticBits | ((uint8)(value << CTS_SHIFT) & CTS_MASK);
}


/*******************************************************************************
* Function Name: CTS_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  CTS_DM_STRONG     Strong Drive 
*  CTS_DM_OD_HI      Open Drain, Drives High 
*  CTS_DM_OD_LO      Open Drain, Drives Low 
*  CTS_DM_RES_UP     Resistive Pull Up 
*  CTS_DM_RES_DWN    Resistive Pull Down 
*  CTS_DM_RES_UPDWN  Resistive Pull Up/Down 
*  CTS_DM_DIG_HIZ    High Impedance Digital 
*  CTS_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void CTS_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(CTS_0, mode);
}


/*******************************************************************************
* Function Name: CTS_Read
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
*  Macro CTS_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 CTS_Read(void) 
{
    return (CTS_PS & CTS_MASK) >> CTS_SHIFT;
}


/*******************************************************************************
* Function Name: CTS_ReadDataReg
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
uint8 CTS_ReadDataReg(void) 
{
    return (CTS_DR & CTS_MASK) >> CTS_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(CTS_INTSTAT) 

    /*******************************************************************************
    * Function Name: CTS_ClearInterrupt
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
    uint8 CTS_ClearInterrupt(void) 
    {
        return (CTS_INTSTAT & CTS_MASK) >> CTS_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
