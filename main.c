/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 Critical Section
*              for ModusToolbox.
*
* Related Document: See README.md  
*
*******************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg_pins.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define SWITCH_INTR_PRIORITY    (3u)
#define LED_DELAY               (5000u)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void Switch_IntHandler(void);

/******************************************************************************
 * Switch interrupt configuration structure
 *****************************************************************************/
const cy_stc_sysint_t switch_interrupt_config =
{
    .intrSrc = CYBSP_USER_BTN_IRQ,             //Source of interrupt signal
    .intrPriority = SWITCH_INTR_PRIORITY,
};

/*******************************************************************************
* Function Name: Switch_IntHandler
********************************************************************************
*
* Summary:
*  This function is executed when interrupt is triggered through the switch.
*
*******************************************************************************/

void Switch_IntHandler(void)
{

    /* Clears the triggered pin interrupt */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_NUM);
    NVIC_ClearPendingIRQ(switch_interrupt_config.intrSrc);

    /*Toggle the User LED state*/
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - initial setup of device
*  - enables switch interrupts
*  - enables and disables critical section
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/

int main(void)
{
    cy_rslt_t result;
    uint32_t interruptState;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize switch interrupt */
    result = Cy_SysInt_Init(&switch_interrupt_config, &Switch_IntHandler);
    if (result != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable switch interrupt*/
    NVIC_ClearPendingIRQ(switch_interrupt_config.intrSrc);
    NVIC_EnableIRQ(switch_interrupt_config.intrSrc);

    for(;;)
    {

        /* Function to enter critical section*/
        interruptState = Cy_SysLib_EnterCriticalSection();

        /* Turn-on the external LED EXTNL_LED (CYBSP_EXTNL_LED) indicating entry of critical section */
        Cy_GPIO_Set(CYBSP_EXTNL_LED_PORT, CYBSP_EXTNL_LED_PIN);

        /* Critical section enabled for 5sec */
        Cy_SysLib_Delay(LED_DELAY);

        /* Function to exit critical section*/
        Cy_SysLib_ExitCriticalSection(interruptState);

        /* Turn-off the external LED EXTNL_LED (CYBSP_EXTNL_LED) indicating exit of critical section */
        Cy_GPIO_Clr(CYBSP_EXTNL_LED_PORT, CYBSP_EXTNL_LED_PIN);

        /* Critical section Disabled for 5sec */
        Cy_SysLib_Delay(LED_DELAY);
    }
}

/* [] END OF FILE */

