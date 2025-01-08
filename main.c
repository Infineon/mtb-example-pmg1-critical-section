/******************************************************************************
* File Name: main.c
*
* Description: This is the source code for the PMG1 Critical Section
*              for ModusToolbox.
*
* Related Document: See README.md  
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include <stdio.h>
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define SWITCH_INTR_PRIORITY    (3u)

/* Delay for 5sec */
#define LED_DELAY               (2000u)

/* CY ASSERT failure */
#define CY_ASSERT_FAILED        (0u)

/* Debug print macro to enable UART print */
#define DEBUG_PRINT             (0u)

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

#if DEBUG_PRINT

/* Structure for UART Context */
cy_stc_scb_uart_context_t CYBSP_UART_context;

/* Variable used for tracking the print status */
volatile bool ENTER_LOOP = true;

/*******************************************************************************
* Function Name: check_status
********************************************************************************
* Summary:
*  Prints the error message.
*
* Parameters:
*  error_msg - message to print if any error encountered.
*  status - status obtained after evaluation.
*
* Return:
*  void
*
*******************************************************************************/
void check_status(char *message, cy_rslt_t status)
{
    char error_msg[50];

    sprintf(error_msg, "Error Code: 0x%08" PRIX32 "\n", status);

    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\nFAIL: ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, message);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, error_msg);
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n=====================================================\r\n");
}
#endif

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
    cy_en_sysint_status_t intr_result;
    uint32_t interruptState;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

#if DEBUG_PRINT

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Sequence to clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");

    /* Print "Critical Section" */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "PMG1 MCU: Critical section");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "****************** \r\n\n");
#endif

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize switch interrupt */
    intr_result = Cy_SysInt_Init(&switch_interrupt_config, &Switch_IntHandler);
    if (intr_result != CY_SYSINT_SUCCESS)
    {
#if DEBUG_PRINT
        check_status("API Cy_SysInt_Init failed with error code", intr_result);
#endif
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable switch interrupt*/
    NVIC_ClearPendingIRQ(switch_interrupt_config.intrSrc);
    NVIC_EnableIRQ(switch_interrupt_config.intrSrc);

    for(;;)
    {
        /* Function to enter critical section*/
        interruptState = Cy_SysLib_EnterCriticalSection();

        /* Turn-on the external LED EXTNL_LED (CYBSP_EXTNL_LED) indicating entry of critical section
         * On-board LED is used for DUAL PORT devices thus the GPIO drive is reversed*/
#if PMG1_PD_DUALPORT_ENABLE
        Cy_GPIO_Clr(CYBSP_EXTNL_LED_PORT, CYBSP_EXTNL_LED_PIN);
#else
        Cy_GPIO_Set(CYBSP_EXTNL_LED_PORT, CYBSP_EXTNL_LED_PIN);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Critical section enabled for 5sec */
        Cy_SysLib_Delay(LED_DELAY);

        /* Function to exit critical section*/
        Cy_SysLib_ExitCriticalSection(interruptState);

        /* Turn-off the external LED EXTNL_LED (CYBSP_EXTNL_LED) indicating exit of critical section
         * On-board LED is used for DUAL PORT devices thus the GPIO drive is reversed*/
#if PMG1_PD_DUALPORT_ENABLE
        Cy_GPIO_Set(CYBSP_EXTNL_LED_PORT, CYBSP_EXTNL_LED_PIN);
#else
        Cy_GPIO_Clr(CYBSP_EXTNL_LED_PORT, CYBSP_EXTNL_LED_PIN);
#endif /* PMG1_PD_DUALPORT_ENABLE */

        /* Critical section Disabled for 5sec */
        Cy_SysLib_Delay(LED_DELAY);

#if DEBUG_PRINT
        if (ENTER_LOOP)
        {
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Entered for loop\r\n");
            ENTER_LOOP = false;
        }
#endif
    }
}

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

    /* Invert the User LED state*/
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
}

/* [] END OF FILE */
