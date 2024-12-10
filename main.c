/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the HPPASS on-chip temperature sensor
*              reading example for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "cy_pdl.h"
#include "mtb_hal.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* The definition of HPPASS AC startup timeout in microseconds.
 * HPPASS startup time contains AREF startup 40us, CSG startup about 15us and 
 * SAR ADC maximum self-calibration time 9ms (HPPASS input clock is 240MHz). To be
 * on the safe side, add to 10ms.
 */
#define HPPASS_AC_STARTUP_TIMEOUT           (10000U)

/* HPPASS AC states definition for on-chip temperature sensor reading.
 * NOTE: These states have been configured in HPPASS AC STT of device configurator.
 */
#define TEMP_VBE2_READING_STATE             (1U) 
#define TEMP_FW_TRIGGER_STATE               (2U)
#define TEMP_VBE1_READING_STATE             (3U)
#define TEMP_READING_COMPLETE_STATE         (4U)

/* The FIFO index of temperature sensor ADC channel */
#define TEMP_RESULT_FIFO_IDX                (0U)

/* The number of temperature results in FIFO */
#define NUM_TEMP_RESULTS_IN_FIFO            (2U)

/* Return status definition */
#define RET_STATUS_SUCCESS                  (0x00)
#define RET_STATUS_FAILED                   (0x01)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* For the Retarget-IO (Debug UART) usage */
static cy_stc_scb_uart_context_t  DEBUG_UART_context;   /* Debug UART context */
static mtb_hal_uart_t DEBUG_UART_hal_obj;               /* Debug UART HAL object */

/*******************************************************************************
* Function Prototypes
********************************************************************************/
/* Read current temperature from on-chip temperature sensor */
uint8_t read_on_chip_temperature(int16_t *temp, uint16_t timeoutUs);

/* Check if the user button is pressed */
bool user_button_is_pressed(void);


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    int16_t cur_temp = 0;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize the debug UART */
    result = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    /* UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&DEBUG_UART_hal_obj, &DEBUG_UART_hal_config, &DEBUG_UART_context, NULL);
    /* HAL UART init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(&DEBUG_UART_hal_obj);
    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Start the HPPASS autonomous controller (AC) from state 0 */
    if(CY_HPPASS_SUCCESS != Cy_HPPASS_AC_Start(0U, HPPASS_AC_STARTUP_TIMEOUT))
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("********************************************************************************\r\n");
    printf("HPPASS: On-chip temperature sensor reading example\r\n");
    printf("********************************************************************************\r\n");
    printf("Press user switch (SW2) to display the die temperature.\r\n");
    
    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
         /* Check if 'SW2' key was pressed */
        if(user_button_is_pressed())
        {
            /* Read current die temperature from on-chip temperature sensor */
            if(RET_STATUS_SUCCESS != read_on_chip_temperature(&cur_temp, 100))
            {
                CY_ASSERT(0);
            }
            printf("Die temperature: %d C\r\n", cur_temp);
        }
    }
}

/*******************************************************************************
* Function Name: wait_for_ac_run_to_state
********************************************************************************
* Summary:
* This function wait for Autonomous Controller (AC) execute to specified state.
*
* Parameters:
* \param state
* Starting state index in the State Transition Table.

* \param timeoutUs
* The timeout in microseconds.
*
* Return:
*  Status of execution.
*
*******************************************************************************/
static uint8_t wait_for_ac_run_to_state(uint8_t state, uint16_t timeoutUs)
{
    uint8_t ac_cur_state = HPPASS_AC_STATUS(HPPASS_BASE) & HPPASS_ACTRLR_STATUS_CUR_STATE_Msk;

    while(ac_cur_state != state)
    {
        if(timeoutUs > 0)
        {
            timeoutUs--;
            Cy_SysLib_DelayUs(1U);
        }
        if(0U == timeoutUs)
        {
            return RET_STATUS_FAILED;
        }
        /* Get AC currently execution state */
        ac_cur_state = HPPASS_AC_STATUS(HPPASS_BASE) & HPPASS_ACTRLR_STATUS_CUR_STATE_Msk;
    }

    return RET_STATUS_SUCCESS;
}

/*******************************************************************************
* Function Name: read_on_chip_temperature
********************************************************************************
* Summary:
* This function read current temperature from on-chip temperature sensor.
*
* Parameters:
* \param tempOutput
* Temperature output value.

* \param timeoutUs
* The timeout in microseconds.
*
* Return:
*  Status of execution.
*
*******************************************************************************/
uint8_t read_on_chip_temperature(int16_t *tempOutput, uint16_t timeoutUs)
{
    cy_en_hppass_status_t status = CY_HPPASS_BAD_PARAM;
    uint8_t ret_status = RET_STATUS_FAILED;
    uint32_t temp_vbe2 = 0;
    uint32_t temp_vbe1 = 0;

    if(tempOutput == NULL)
    {
        return ret_status;
    }

    /* Temperature sensor current selection: low current */
    Cy_HPPASS_SAR_SetTempSensorCurrent(false);
    
    /* Start the AC from state 1 (TEMP_VBE2_READING_STATE) to read temperature */
    status = Cy_HPPASS_AC_Start(TEMP_VBE2_READING_STATE, 0U);
    if(CY_HPPASS_SUCCESS != status)
    {
        return RET_STATUS_FAILED;
    }

    /* AC state 1: TEMP_VBE2_READING_STATE
     * AC trigger ADC group 0 conversion, and wait for group 0 conversion is done.
     * Don't need to do anything in the code for state 1.
     */

    /* Wait for AC execute to state 2 (TEMP_FW_TRIGGER_STATE) */
    ret_status = wait_for_ac_run_to_state(TEMP_FW_TRIGGER_STATE, timeoutUs);
    if(RET_STATUS_SUCCESS != ret_status)
    {
        return ret_status;
    }
    
    /* AC state 2: TEMP_FW_TRIGGER_STATE
     * Wait for HPPASS FW pulse trigger 0.
     * NOTE: Firstly set temp sensor current to high, then generate FW trigger 0 to measure VBE1.
     */
    /* Temperature sensor current selection: high current */
    Cy_HPPASS_SAR_SetTempSensorCurrent(true);

    /* Generates FW Pulse trigger 0 to jump to AC state 3 */
    Cy_HPPASS_SetFwTriggerPulse(CY_HPPASS_TRIG_0_MSK);

    /* AC state 3: TEMP_VBE1_READING_STATE
     * AC trigger ADC group 0 conversion, and wait for group 0 conversion is done.
     * Don't need to do anything in the code for state 3.
     */

    /* Wait for AC execute to state 4 (TEMP_READING_COMPLETE_STATE) */
    ret_status = wait_for_ac_run_to_state(TEMP_READING_COMPLETE_STATE, timeoutUs);
    if(RET_STATUS_SUCCESS != ret_status)
    {
        return ret_status;
    }

    /* Get the VBE2 and VBE1 from FIFO */
    if(NUM_TEMP_RESULTS_IN_FIFO == Cy_HPPASS_FIFO_GetSampleCount(TEMP_RESULT_FIFO_IDX))
    {
        temp_vbe2 = *CY_HPPASS_SAR_FIFO_READ_PTR(TEMP_RESULT_FIFO_IDX);
        temp_vbe1 = *CY_HPPASS_SAR_FIFO_READ_PTR(TEMP_RESULT_FIFO_IDX);
    }

    /* Calculates the temperature based on two ADC reading results */
    *tempOutput = Cy_HPPASS_TEMP_Calc(temp_vbe1, temp_vbe2);

    return ret_status;
}

/*******************************************************************************
* Function Name: user_button_is_pressed
****************************************************************************//**
* Summary:
*  Check if the user button is pressed.
*
* Return:
*  Returns the status of user button.
*
*******************************************************************************/
bool user_button_is_pressed(void)
{
    uint32_t pressCount = 0;

    if(Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) != CYBSP_BTN_PRESSED)
    {
        return false;
    }
    /* Check if User button is pressed */
    while (Cy_GPIO_Read(CYBSP_USER_BTN2_PORT, CYBSP_USER_BTN2_NUM) == CYBSP_BTN_PRESSED)
    {
        /* Wait for 10 ms */
        Cy_SysLib_Delay(10);
        pressCount++;
    }
    /* Add a delay to avoid glitches */
    Cy_SysLib_Delay(10);

    if(10 < pressCount)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/* [] END OF FILE */
