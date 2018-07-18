/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*
 *************************************************************************
 *
 *                           system.c
 *
 * System management file
 *
 *
 * Hardware:
 *  SAML21 Xplained Pro + SX1276 Radio Wing board
 *
 * Author            Date            Ver     Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * M15723         2014.10.12        0.5
 ******************************************************************************/

#include "sys.h"
#include "sw_timer.h"
#include "atomic.h"
#ifdef UT
#include "lora_test_main.h"
#else
#include "delay.h"
#include "system_interrupt.h"
#endif /* UT */

/** 
 * \brief Performs a blocking delay
 * \param[in] ms Delay time in milliseconds
 * \note : This function should allow interrupts to happen (unless it was called from
 * an interrupt itself) and keep its timing accurate. Ideally it should do the
 * waiting with the MCU in sleep.
 * Find out how long it takes the MCU to go to and wake up from sleep to see if
 * it makes sense to go to sleep at all 
 */
void SystemBlockingWaitMs(uint32_t ms)
{
#ifndef UT
    delay_ms(ms);
#endif
}

void System_GetExternalEui(uint8_t *id)
{

}

void system_enter_critical_section(void)
{
#ifndef UT
	system_interrupt_enter_critical_section();
#endif
}

void system_leave_critical_section(void)
{
#ifndef UT
	system_interrupt_leave_critical_section();
#endif
}

uint16_t System_GetAnalogReading(uint8_t channel)
{
	return 0;
}
