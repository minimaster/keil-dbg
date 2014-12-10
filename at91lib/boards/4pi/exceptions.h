/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

/*
** This file contains the default exception handlers
** and exception table.
*/

//------------------------------------------------------------------------------
//         Types
//------------------------------------------------------------------------------

/// Function prototype for exception table items - interrupt handler.
//typedef void( *IrqHandler )( void );
typedef void( *IntFunc )( void );

/// Weak attribute

#if defined   ( __CC_ARM   )
    #define WEAK __attribute__ ((weak))
#elif defined ( __ICCARM__ )
    #define WEAK __weak
#elif defined (  __GNUC__  )
    #define WEAK __attribute__ ((weak))
#endif

//------------------------------------------------------------------------------
//         Global functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//         Exception Handlers
//------------------------------------------------------------------------------

extern WEAK void NMI_Handler( void );
extern WEAK void HardFault_Handler( void );
extern WEAK void MemManage_Handler( void );
extern WEAK void BusFault_Handler( void );
extern WEAK void UsageFault_Handler( void );
extern WEAK void SVC_Handler( void );
extern WEAK void DebugMon_Handler( void );
extern WEAK void PendSV_Handler( void );
extern WEAK void SysTick_Handler( void );

// SUPPLY CONTROLLER
extern WEAK void SUPC_IRQHandler(void);
// RESET CONTROLLER
extern WEAK void RSTC_IRQHandler(void);
// REAL TIME CLOCK
extern WEAK void RTC_IRQHandler(void);
// REAL TIME TIMER
extern WEAK void RTT_IRQHandler(void);
// WATCHDOG TIMER
extern WEAK void WDT_IRQHandler(void);
// PMC
extern WEAK void PMC_IRQHandler(void);
// EFC0
extern WEAK void EFC0_IRQHandler(void);
// EFC1
extern WEAK void EFC1_IRQHandler(void);
// DBGU
extern WEAK void UART_IRQHandler(void);
// HSMC4
extern WEAK void SMCI_IRQHandler(void);
// Parallel IO Controller A
extern WEAK void PIOA_IRQHandler(void);
// Parallel IO Controller B
extern WEAK void PIOB_IRQHandler(void);
// Parallel IO Controller C
extern WEAK void PIOC_IRQHandler(void);
// USART 0
extern WEAK void USART0_IRQHandler(void);
// USART 1
extern WEAK void USART1_IRQHandler(void);
// USART 2
extern WEAK void USART2_IRQHandler(void);
// USART 3
extern WEAK void USART3_IRQHandler(void);
// Multimedia Card Interface
extern WEAK void HSMCI_IRQHandler(void);
// TWI 0
extern WEAK void TWI0_IRQHandler(void);
// TWI 1
extern WEAK void TWI1_IRQHandler(void);
// Serial Peripheral Interface 0
extern WEAK void SPI_IRQHandler(void);
// Serial Synchronous Controller 0
extern WEAK void SSC_IRQHandler(void);
// Timer Counter 0
extern WEAK void TC0_IRQHandler(void);
// Timer Counter 1
extern WEAK void TC1_IRQHandler(void);
// Timer Counter 2
extern WEAK void TC2_IRQHandler(void);
// PWM Controller
extern WEAK void PWM_IRQHandler(void);
// ADC controller0
extern WEAK void ADC12B_IRQHandler(void);
// ADC controller1
extern WEAK void ADC_IRQHandler(void);
// HDMA
extern WEAK void DMAC_IRQHandler(void);
// USB Device High Speed UDP_HS
extern WEAK void UDPD_IRQHandler(void);

