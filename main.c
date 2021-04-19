/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/******************************************************************************
 * MSP432 Empty Project
 *
 * Description: An empty project that uses DriverLib
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST               |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 *            |                  |
 * Author: 
*******************************************************************************/
/*
 * pwm.c
 *
 *  Created on: Apr 2, 2021
 *      Author: Yu L
 */
#include "sensor.h"
#include <msp.h>
#include <stdio.h>

void config_pulse_timer(void){
    TIMER_A0->CTL |= TIMER_A_CTL_CLR;   //clear
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK; //3MHz Clock
    TIMER_A0->CTL |= TIMER_A_CTL_ID__4;  //divide by 2^4
    TIMER_A0->CCR[0] |= TICKS; //load CCR0 with calculated value
    TIMER_A0->CCR[1] |= TICKS2;
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_7;  //output mode switched to transition mode
    TIMER_A0->CCTL[1] &= TIMER_A_CCTLN_OUTMOD_2;  //select toggle/reset output mode
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_CCIE; //capture/compare interrupt

}

//start the timer by making  MC > 0

void start_pulse(void){
    TIMER_A0->CTL |= TIMER_A_CTL_MC__UP;   //up mode, counts to TAxCCR0, starts timer
}

void stop_pulse(void){
    TIMER_A0->CTL &= ~(TIMER_A_CTL_MC__STOP); //stops timer, timer must be stopped before it can be configured
}

void config_gpio(void){    //configuring output bits of the pulse signal, also lights an LED for visual feedback
    P1->DIR |= BIT0;
    P1->OUT |= BIT0;
    P1->DS |= BIT0;

    P2->DIR |= BIT0;
    P2->DS |= BIT0;
    P2->OUT |= BIT0;

}

void config_capture_timer(void){
    TIMER_A1->CTL |= TIMER_A_CTL_CLR;   //clear
    TIMER_A1->CTL |= TIMER_A_CTL_SSEL__SMCLK; //3MHz Clock
    TIMER_A1->CTL |= TIMER_A_CTL_ID__4;  //divide by 2^4
    TIMER_A1->CCR[0] |= TICKS; //load CCR0 with calculated value
    TIMER_A1->CCTL[0] |= TIMER_A_CCTLN_OUTMOD_7;  //output mode switched to transition mode
    TIMER_A1->CCTL[0] &= TIMER_A_CCTLN_OUTMOD_4;  //select Toggle output mode


    //posedge capture

    TIMER_A1->CCTL[1] |= TIMER_A_CCTLN_CCIS_0; // TA1.CCIA selected as the capture pin
    P7->SEL0 |= BIT7; //Port 7-7 secondary function
    P7->DIR &= ~BIT7;
    TIMER_A1->CCTL[1] |= TIMER_A_CCTLN_SCS; //synchronous capture


    TIMER_A1->CCTL[1] |= TIMER_A_CCTLN_CAP;//select capture mode
    //TIMER_A1->CCTL[1] |= TIMER_A_CCTLN_CCIE; //capture/compare interrupt
    TIMER_A1->CCTL[1] |= TIMER_A_CCTLN_CM_1; //capture on positive edge
    //TIMER_A1->CCTL[1] |= TIMER_A_CCTLN_CCIS_0; // TA1.CCIA selected as the capture pin

    TIMER_A1->CCTL[1] |= TIMER_A_CCTLN_CCIE; //capture/compare interrupt

    TIMER_A1->CCTL[2] |= TIMER_A_CCTLN_CCIE; //capture/compare interrupt
}

