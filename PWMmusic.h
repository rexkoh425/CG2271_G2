#include "MKL25Z4.h" 

#define PTE22_Pin  22
#define WEE_FREQUENCY 4000
#define WOO_FREQUENCY 1000

/* #define CLOCK_SETUP 1 */
/* was set up in system_MKL25Z4.h */

/* Initialise PWM for Buzzer
   Using PTE22 for buzzer
   TPM2 Channel 0
*/

void initPWM_buzzer(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;
    
    PORTE->PCR[PTE22_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[PTE22_Pin] |= PORT_PCR_MUX(3);
	
    SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;

    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    TPM2->MOD = 7500;

    TPM2->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
    TPM2->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));

    TPM2->SC &= ~(TPM_SC_CPWMS_MASK);

    TPM2_C0SC &= ~(TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK);
    TPM2_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

void setBuzzerFrequency(int frequency)
{
    int mod_value = 375000 / frequency;  // Calculate the MOD value for the given frequency
    TPM2->MOD = mod_value;
    TPM2_C0V = mod_value / 2;            // Set duty cycle to 50%
}

