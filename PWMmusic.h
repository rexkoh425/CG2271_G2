#include "MKL25Z4.h" 

#define PTB0_Pin  0
#define PTB1_Pin  1

/* #define CLOCK_SETUP 1 */
/* was set up in system_MKL25Z4.h */

/* Initialise PWM for Buzzer
   Using PTB0 for buzzer
*/

void initPWM(void)
{
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

    PORTB->PCR[PTB0_Pin] &= ~PORT_PCR_MUX_MASK;
    PORTB->PCR[PTB0_Pin] |= PORT_PCR_MUX(3);

    //PORTB->PCR[PTB1_Pin] &= ~PORT_PCR_MUX_MASK;
    //PORTB->PCR[PTB1_Pin] |= PORT_PCR_MUX(3);
	
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;

    SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);

    TPM1->MOD = 7500;

    TPM1->SC &= ~(TPM_SC_CMOD_MASK | TPM_SC_PS_MASK);
    TPM1->SC |= (TPM_SC_CMOD(1) | TPM_SC_PS(7));

    TPM1->SC &= ~(TPM_SC_CPWMS_MASK);

    TPM1_C0SC &= ~(TPM_CnSC_ELSA_MASK | TPM_CnSC_ELSB_MASK | TPM_CnSC_MSA_MASK);
    TPM1_C0SC |= (TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1));
}

/*int main (void) {
	SystemCoreClockUpdate();
	initPWM();
	TPM1_C0V = 7500;
	TPM1_C1V = 7500;
	while(1);
} */
