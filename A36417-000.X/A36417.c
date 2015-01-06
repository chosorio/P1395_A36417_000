#include "A36417.h"
#include "MCP4822.h"

_FOSC(ECIO & CSW_FSCM_OFF);
_FWDT(WDT_ON & WDTPSA_64 & WDTPSB_8);  // 1 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);

void DoStateMachine(void);
void InitializeA36417(void);
void DoA36417_000(void);

MCP4822 U42_MCP4822;
MCP4822 U44_MCP4822;


unsigned int control_state;

IonPumpControlData global_data_A36417_000;

#define STATE_STARTUP                0x10
#define STATE_WAITING_FOR_CONFIG     0x20
#define STATE_OPERATE                0x30

int main (void){
    control_state=STATE_STARTUP;
    while(1){
    DoStateMachine();
    }
}

void DoStateMachine(void){

   switch(control_state){
        case STATE_STARTUP:
            InitializeA36417();
            control_state=STATE_OPERATE;
            break;

        case STATE_OPERATE:
            DoA36417_000();
            ETMCanDoCan();
            break;

        default:
            control_state = STATE_STARTUP;
            break;
    }
}

void DoA36417_000(void){


    return;
}

void InitializeA36417(void){

    
    return;
}