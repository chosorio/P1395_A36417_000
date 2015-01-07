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

MCP4822 U11_MCP4822;


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

if(_T5IF){

    _T5IF=0;

 // -------------------- CHECK FOR FAULTS ------------------- //


    if (global_reset_faults) {
      etm_can_system_debug_data.debug_0++;
      etm_can_status_register.status_word_1 = 0x0000;
      global_reset_faults = 0;
    }


}

    return;
}

void InitializeA36417(void){

    U11_MCP4822.pin_chip_select_not = _PIN_RF2;
    U11_MCP4822.pin_load_dac_not = _PIN_RF3;
    U11_MCP4822.spi_port = ETM_SPI_PORT_1;
    U11_MCP4822.spi_con1_value = MCP4822_SPI_CON_VALUE;
    U11_MCP4822.spi_con2_value = MCP4822_SPI_CON2_VALUE;
    U11_MCP4822.spi_stat_value = MCP4822_SPI_STAT_VALUE;
    U11_MCP4822.spi_bit_rate = MCP4822_SPI_1_M_BIT;
    U11_MCP4822.fcy_clk = FCY_CLK;

    SetupMCP4822(&U11_MCP4822);


  // Configure Inhibit Interrupt
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT1EP = 0; // Positive Transition

  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)


  // Initialize all I/O Registers
  TRISA = A36417_TRISA_VALUE;
  TRISB = A36417_TRISB_VALUE;
  TRISC = A36417_TRISC_VALUE;
  TRISD = A36417_TRISD_VALUE;
  TRISF = A36417_TRISF_VALUE;
  TRISG = A36417_TRISG_VALUE;

   // Initialize TMR5
  T5CON = T5CON_VALUE;
  TMR5  = 0;
  _T5IF = 0;
  PR5   = PR5_VALUE_10_MILLISECONDS;

  // Initialize integral ADC
  // ---- Configure the dsPIC ADC Module ------------ //
  ADCON1 = ADCON1_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON2 = ADCON2_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCON3 = ADCON3_SETTING;             // Configure the high speed ADC module based on H file parameters
  ADCHS  = ADCHS_SETTING;              // Configure the high speed ADC module based on H file parameters

  ADPCFG = ADPCFG_SETTING;             // Set which pins are analog and which are digital I/O
  ADCSSL = ADCSSL_SETTING;             // Set which analog pins are scanned
  _ADIF = 0;
  _ADIE = 1;
  _ADON = 1;

   // Initialize the CAN module
  ETMCanInitialize();

  // Flash LEDs at boot up
  __delay32(1000000);
  ClrWdt();
  PIN_LED_OPERATIONAL = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_TEST_POINT_A = 1;

  __delay32(1000000);
  ClrWdt();
  PIN_LED_TEST_POINT_B = 1;



}


void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;

  if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator          += ADCBUF0;
      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator          += ADCBUF1;
      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator                +=ADCBUF2;
      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator               +=ADCBUF3;
      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator          +=ADCBUF4;
      global_data_A36417_000.analog_input_target_current.adc_accumulator            +=ADCBUF5;
    } else {
      // read ADCBUF 8-15
      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator          += ADCBUF8;
      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator          += ADCBUF9;
      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator                +=ADCBUFA;
      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator               +=ADCBUFB;
      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator          +=ADCBUFC;
      global_data_A36417_000.analog_input_target_current.adc_accumulator            +=ADCBUFD;
    }

  global_data_A36417_000.accumulator_counter += 1;

  if (global_data_A36417_000.accumulator_counter >= 128) {

      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_ion_pump_current.filtered_adc_reading = global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator;
      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator = 0;

      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_ion_pump_voltage.filtered_adc_reading = global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator;
      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator = 0;

      global_data_A36417_000.analog_input_target_current.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_target_current.filtered_adc_reading = global_data_A36417_000.analog_input_target_current.adc_accumulator;
      global_data_A36417_000.analog_input_target_current.adc_accumulator = 0;

      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_5V_monitor.filtered_adc_reading = global_data_A36417_000.analog_input_5V_monitor.adc_accumulator;
      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator = 0;

      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_15V_monitor.filtered_adc_reading = global_data_A36417_000.analog_input_15V_monitor.adc_accumulator;
      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator = 0;

      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator >>= 3;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_minus_5V_monitor.filtered_adc_reading = global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator;
      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator = 0;

      global_data_A36417_000.accumulator_counter = 0;
  }


}