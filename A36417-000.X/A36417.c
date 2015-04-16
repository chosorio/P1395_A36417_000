#include "A36417.h"
#include "MCP4822.h"
#include "ETM_EEPROM.h"
#include "FIRMWARE_VERSION.h"

_FOSC(ECIO & CSW_FSCM_OFF);
_FWDT(WDT_ON & WDTPSA_512 & WDTPSB_8);  // 8 Second watchdog timer
_FBORPOR(PWRT_OFF & BORV_45 & PBOR_OFF & MCLR_EN);
_FBS(WR_PROTECT_BOOT_OFF & NO_BOOT_CODE & NO_BOOT_EEPROM & NO_BOOT_RAM);
_FSS(WR_PROT_SEC_OFF & NO_SEC_CODE & NO_SEC_EEPROM & NO_SEC_RAM);
_FGS(CODE_PROT_OFF);
_FICD(PGD);

void DoStateMachine(void);
void InitializeA36417(void);
void DoA36417_000(void);
void SelfTestA36417(void);

MCP4822 U11_MCP4822;

unsigned int control_state;
unsigned int EMCO_control_setpoint;
volatile unsigned int target_current_flag;
volatile unsigned int target_current;

unsigned int ADCDEBUG5, ADCDEBUG6, ADCDEBUG7;

SPid emco_pid;
IonPumpControlData global_data_A36417_000;

#define STATE_STARTUP                0x10
#define STATE_SELF_TEST              0x20
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
            control_state=STATE_SELF_TEST;
            break;

       case STATE_SELF_TEST:
           ETMCanSlaveDoCan();
           SelfTestA36417();
           break;

        case STATE_OPERATE:

            DoA36417_000();
            ETMCanSlaveDoCan();
            break;

        default:
            control_state = STATE_STARTUP;
            break;
    }
}

void DoA36417_000(void){
  if (global_data_A36417_000.trigger_recieved) {
    if (global_data_A36417_000.sample_level) {
      //ETMCanSlaveIonPumpSendTargetCurrentReading(0x2002, 0x0000, global_data_A36417_000.pulse_id);
    } else {
      //ETMCanSlaveIonPumpSendTargetCurrentReading(0x0000, 0x1001, global_data_A36417_000.pulse_id);
    }
    global_data_A36417_000.trigger_recieved = 0;
  }
  
    //If a target pulse was received

        ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_target_current);
        //need to send message up...don't know how.
        //basically, if high
        global_data_A36417_000.target_current_high=global_data_A36417_000.analog_input_target_current.reading_scaled_and_calibrated;
        //if low
        global_data_A36417_000.target_current_high=global_data_A36417_000.analog_input_target_current.reading_scaled_and_calibrated;

	

if(_T5IF){

            
    _T5IF=0;

    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_ion_pump_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_ion_pump_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_target_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_5V_monitor);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_15V_monitor);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_minus_5V_monitor);

    unsigned int ion_pump_voltage=global_data_A36417_000.analog_input_ion_pump_voltage.reading_scaled_and_calibrated;

    EMCO_control_setpoint=(unsigned int)(UpdatePID(&emco_pid,(EMCO_SETPOINT-(double)ion_pump_voltage), (double) ion_pump_voltage));


    local_debug_data.debug_8=EMCO_control_setpoint;

    //babysitter
    if(EMCO_control_setpoint>1200){
        EMCO_control_setpoint=1200;
    }
        local_debug_data.debug_7=EMCO_control_setpoint;
    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_ion_pump_voltage)) {
        //Maybe go to a fault state?
           _FAULT_ION_PUMP_OVER_VOLTAGE=1;
           //EMCO_control_setpoint=0;
    }
    else{
        _FAULT_ION_PUMP_OVER_VOLTAGE=0;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A36417_000.analog_input_ion_pump_voltage)) {
           _FAULT_ION_PUMP_UNDER_VOLTAGE=1;
    }
    else{
        _FAULT_ION_PUMP_OVER_VOLTAGE=0;
    }
        WriteMCP4822(&U11_MCP4822, MCP4822_OUTPUT_A_4096, EMCO_control_setpoint);


    local_debug_data.debug_1=global_data_A36417_000.analog_input_5V_monitor.reading_scaled_and_calibrated;
    local_debug_data.debug_2=global_data_A36417_000.analog_input_15V_monitor.reading_scaled_and_calibrated;
    local_debug_data.debug_3=global_data_A36417_000.analog_input_minus_5V_monitor.reading_scaled_and_calibrated;

// -------------------- CHECK FOR FAULTS ------------------- //

    if (_SYNC_CONTROL_RESET_ENABLE) {
      local_debug_data.debug_0++;
      _FAULT_REGISTER = 0x0000;
    }

     if (ETMAnalogCheckUnderAbsolute(&global_data_A36417_000.analog_input_ion_pump_voltage)) {
         _FAULT_ION_PUMP_UNDER_VOLTAGE=1;
    }
     else{

         _FAULT_ION_PUMP_UNDER_VOLTAGE=0;
     }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_ion_pump_current)) {
            _FAULT_ION_PUMP_OVER_CURRENT=1;
    }

    else{
           _FAULT_ION_PUMP_OVER_VOLTAGE=0;
    }


    //Write to DAC
    WriteMCP4822(&U11_MCP4822, MCP4822_OUTPUT_A_4096, EMCO_control_setpoint);

}
    return;
}

double UpdatePID(SPid* pid, double error, double reading){
    double pTerm,dTerm, iTerm;
    pTerm=pid->pGain*error;

    pid->iState +=error;
    if (pid->iState > pid->iMax){
        pid->iState = pid->iMax;
    }

    else if (pid->iState< pid->iMin){
        pid->iState = pid->iMin;
    }

  iTerm = pid->iGain * pid->iState;  // calculate the integral term
  dTerm = pid->dGain * (reading - pid->dState);
  pid->dState = reading;
    local_debug_data.debug_9=error;
    local_debug_data.debug_A=pTerm;
    local_debug_data.debug_B=iTerm;
    local_debug_data.debug_C=dTerm;
    if(pTerm + iTerm - dTerm < 0)
        return 1;
    else
        return pTerm + iTerm - dTerm;

}

void SelfTestA36417(void){
    int test_count=0;


        if(_T5IF){
            _T5IF=0;

            //Check 5V, -5V, 15V monitors
            ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_5V_monitor);
            ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_15V_monitor);
            ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_minus_5V_monitor);
            
            unsigned int _5Vmonitor=global_data_A36417_000.analog_input_5V_monitor.reading_scaled_and_calibrated;
            unsigned int _15Vmonitor=global_data_A36417_000.analog_input_15V_monitor.reading_scaled_and_calibrated;
            unsigned int minus_5Vmonitor=global_data_A36417_000.analog_input_minus_5V_monitor.reading_scaled_and_calibrated;



            if(_5Vmonitor>2400&&_5Vmonitor<2600){
                if(_15Vmonitor>2400&&_15Vmonitor<2600){
                    if(minus_5Vmonitor>1570&&minus_5Vmonitor<1770){
                        control_state=STATE_OPERATE;
                        return;
                    }
                }
            }
            
            if(test_count>SELF_TEST_FAIL_COUNT){
                //Set Board self check fail bit.
                _BOARD_SELF_CHECK_FAILED=1;
            }
            
        }
    
}

void InitializeA36417(void){

  // Configure Sample Target Current Interrupt
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT3EP = 0; // Positive Transition COSORIO check this
  _INT3IF=0; //Clear the interrupt flag.
  _INT3IE=1;    


  // Configure ADC Interrupt
  _ADIP   = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)

  // Initialize all I/O Registers
  TRISA = A36417_TRISA_VALUE;
  TRISB = A36417_TRISB_VALUE;
  TRISC = A36417_TRISC_VALUE;
  TRISD = A36417_TRISD_VALUE;
  TRISF = A36417_TRISF_VALUE;
  TRISG = A36417_TRISG_VALUE;
  
  target_current=0;
  target_current_flag=0;
  

  etm_can_my_configuration.firmware_major_rev = FIRMWARE_AGILE_REV;
  etm_can_my_configuration.firmware_branch = FIRMWARE_BRANCH;
  etm_can_my_configuration.firmware_minor_rev = FIRMWARE_MINOR_REV;

    U11_MCP4822.pin_chip_select_not = _PIN_RF2;
    U11_MCP4822.pin_load_dac_not = _PIN_RF3;
    U11_MCP4822.spi_port = ETM_SPI_PORT_1;
    U11_MCP4822.spi_con1_value = MCP4822_SPI_CON_VALUE;
    U11_MCP4822.spi_con2_value = MCP4822_SPI_CON2_VALUE;
    U11_MCP4822.spi_stat_value = MCP4822_SPI_STAT_VALUE;
    U11_MCP4822.spi_bit_rate = MCP4822_SPI_1_M_BIT;
    U11_MCP4822.fcy_clk = FCY_CLK;

    SetupMCP4822(&U11_MCP4822);

  _BOARD_SELF_CHECK_FAILED=0;

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

  //initialize PID control loop variables
  emco_pid.dGain=PID_DGAIN;
  emco_pid.dState=0;
  emco_pid.iState=0;
  emco_pid.iGain=PID_IGAIN;
  emco_pid.pGain=PID_PGAIN;
  emco_pid.iMax=PID_IMAX;
  emco_pid.iMin=PID_IMIN;

  EMCO_control_setpoint=0;
  WriteMCP4822(&U11_MCP4822, MCP4822_OUTPUT_A_4096, EMCO_control_setpoint);
  //Initialize analog input/output scaling

    global_data_A36417_000.analog_input_ion_pump_current.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ION_PUMP_CURRENT_SCALE_FACTOR);
    global_data_A36417_000.analog_input_ion_pump_current.fixed_offset                    = 0;
    global_data_A36417_000.analog_input_ion_pump_current.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_ion_pump_current.calibration_internal_offset     = 0;
    global_data_A36417_000.analog_input_ion_pump_current.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_ion_pump_current.calibration_external_offset     = 0;
    global_data_A36417_000.analog_input_ion_pump_current.over_trip_point_absolute        = ION_PUMP_CURRENT_OVER_TRIP_POINT;

    global_data_A36417_000.analog_input_ion_pump_voltage.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ION_PUMP_VOLTAGE_SCALE_FACTOR);
    global_data_A36417_000.analog_input_ion_pump_voltage.fixed_offset                    = 0;
    global_data_A36417_000.analog_input_ion_pump_voltage.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_ion_pump_voltage.calibration_internal_offset     = 0;
    global_data_A36417_000.analog_input_ion_pump_voltage.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_ion_pump_voltage.calibration_external_offset     = 0;
    global_data_A36417_000.analog_input_ion_pump_voltage.under_trip_point_absolute       = ION_PUMP_VOLTAGE_UNDER_TRIP_POINT;
    global_data_A36417_000.analog_input_ion_pump_voltage.over_trip_point_absolute       = ION_PUMP_VOLTAGE_OVER_TRIP_POINT;

    global_data_A36417_000.analog_input_target_current.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(TARGET_CURRENT_SCALE_FACTOR);
    global_data_A36417_000.analog_input_target_current.fixed_offset                    = 0;
    global_data_A36417_000.analog_input_target_current.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_target_current.calibration_internal_offset     = 0;
    global_data_A36417_000.analog_input_target_current.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_target_current.calibration_external_offset     = 0;
    global_data_A36417_000.analog_input_target_current.over_trip_point_absolute        = TARGET_CURRENT_OVER_TRIP_POINT;

    global_data_A36417_000.analog_input_5V_monitor.fixed_scale                         = MACRO_DEC_TO_SCALE_FACTOR_16(_5V_MONITOR_SCALE_FACTOR);
    global_data_A36417_000.analog_input_5V_monitor.fixed_offset                        = 0;
    global_data_A36417_000.analog_input_5V_monitor.calibration_internal_scale          = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_5V_monitor.calibration_internal_offset         = 0;
    global_data_A36417_000.analog_input_5V_monitor.calibration_external_scale          = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_5V_monitor.calibration_external_offset         = 0;

    global_data_A36417_000.analog_input_15V_monitor.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(_15V_MONITOR_SCALE_FACTOR);
    global_data_A36417_000.analog_input_15V_monitor.fixed_offset                    = 0;
    global_data_A36417_000.analog_input_15V_monitor.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_15V_monitor.calibration_internal_offset     = 0;
    global_data_A36417_000.analog_input_15V_monitor.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_15V_monitor.calibration_external_offset     = 0;

    global_data_A36417_000.analog_input_minus_5V_monitor.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(MINUS_5V_MONITOR_SCALE_FACTOR);
    global_data_A36417_000.analog_input_minus_5V_monitor.fixed_offset                    = 0;
    global_data_A36417_000.analog_input_minus_5V_monitor.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_minus_5V_monitor.calibration_internal_offset     = 0;
    global_data_A36417_000.analog_input_minus_5V_monitor.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_input_minus_5V_monitor.calibration_external_offset     = 0;

    global_data_A36417_000.analog_output_emco_control.fixed_scale                     = MACRO_DEC_TO_SCALE_FACTOR_16(ANALOG_OUT_SCALE_FACTOR);
    global_data_A36417_000.analog_output_emco_control.fixed_offset                    = 0;
    global_data_A36417_000.analog_output_emco_control.calibration_internal_scale      = MACRO_DEC_TO_CAL_FACTOR_2(ANALOG_OUT_INTERNAL_SCALE);
    global_data_A36417_000.analog_output_emco_control.calibration_internal_offset     = 0;
    global_data_A36417_000.analog_output_emco_control.calibration_external_scale      = MACRO_DEC_TO_CAL_FACTOR_2(1);
    global_data_A36417_000.analog_output_emco_control.calibration_external_offset     = 0;
    global_data_A36417_000.analog_output_emco_control.set_point                       = 0;
    global_data_A36417_000.analog_output_emco_control.enabled                         = 1;

   // Initialize the CAN module
  ETMCanSlaveInitialize();

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

void __attribute__((interrupt, no_auto_psv)) _INT3Interrupt(void){
  _INT3IF = 0;
  global_data_A36417_000.trigger_recieved = 1;
  global_data_A36417_000.pulse_id = etm_can_next_pulse_count;
  global_data_A36417_000.sample_level = etm_can_next_pulse_level;

    //ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_target_current);
    //Want to pass the message. preferably without actually calling other functions.(latency)
    //In order to to this, set some flags. Save the current reading from the ADC, and set flags to convert it.

    target_current_flag=1;

//  This is an attempt to sample the target current on the interrupt. For now, comment out. 
    //Clear ADON bit?
//
//    ADCHS = 0X000A; //Scan Channel 10
//    ADCSSL =0;  //No Scan
//    //Might need to set ADCON3- I dont think i have to. The sampling time doesnt need to change.
//    ADCON2=0; //Change this- you don't want an interrupt, just poll the done bit.
//    //Turn on the ADC
//    ADCON1.F15=1;
//
//    ADCON1.F1=1;     //Start Sampling
//    //wait about .5us?
//
//    //
//
//    //global_data_A36417_000.analog_input_target_current.filtered_adc_reading=target_current;
//    _INT3IF=0;
//    ADCDEBUG5=0;
}

void __attribute__((interrupt, no_auto_psv)) _ADCInterrupt(void) {
  _ADIF = 0;

  if (_BUFS) {
      // read ADCBUF 0-7
      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator          += ADCBUF0;
      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator          += ADCBUF1;
      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator                +=ADCBUF2;
      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator               +=ADCBUF3;
      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator          +=ADCBUF4;

      if(target_current_flag){
      global_data_A36417_000.analog_input_target_current.filtered_adc_reading       =ADCBUF5;
      }
      
      global_data_A36417_000.analog_input_target_current.adc_accumulator                +=ADCBUF5;
      if(ADCDEBUG5<ADCBUF5){
      ADCDEBUG5=ADCBUF5;
      }

      
    } else {
      // read ADCBUF 8-15
      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator          += ADCBUF8;
      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator          += ADCBUF9;
      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator                +=ADCBUFA;
      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator               +=ADCBUFB;
      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator          +=ADCBUFC;
          if(target_current_flag){
      global_data_A36417_000.analog_input_target_current.filtered_adc_reading       =ADCBUFD;
      }
      global_data_A36417_000.analog_input_target_current.adc_accumulator                +=ADCBUFD;
      if(ADCDEBUG5<ADCBUF5){
        ADCDEBUG5=ADCBUFD;
      }
    }

  global_data_A36417_000.accumulator_counter += 1;

  if (global_data_A36417_000.accumulator_counter >= 64) {

      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_ion_pump_current.filtered_adc_reading = global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator;
      global_data_A36417_000.analog_input_ion_pump_current.adc_accumulator = 0;

      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_ion_pump_voltage.filtered_adc_reading = global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator;
      global_data_A36417_000.analog_input_ion_pump_voltage.adc_accumulator = 0;

      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_5V_monitor.filtered_adc_reading = global_data_A36417_000.analog_input_5V_monitor.adc_accumulator;
      global_data_A36417_000.analog_input_5V_monitor.adc_accumulator = 0;

      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_15V_monitor.filtered_adc_reading = global_data_A36417_000.analog_input_15V_monitor.adc_accumulator;
      global_data_A36417_000.analog_input_15V_monitor.adc_accumulator = 0;

      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator >>= 2;  // This is now a 16 bit number average of previous 128 samples
      global_data_A36417_000.analog_input_minus_5V_monitor.filtered_adc_reading = global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator;
      global_data_A36417_000.analog_input_minus_5V_monitor.adc_accumulator = 0;

      global_data_A36417_000.analog_input_target_current.adc_accumulator >>=2;
      global_data_A36417_000.analog_input_target_current.adc_accumulator =0;
      global_data_A36417_000.accumulator_counter = 0;
  }

    local_debug_data.debug_4=global_data_A36417_000.analog_input_ion_pump_voltage.filtered_adc_reading;
    local_debug_data.debug_5=global_data_A36417_000.analog_input_ion_pump_current.filtered_adc_reading;
    local_debug_data.debug_6=global_data_A36417_000.analog_input_target_current.filtered_adc_reading;

}
