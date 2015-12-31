

#include "A36417.h"
#include "A36417_SETTINGS.h"
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
void Reset_Faults(void);
//void SelfTestA36417(void);
unsigned int Check_UV_OC_Fault(void);
unsigned int Check_Supplies(void);


volatile unsigned int target_current_flag;
volatile unsigned int target_current;

unsigned int ADCDEBUG5, ADCDEBUG6, ADCDEBUG7;

MCP4822 U11_MCP4822;

SPid emco_pid;
IonPumpControlData global_data_A36417_000;

#define STATE_STARTUP                0x10
#define STATE_SELF_TEST              0x20
#define STATE_OPERATE                0x30
#define STATE_FAULT                  0x40

int main (void){
    global_data_A36417_000.control_state = STATE_STARTUP;
    while(1){
    DoStateMachine();
    }
}

void DoStateMachine(void){

   switch(global_data_A36417_000.control_state){
        case STATE_STARTUP:
            _CONTROL_NOT_READY = 1;
            InitializeA36417();
            Reset_Faults();
            global_data_A36417_000.control_state = STATE_SELF_TEST;
            break;

       case STATE_SELF_TEST:
           //ETMCanSlaveDoCan();
           _CONTROL_NOT_READY = 1;
           global_data_A36417_000.EMCO_enable = 1;
           global_data_A36417_000.self_test_count = 0;
           while (global_data_A36417_000.control_state == STATE_SELF_TEST) {
             DoA36417_000();

             if (global_data_A36417_000.self_test_count > SELF_TEST_TIME) {

               if(Check_UV_OC_Fault() == 0) {
                 global_data_A36417_000.control_state = STATE_OPERATE;
               }
               if(Check_Supplies()) {     //fail self test
                 global_data_A36417_000.control_state = STATE_FAULT;
               }
               if (_FAULT_ION_PUMP_OVER_VOLTAGE) {
                 global_data_A36417_000.control_state = STATE_FAULT;
               }
             }

             if(global_data_A36417_000.self_test_count >= MAX_SELF_TEST_TIME) {
               global_data_A36417_000.control_state = STATE_FAULT;
             }

           }
           break;

        case STATE_OPERATE:
            _CONTROL_NOT_READY = 0;
            global_data_A36417_000.EMCO_enable = 1;

            while (global_data_A36417_000.control_state == STATE_OPERATE) {
              DoA36417_000();
              //ETMCanSlaveDoCan();
              if (Check_UV_OC_Fault()) {
                global_data_A36417_000.control_state = STATE_SELF_TEST;
              }
              if (_FAULT_ION_PUMP_OVER_VOLTAGE) {
                global_data_A36417_000.control_state = STATE_FAULT;
              }
            }
            break;

       case STATE_FAULT:
            _CONTROL_NOT_READY = 1;
            global_data_A36417_000.EMCO_enable = 0;
            while (global_data_A36417_000.control_state == STATE_FAULT) {
              DoA36417_000();
              if (global_data_A36417_000.reset_active) {
	        Reset_Faults();
	        global_data_A36417_000.control_state = STATE_SELF_TEST;
	        global_data_A36417_000.reset_active = 0;
              }
            }            
            break;

        default:
            global_data_A36417_000.control_state = STATE_STARTUP;
            break;
    }
}

void DoA36417_000(void){

  ETMCanSlaveDoCan();

//  if (global_data_A36417_000.trigger_recieved) {
//    if (global_data_A36417_000.sample_level) {
//      //ETMCanSlaveIonPumpSendTargetCurrentReading(0x2002, 0x0000, global_data_A36417_000.pulse_id);
//    } else {
//      //ETMCanSlaveIonPumpSendTargetCurrentReading(0x0000, 0x1001, global_data_A36417_000.pulse_id);
//    }
//    global_data_A36417_000.trigger_recieved = 0;
//  }
  
    //If a target pulse was received
  ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_target_current);
  //      //need to send message up...don't know how.
  //      //basically, if high
  //global_data_A36417_000.target_current_high=global_data_A36417_000.analog_input_target_current.reading_scaled_and_calibrated;
  //      //if low
  //global_data_A36417_000.target_current_high=global_data_A36417_000.analog_input_target_current.reading_scaled_and_calibrated;

  _FAULT_CAN_COMMUNICATION = ETMCanSlaveGetComFaultStatus();

  if(_T3IF){
 
    slave_board_data.log_data[2] = global_data_A36417_000.analog_input_ion_pump_voltage.reading_scaled_and_calibrated;
    slave_board_data.log_data[3] = global_data_A36417_000.analog_input_ion_pump_current.reading_scaled_and_calibrated;

    _T3IF=0;

    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_ion_pump_voltage);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_ion_pump_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_target_current);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_5V_monitor);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_15V_monitor);
    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_minus_5V_monitor);

    unsigned int ion_pump_voltage = global_data_A36417_000.analog_input_ion_pump_voltage.reading_scaled_and_calibrated;

    if (global_data_A36417_000.EMCO_enable) {
        global_data_A36417_000.EMCO_control_setpoint = (unsigned int)(UpdatePID(&emco_pid,(EMCO_SETPOINT-(double)ion_pump_voltage), (double) ion_pump_voltage));
    } else {
        global_data_A36417_000.EMCO_control_setpoint = 0;
    }

    ETMCanSlaveSetDebugRegister(8, global_data_A36417_000.EMCO_control_setpoint);
   
    // Rogue setpoint protection
    if(global_data_A36417_000.EMCO_control_setpoint>2000){
        global_data_A36417_000.EMCO_control_setpoint=2000;
    }
    ETMCanSlaveSetDebugRegister(7, global_data_A36417_000.EMCO_control_setpoint);

    global_data_A36417_000.self_test_count++;
        
//    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_ion_pump_voltage)) {
//        //Maybe go to a fault state?
//           _FAULT_ION_PUMP_OVER_VOLTAGE=1;
//           //global_data_A36417_000.EMCO_control_setpoint=0;
//    }
//    else{
//        _FAULT_ION_PUMP_OVER_VOLTAGE=0;
//    }
//
//    if (ETMAnalogCheckUnderAbsolute(&global_data_A36417_000.analog_input_ion_pump_voltage)) {
//           _FAULT_ION_PUMP_UNDER_VOLTAGE=1;
//    }
//    else{
//        _FAULT_ION_PUMP_OVER_VOLTAGE=0;
//    }

    WriteMCP4822(&U11_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36417_000.EMCO_control_setpoint);


    ETMCanSlaveSetDebugRegister(1, global_data_A36417_000.analog_input_5V_monitor.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(2, global_data_A36417_000.analog_input_15V_monitor.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(3, global_data_A36417_000.analog_input_minus_5V_monitor.reading_scaled_and_calibrated);

    ETMCanSlaveSetDebugRegister(0xF, global_data_A36417_000.control_state);


// -------------------- CHECK FOR FAULTS ------------------- //

    if (ETMCanSlaveGetSyncMsgResetEnable()) {
      global_data_A36417_000.reset_active = 1;
      _FAULT_REGISTER = 0x0000;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A36417_000.analog_input_ion_pump_voltage)) {
        _FAULT_ION_PUMP_UNDER_VOLTAGE=1;
    }else{
        _FAULT_ION_PUMP_UNDER_VOLTAGE=0;
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_ion_pump_voltage)) {
        _FAULT_ION_PUMP_OVER_VOLTAGE=1;
    }else{
        _FAULT_ION_PUMP_OVER_VOLTAGE=0;
    }


    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_ion_pump_current)) {
        _FAULT_ION_PUMP_OVER_CURRENT=1;
    }else{
        _FAULT_ION_PUMP_OVER_CURRENT=0;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A36417_000.analog_input_5V_monitor)) {
      _FAULT_5V_UV = 1;
    }else{
      _FAULT_5V_UV = 0;
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_5V_monitor)) {
      _FAULT_5V_OV = 1;
    }else{
      _FAULT_5V_OV = 0;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A36417_000.analog_input_15V_monitor)) {
      _FAULT_15V_UV = 1;
    }else{
      _FAULT_15V_UV = 0;
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_15V_monitor)) {
      _FAULT_15V_OV = 1;
    }else{
      _FAULT_15V_OV = 0;
    }

    if (ETMAnalogCheckUnderAbsolute(&global_data_A36417_000.analog_input_minus_5V_monitor)) {
      _FAULT_MINUS_5V_UV = 1;
    }else{
      _FAULT_MINUS_5V_UV = 0;
    }

    if (ETMAnalogCheckOverAbsolute(&global_data_A36417_000.analog_input_minus_5V_monitor)) {
      _FAULT_MINUS_5V_OV = 1;
    }else{
      _FAULT_MINUS_5V_OV = 0;
    }

  }
  return;
}

unsigned int Check_UV_OC_Fault(void) {
    unsigned int fault;

    fault  = _FAULT_ION_PUMP_OVER_CURRENT;
    fault |= _FAULT_ION_PUMP_UNDER_VOLTAGE;

    return fault;
}

unsigned int Check_Supplies(void) {
    unsigned int fault;
    fault  = _FAULT_5V_UV;
    fault |= _FAULT_5V_OV;
    fault |= _FAULT_15V_UV;
    fault |= _FAULT_15V_OV;
    fault |= _FAULT_MINUS_5V_UV;
    fault |= _FAULT_MINUS_5V_OV;

    return fault;
}

void Reset_Faults(void) {
    _FAULT_ION_PUMP_OVER_CURRENT = 0;
    _FAULT_ION_PUMP_OVER_VOLTAGE = 0;
    _FAULT_ION_PUMP_UNDER_VOLTAGE = 0;
    _FAULT_CAN_COMMUNICATION = 0;
    _FAULT_5V_UV = 0;
    _FAULT_5V_OV = 0;
    _FAULT_15V_UV = 0;
    _FAULT_15V_OV = 0;
    _FAULT_MINUS_5V_UV = 0;
    _FAULT_MINUS_5V_OV = 0;

}

double UpdatePID(SPid* pid, double error, double reading){
  double pTerm, dTerm, iTerm;
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

  ETMCanSlaveSetDebugRegister(9, (unsigned int)error);
  ETMCanSlaveSetDebugRegister(0xA, (unsigned int)pTerm);
  ETMCanSlaveSetDebugRegister(0xB, (unsigned int)iTerm);
  ETMCanSlaveSetDebugRegister(0xC, (unsigned int)dTerm);


  if(pTerm + iTerm - dTerm < 0)
    return 1;
  else
    return pTerm + iTerm - dTerm;

}

//void SelfTestA36417(void){
//
//  if(_T3IF){
//    _T3IF=0;
//
//            //Check 5V, -5V, 15V monitors
//    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_5V_monitor);
//    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_15V_monitor);
//    ETMAnalogScaleCalibrateADCReading(&global_data_A36417_000.analog_input_minus_5V_monitor);
//
//    //unsigned int _5Vmonitor=global_data_A36417_000.analog_input_5V_monitor.reading_scaled_and_calibrated;
//    //unsigned int _15Vmonitor=global_data_A36417_000.analog_input_15V_monitor.reading_scaled_and_calibrated;
//    //unsigned int minus_5Vmonitor=global_data_A36417_000.analog_input_minus_5V_monitor.reading_scaled_and_calibrated;
//
//            if(_5Vmonitor>2400&&_5Vmonitor<2600){
//                if(_15Vmonitor>2400&&_15Vmonitor<2600){
//                    if(minus_5Vmonitor>1570&&minus_5Vmonitor<1770){
//                        control_state=STATE_OPERATE;
//                        return;
//                    }
//                }
//            }            
//            if(test_count>SELF_TEST_FAIL_COUNT){
//                //Set Board self check fail bit.
//                _CONTROL_SELF_CHECK_ERROR=1;
//            }
//  }
//}

void InitializeA36417(void){

  unsigned int startup_counter;

  // Initialize the status register and load the inhibit and fault masks
  _FAULT_REGISTER = 0;
  _CONTROL_REGISTER = 0;
  _WARNING_REGISTER = 0;
  _NOT_LOGGED_REGISTER = 0;


  // Configure Sample Target Current Interrupt
  _INT3IP = 7; // This must be the highest priority interrupt
  _INT3EP = 0; // Positive Transition COSORIO check this
  _INT3IF = 0; //Clear the interrupt flag.
  _INT3IE = 1;


  // Configure ADC Interrupt
  _ADIP = 6; // This needs to be higher priority than the CAN interrupt (Which defaults to 4)

  // Initialize all I/O Registers
  TRISA = A36417_TRISA_VALUE;
  TRISB = A36417_TRISB_VALUE;
  TRISC = A36417_TRISC_VALUE;
  TRISD = A36417_TRISD_VALUE;
  TRISF = A36417_TRISF_VALUE;
  TRISG = A36417_TRISG_VALUE;
  
  target_current = 0;
  target_current_flag = 0;
  

  U11_MCP4822.pin_chip_select_not = _PIN_RF2;
  U11_MCP4822.pin_load_dac_not = _PIN_RF3;
  U11_MCP4822.spi_port = ETM_SPI_PORT_1;
  U11_MCP4822.spi_con1_value = MCP4822_SPI_CON_VALUE;
  U11_MCP4822.spi_con2_value = MCP4822_SPI_CON2_VALUE;
  U11_MCP4822.spi_stat_value = MCP4822_SPI_STAT_VALUE;
  U11_MCP4822.spi_bit_rate = MCP4822_SPI_1_M_BIT;
  U11_MCP4822.fcy_clk = FCY_CLK;

  SetupMCP4822(&U11_MCP4822);

     // Initialize the External EEprom
  ETMEEPromConfigureExternalDevice(EEPROM_SIZE_8K_BYTES, FCY_CLK, 400000, EEPROM_I2C_ADDRESS_0, 1);

//  _CONTROL_SELF_CHECK_ERROR = 0;

 
  // Initialize TMR3
  T3CON = T3CON_VALUE;
  TMR3  = 0;
  _T3IF = 0;
  PR3   = PR3_VALUE_10_MILLISECONDS;

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

  global_data_A36417_000.EMCO_control_setpoint=0;
  WriteMCP4822(&U11_MCP4822, MCP4822_OUTPUT_A_4096, global_data_A36417_000.EMCO_control_setpoint);


     // Initialize the CAN module

  ETMCanSlaveInitialize(CAN_PORT_1, FCY_CLK, ETM_CAN_ADDR_ION_PUMP_BOARD, _PIN_RG13, 4, _PIN_RA7, _PIN_RG12);
  ETMCanSlaveLoadConfiguration(36417, 000, FIRMWARE_AGILE_REV, FIRMWARE_BRANCH, FIRMWARE_MINOR_REV);


  //Initialize analog input/output scaling

  ETMAnalogInitializeInput(&global_data_A36417_000.analog_input_ion_pump_voltage,
                           MACRO_DEC_TO_SCALE_FACTOR_16(ION_PUMP_VOLTAGE_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_0,
                           ION_PUMP_VOLTAGE_OVER_TRIP_POINT,
            	           ION_PUMP_VOLTAGE_UNDER_TRIP_POINT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ION_PUMP_VOLTAGE_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36417_000.analog_input_ion_pump_current,
                           MACRO_DEC_TO_SCALE_FACTOR_16(ION_PUMP_CURRENT_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_1,
                           ION_PUMP_CURRENT_OVER_TRIP_POINT,
            	           ION_PUMP_CURRENT_UNDER_TRIP_POINT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   ION_PUMP_CURRENT_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36417_000.analog_input_target_current,
                           MACRO_DEC_TO_SCALE_FACTOR_16(TARGET_CURRENT_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_2,
                           TARGET_CURRENT_OVER_TRIP_POINT,
            	           TARGET_CURRENT_UNDER_TRIP_POINT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_RELATIVE_COUNTER,
			   TARGET_CURRENT_ABSOLUTE_TRIP_TIME);

  ETMAnalogInitializeInput(&global_data_A36417_000.analog_input_5V_monitor,
                           MACRO_DEC_TO_SCALE_FACTOR_16(_5V_MONITOR_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_3,
                           _5V_MONITOR_OVER_TRIP_POINT,
            	           _5V_MONITOR_UNDER_TRIP_POINT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_COUNTER,
			   NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36417_000.analog_input_15V_monitor,
                           MACRO_DEC_TO_SCALE_FACTOR_16(_15V_MONITOR_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_4,
                           _15V_MONITOR_OVER_TRIP_POINT,
            	           _15V_MONITOR_UNDER_TRIP_POINT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_COUNTER,
			   NO_COUNTER);

  ETMAnalogInitializeInput(&global_data_A36417_000.analog_input_minus_5V_monitor,
                           MACRO_DEC_TO_SCALE_FACTOR_16(MINUS_5V_MONITOR_SCALE_FACTOR),
                           OFFSET_ZERO,
                           ANALOG_INPUT_5,
                           MINUS_5V_MONITOR_OVER_TRIP_POINT,
            	           MINUS_5V_MONITOR_UNDER_TRIP_POINT,
			   NO_TRIP_SCALE,
			   NO_FLOOR,
			   NO_COUNTER,
			   NO_COUNTER);

  ETMAnalogInitializeOutput(&global_data_A36417_000.analog_output_emco_control,
			    MACRO_DEC_TO_SCALE_FACTOR_16(EMCO_CTRL_VOLTAGE_SCALE_FACTOR),
			    OFFSET_ZERO,
			    ANALOG_OUTPUT_0,
			    EMCO_MAX_CTRL_VOLTAGE,
			    EMCO_MIN_CTRL_VOLTAGE,
			    0);

// Startup LEDs
  startup_counter = 0;
  while (startup_counter <= 400) {  // 4 Seconds total
    ETMCanSlaveDoCan();
    if (_T3IF) {
      _T3IF =0;
      startup_counter++;
    }
    switch (((startup_counter >> 4) & 0b11)) {

    case 0:
      PIN_LED_OPERATIONAL_GREEN = !OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;

    case 1:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = !OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;

    case 2:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = !OLL_LED_ON;
      break;

    case 3:
      PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
      PIN_LED_A_RED = OLL_LED_ON;
      PIN_LED_B_GREEN = OLL_LED_ON;
      break;
    }
  }

  PIN_LED_OPERATIONAL_GREEN = OLL_LED_ON;
}


void __attribute__((interrupt, no_auto_psv)) _INT3Interrupt(void){
  _INT3IF = 0;
  global_data_A36417_000.trigger_recieved = 1;
  global_data_A36417_000.pulse_id = ETMCanSlaveGetPulseCount();
  global_data_A36417_000.sample_level = ETMCanSlaveGetPulseLevel();

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

    ETMCanSlaveSetDebugRegister(4, global_data_A36417_000.analog_input_ion_pump_voltage.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(5, global_data_A36417_000.analog_input_ion_pump_current.reading_scaled_and_calibrated);
    ETMCanSlaveSetDebugRegister(6, global_data_A36417_000.analog_input_target_current.filtered_adc_reading);
 
}




void ETMCanSlaveExecuteCMDBoardSpecific(ETMCanMessage* message_ptr) {
  unsigned int index_word;
 
  index_word = message_ptr->word3;
  switch (index_word)
    {
      /*
	Place all board specific commands here
      */
     
    default:
      //local_can_errors.invalid_index++;
      break;
    }
}
