// $Id: ref_perf_eval_iguana_dingo.c,v 1.1 2007/10/11 20:27:45 walton Exp $
/******************************************************************************
**                 Proprietary and Confidential Information                  **
**                                                                           **
**                 Copyright (c) Conexant Systems Unpublished                  **
**                                                                           **
** This source code and the algorithms implemented therein constitute        **
** confidential information and may compromise trade secrets of Conexant,    **
** Inc. or its associates, and any unauthorized use thereof is prohibited.   **
**                                                                           **
******************************************************************************/
/*****************************************************************************
This application will demonstrate using the LM_Snoop registers to monitor motor
position by monitoring the GPIO where motor pulses are delivered. Each pulse is
counted and interrtupts are generated off of specific counts. In this example
after initialization the motor counts are monitored until the ramp pulses have been
deliverd to the GPIO. At this time an interrupt is generated. The state
machine is incremented to the motor running in steady state. In this mode an
interrupt is generated every two motor pulses. The cumulative pulses are also counted.
When the motor has run long enough to have finished printing a simulated page,
the state table is updated and the motor ramps down and then stops. The motor
pulses can be monitored with a scope on GPIO N_3. The motor pulses begin when
the user enters:
>wr ac-scratch0 1
at a dbgmon prompt after loading the application. This can be done multiple times.
There is a second interrupt running in the system which provides value/delay pairs
to the stepper motor motor control block. In a product, it may be better to set the
interrupt threshold to a non zero number to insure that there is always valid data in
the motor control block. In this example the MC block interrupt does not occur until
the buffer volume is zero. The motor is running slow enough, and there are not many
tasks running such that the buffer is replenished prior to the next value/delay pair
begin required. In an actual system, some tolerances should be factored in.

*****************************************************************************/

#include "string.h"
#include "stdlib.h"

#include "tx_api.h"

#include "dc_type.h"
#include "dc_return_codes.h"
#include "dc_rammgr_api.h"
#include "dc_log_api.h"
#include "dc_dbgmon_api.h"
#include "dc_nvram_api.h"
#include "dc_time_api.h"
#include "dc_mi_api.h"
#include "ra_mi.h"
#include "dc_int_api.h"
#include "dc_sys_info_api.h"

#include "radef.h"
#include "ra_ip.h"
#include "ra_jd.h"
#include "ra_je.h"
#include "ra_pi.h"
#include "ra_po.h"
#include "ra_ro.h"
#include "ra_ck.h"
#include "ra_ac.h"
#include "ra_ms.h"
#include "ra_lm.h"
#include "ra_gp.h"
#include "ra_mc.h"
#include "ra_gp.h"
#include "ra_ic.h"

#include "./Generic/dc_mc_api.h"


#include "dc_container_api.h"
#include "dc_imgbuf_api.h"

#include "dc_mc_api.h"

#include "ds_common.h"


void setMotorLoop(UINT8 val);	// { gMotorInfinite = val; }

//Debug Level
#define MOTOR_DEBUG	DS_DEBUG_MOTOR
#define DC_LOG_PRINTF(fmt, ...) DS_DBGPRINTF(MOTOR_DEBUG, fmt, ##__VA_ARGS__)

#define PSENS_LEDCHK() \
  if ((dcreg(RA_GP_OUTPUT0_D) & 8 ) == 8) { \
     dc_log_printf(DC_LOG_ERROR "Psens LED is OFF"); \
     dc_log_printf(DC_LOG_CRITICAL " file %s",__FILE__); \
     dc_log_printf(DC_LOG_CRITICAL " line %d",__LINE__); \
     dc_log_printf(DC_LOG_CRITICAL " function %s",__func__); \
  }

extern UINT16 ds_GetPsensThreshold(UINT32 high);

#ifdef RETRY_MISSFEED
extern void retryFeed(UINT16 ref_period, UINT32 mode);
#endif

char * tray_string[] =
   {
      "tray_idle",
      "tray_SmartSheet",
      "tray_Paper",
      "tray_Empty",
      "tray_Jam",
      "tray_MisFeed",
      "tray_MisMatch??",
      "NA",
   };

   /* FIXME - Keith, why different start printing positions for MCal vs User? */
#define PAPER_POSITION_START_PRINTING_MCAL 1132
#define PAPER_POSITION_START_PRINTING_USER 1147


/******************************************************************************
*  FILE FORMAT: <PUBLIC>
*      1. INCLUDES
*      2. ENUMERATIONS
*      3. DEFINES
*      4. STRUCTURES
*      5. GLOBALS
*      6. PUBLIC FUNCTION PROTOTYPES
******************************************************************************/
/******************************************************************************
*  1. INCLUDES
******************************************************************************/
#include "snoop.h"

/******************************************************************************
*  2. ENUMERATIONS

******************************************************************************/
/******************************************************************************
*  3. DEFINES
******************************************************************************/

#define STACK_BYTES   32000

#define RUN_INTERRUPT_INTERVAL 127
/* pz - FIXME derive this */
#define RAMP_DELAY_MAX 255
#define RAMP_DELAY_TARGET (RAMP_DELAY_MAX/(3/(t_IPS*10)))

#define POSITION_TIMEOUT 30000 /* Keith - the unit is msec. Need to be adjust for graceful feeding */
#define POSITION_SLEEP_INT 4   /* hack on Keith's hack ... interval for checking PsensA */
#define SAMPLE_NUMBER 100 /* The number of samples to calculate the average of PSensA */


#define NUM_BARCODE_SAMPLES 20000

/******************************************************************************
*  4. STRUCTURES
******************************************************************************/

/******************************************************************************
*  5. GLOBALS
******************************************************************************/
//NEVER_REFER	static TX_THREAD int_test_thr;
static TX_THREAD app_test_thr;
UINT16 motor_period_count = 0;
UINT8 pPrintProcessFlag = 0;	// 0: IDLE , 1: Printing
#ifdef RETRY_MISSFEED
UINT8 Retry_MissFeeding = 0;
#endif
UINT8 FeedErrorFlag = 0;
UINT8 PaperJamFlag = 0;
UINT8 EngineInitStallFlag = 0;

TX_EVENT_FLAGS_GROUP motor_flags;

extern UINT8 gReturnCode;
extern UINT16 gMotorStepChange;
/******************************************************************************
*  6. PUBLIC FUNCTION PROTOTYPES
******************************************************************************/
void do_serial(int primeOnly, UINT32 addr);
void pi_setup ( UINT32 addr );

//NEVER_REFER	static UINT32 int_test_stack[1000];
static UINT32 app_test_stack[1000];
static volatile UINT32 ramp_cycles, ramp_cycles_remaining;
static volatile UINT32 run_cycles, run_cycles_initial;
static UINT32 end_of_ramp;

DC_RETURN_CODE init_motor(STEPPER_CONFIG *motor_config, MOTOR_DRIVER_CONFIG *mdc);

// define a motor ramping table
#ifdef MOTOR_PHASE_1_2
#define RAMP_COUNT 2048
#else
#define RAMP_COUNT 1024
#endif
//Motor Control Tables and *'s
static UINT8 mtr1_ramp[RAMP_COUNT];
static UINT8 mtr1_down_ramp[RAMP_COUNT];
static UINT8 mtr1_steady_state[MCx_WRITE_SIZE];

static volatile UINT8 *pMCT_RampUp;
static volatile UINT8 *pMCT_Steady;
static volatile UINT8 *pMCT_RampDown;

static volatile STEPPER_CONFIG *pStepper_config;
static volatile MOTOR_DRIVER_CONFIG *pmdc;

int GetScanState(void);
void SetMotorStop(void);

extern UINT16 read_ADC(UINT8 channel);
extern void task_test_set_flag(void) ;

static volatile int mtrState = MTR_STATE_IDLE;

static volatile int numBarCodeSamples = 0;
static volatile int flagBarcodeActive = 0;
UINT16 samplesBarCode[NUM_BARCODE_SAMPLES];

void ref_motor_setup(void);

//NEVER_REFER	static UINT32 gLMIrqCount=0;//bjke
static void LM_MOTOR0_isr(void);

// callback fn* for when move completes or is interrupted with another
// call to stepperStart()
static void (*callback)(int steps_remaining) = NULL;

typedef struct _STEPPER_CONTROL_BLOCK {
	int ipsStart;
	int ipsRun;
	// c/b*
	int stepsNum;
} STEPPER_CONTROL_BLOCK, *pSTEPPER_CONTROL_BLOCK;

//Simple Motor Test
#ifdef SIMPLE_MOTOR_EN
UINT16 getRefPeriod(BOOL bRecalculate)
{
    UINT32 main_hz = 0;
    UINT16 ref_period = 0;

    //if (ref_period == 0 || bRecalculate == TRUE)
    {
        RESULT_CHK(dc_clock_freq( &main_hz, RA_CK_MAIN_LO));
        ref_period = (main_hz/(TARGET_PPS*RAMP_DELAY_TARGET));

        ds_log_printf("MMPS=%.3f, PRD=%.3f MGD=%.3f PPS=%.3f ref_period=%d",
            TARGET_MMPS, TARGET_PRD, TARGET_MGD, TARGET_PPS, (UINT16)ref_period);
    }

    return (ref_period);
}

INT32 getMotorState(void)
{
   return (mtrState);
}

void setMotorState(INT32 val)
{
   mtrState = val;
}

/* It will be set  default runcycle after when an event of eject or smartSheent has happend. */
void setRunCycleAsInitial(void)
{
   run_cycles = run_cycles_initial;
   ds_log_printf("%s run_cycles=%d", __func__, run_cycles);
}
#endif /* #ifdef SIMPLE_MOTOR_EN */

/* LM_MOTOR0 monitors the drive pin as if it were an encoder */
static void LM_MOTOR0_isr(void)
{
   //   UINT16 headTemp;
   //   UINT16 psenA_volts;
   UINT32 t;

   //dc_log_printf("LM int Pos=%d, sensor = %x", dcreg16(RA_LM_POSITION0_LO), readCurrentADC(PSENB_CHANNEL));  /* counts steps? */
   //dc_log_printf("LM int");
   //dc_log_printf("ramp : %d",ramp_cycles_remaining);

   if (mtrState == MTR_STATE_RAMP_UP)
   {
      dc_log_printf("lm ramp : %d",ramp_cycles_remaining);
      // done with ramp??
      if (ramp_cycles_remaining == 0)
      {
         dcreg(RA_LM_INTDISTANCE0) = RUN_INTERRUPT_INTERVAL;     // Every RUN_INTERRUPT_INTERVAL (default 127) counts an interrupt will be generated
         mtrState = MTR_STATE_RUNNING;
         //DC_LOG_PRINTF("end ramp up - start steady state");
         dc_log_printf("Ramp->Stdy - %d", run_cycles);
      }
      else if (ramp_cycles_remaining <= LM_DISTANCE_MAX_VALUE)
      {
         dcreg(RA_LM_INTDISTANCE0) = ramp_cycles_remaining;
         ramp_cycles_remaining = 0;
      }
      else
      {
         dcreg(RA_LM_INTDISTANCE0) = LM_DISTANCE_MAX_VALUE;     // after 127 counts an interrupt will be generated. This is the max value
         ramp_cycles_remaining -= LM_DISTANCE_MAX_VALUE;
      }
   }

   else if (mtrState == MTR_STATE_RUNNING)
   {
#ifdef BAR_ISR_EN
      if (gIsBarSection == TRUE && dcreg(RA_LM_INTDISTANCE0) == BARCODE_READ_INTERVAL )
      {
         if (gBarCount < BARCODE_BUFF_SZ)
         {
            if (gBarBuf != NULL)
               gBarBuf[gBarCount] = readADCfromISR(PSENB_CHANNEL);

            //dc_log_printf("gBarBuf[%d] %d", gBarCount, gBarBuf[gBarCount]);

            gBarCount++;
         }
         else
            gIsBarSection = FALSE;
      }
#endif
      if ( (flagBarcodeActive) && (dcreg(RA_LM_INTDISTANCE0) == BARCODE_READ_INTERVAL) ) {
        if (numBarCodeSamples < NUM_BARCODE_SAMPLES) {
          samplesBarCode[numBarCodeSamples++] = readADCfromISR(PSENB_CHANNEL);
        } else {
          flagBarcodeActive = 0;
        }
      }
      
      if (run_cycles == 0)
      {
         //dc_log_printf("Stdy->RampDown: %d", run_cycles);
         // dcreg(RA_LM_INTENABLE0) = 0x00; // disable interrupts
         DC_LOG_PRINTF("ramp mtr down - start %d", ramp_cycles);
         mtrState = MTR_STATE_RAMP_DOWN;
         ramp_cycles_remaining = ramp_cycles;
         if (ramp_cycles <= LM_DISTANCE_MAX_VALUE)
         {
            dcreg(RA_LM_INTDISTANCE0) = ramp_cycles;     // after ramp_cycles counts an interrupt will be generated
            ramp_cycles_remaining = 0;
         }
         else
         {
            dcreg(RA_LM_INTDISTANCE0) = LM_DISTANCE_MAX_VALUE;     // after 127 counts an interrupt will be generated. This is the max value
            ramp_cycles_remaining -= LM_DISTANCE_MAX_VALUE;
         }
      }
      else
      {
         if (run_cycles >= RUN_INTERRUPT_INTERVAL)
         {
            //            gLMIrqCount++;
           if ( (gIsBarSection == TRUE) || (flagBarcodeActive) )
               t = BARCODE_READ_INTERVAL; //bjke Every 1 step in reading barcode mode
            else
               t = RUN_INTERRUPT_INTERVAL; // Every RUN_INTERRUPT_INTERVAL (default 127) counts an interrupt will be generated

            dcreg(RA_LM_INTDISTANCE0) = t;

            if (gMotorInfinite == 0 && gIsBarSection == 0)
               run_cycles -= t; //RUN_INTERRUPT_INTERVAL;

            //ds_log_printf(">=127, run_cycles=%d, MOT_nFAULT=%d", run_cycles, ds_MOT_nFAULT_RD());
         }
         else
         {
            dcreg(RA_LM_INTDISTANCE0) = run_cycles;
            run_cycles = 0;
            ds_log_printf("<127, run_cycles=%d, MOT_nFAULT=%d", run_cycles, ds_MOT_nFAULT_RD());
         }
      }

   } else if (mtrState == MTR_STATE_RAMP_DOWN) {
       if (ramp_cycles_remaining <= 0) {
           mtrState = MTR_STATE_STOP;
       } else if (ramp_cycles_remaining <= LM_DISTANCE_MAX_VALUE) {
           dcreg(RA_LM_INTDISTANCE0) = ramp_cycles_remaining;
           ramp_cycles_remaining = 0;
       } else {
           dcreg(RA_LM_INTDISTANCE0) = LM_DISTANCE_MAX_VALUE;
           ramp_cycles_remaining -= LM_DISTANCE_MAX_VALUE;
       }
   }

   dcreg(RA_LM_INTFLAG0_CLEAR) = 0x01; // clear the position interrupt

}

static void MC0_isr(void)
{
   static int interrupt_count = 1;
   int loopCnt;
   static int index;
   static UINT8 mtr1_check_speed = 0;
   static UINT32 mtr1_check_period_count = 0;

   static int lastMtrState = 0xff;

   //dc_log_printf("interrupt cnt : %d", interrupt_count);
   //dc_log_printf("index : %d", index);
   if (lastMtrState != mtrState) {
       dc_log_printf("MC int State = %d", mtrState);
   }

   /* pz  FIXME - does this run the initial 64 twice?  see indexing ...  :( */
   if (mtrState == MTR_STATE_RAMP_UP)
   {
      if (lastMtrState != MTR_STATE_RAMP_UP) {
        interrupt_count = 1;
        dcreg(RA_LM_INTDISTANCE0) = 0x30;
      }
      //DC_LOG_PRINTF("interrupt count in ramp = %d", interrupt_count);
      dc_log_printf("irq count in ramp = %d", interrupt_count);
      //dc_log_printf("ramp cyc rem: %d", ramp_cycles_remaining);
      //dc_log_printf("MC LM pos:%#x", dcreg16(RA_LM_POSITION0_LO) );
      //dc_log_printf("MC LM curd:%#x,%#x", dcreg(RA_LM_CURDISTANCE0), dcreg(RA_LM_INTDISTANCE0) );

      for (index = interrupt_count * MCx_WRITE_SIZE; index < interrupt_count * MCx_WRITE_SIZE + MCx_WRITE_SIZE; index++)
      {
         dcreg(RA_MC0_WRITE) = pMCT_RampUp[index];
      }
      interrupt_count++;
      dc_int_flag_clear(DC_INT_MC0_BUFFER);
   }

   else if (mtrState == MTR_STATE_RUNNING)
   {
      /* pz FIXME - only need to load once - first time in or at end of RAMP UP? */
      for (loopCnt = 0; loopCnt < MCx_WRITE_SIZE; loopCnt++)
      {
         dcreg(RA_MC0_WRITE) = pMCT_Steady[loopCnt];   // toggle high
      }
      dc_int_flag_clear(DC_INT_MC0_BUFFER);
      if (mtr1_check_speed != mtr1_steady_state[1])
      {
              //dc_log_printf("MC0speedchange %d %d",mtr1_check_speed,mtr1_steady_state[1]);
              mtr1_check_speed = mtr1_steady_state[1];
      }
      if (dcreg32(RA_MC0_PERIOD_LO) != mtr1_check_period_count )
      {
              //dc_log_printf("MC0periodchange %d %d",mtr1_check_period_count,dcreg32(RA_MC0_PERIOD_LO));
              mtr1_check_period_count = dcreg32(RA_MC0_PERIOD_LO);
      }
   }
   else if (mtrState == MTR_STATE_RAMP_DOWN)  /* PZ - DSG is not using ... just stopping - good thing . */
   {
      int offset = 0;
      
      if (lastMtrState == MTR_STATE_RUNNING) {
        interrupt_count = 0;
      }
      dc_log_printf("ramp down %d", interrupt_count);
      
      offset = interrupt_count++ * MCx_WRITE_SIZE;
      
      // pz - FIX shut off the head voltage
      if (tph_power_is_on())
      {
         tph_power(0);
      }
      for (index = offset; index < offset+MCx_WRITE_SIZE; index++) {
        dcreg(RA_MC0_WRITE) = pMCT_RampDown[index];
      }

      dc_int_flag_clear(DC_INT_MC0_BUFFER);
   }
   else if (mtrState == MTR_STATE_STOP)
   {
      dc_log_printf("mtr stop");
      for ( loopCnt = 0; loopCnt < MCx_WRITE_SIZE/2; loopCnt++)
      {
         dcreg(RA_MC0_WRITE) = mtr1_ramp[index ];
         dcreg(RA_MC0_WRITE) = mtr1_ramp[index + 1];
         index += 2;
      }
      mtrState = MTR_STATE_IDLE;
      dc_int_flag_clear(DC_INT_MC0_BUFFER);

      if (NULL != callback) {
        (*callback)(0);// 0 == move completed
        callback = NULL;
      }
   }
   else if (mtrState == MTR_STATE_IDLE)
   {
      DC_LOG_PRINTF("disable mtr int");
      interrupt_count = 0;
      //mtrState = MTR_STATE_RAMP_UP;
      dcreg(RA_MC0_CONTROL) = 0x00;     // stop sending pulses to the motor
      dc_log_printf("mtr stopped");
      dcreg16(RA_LM_POS0UPDATE_LO) = 0;   // reset the counter
      dcreg(RA_LM_INTENABLE0) = 0x00; // Disable counting pulses
      dc_int_disable(DC_INT_MC0_BUFFER);
      dc_int_flag_clear(DC_INT_MC0_BUFFER);
      DC_LOG_PRINTF("LM Pos reset =%d", dcreg16(RA_LM_POSITION0_LO));  /* counts steps? */
      // pz - disable motor output FIXME
      dcreg(RA_GP_OUTPUT0_B) = 0x00;

      //re-initialize the motor for the next run
      for (index = interrupt_count * MCx_WRITE_SIZE; index < interrupt_count * MCx_WRITE_SIZE + MCx_WRITE_SIZE; index++)
      {
         dcreg(RA_MC0_WRITE) = mtr1_ramp[index];
      }
      interrupt_count++;
   }
   else
   {
      interrupt_count = 0;
      dcreg(RA_MC0_CONTROL) = 0x00;     // stop sending pulses to the motor
      dcreg(RA_GP_OUTPUT0_B) = 0x00;
      mtrState = MTR_STATE_RAMP_UP;
      dcreg(RA_LM_INTENABLE0) = 0x00; // Disable counting pulses
      dc_log_printf("UNKNOWN MTR STATE");
      dc_int_disable(DC_INT_MC0_BUFFER);
   }

   lastMtrState = mtrState;

   //dc_log_printf("MC0 int");
}// MC0_isr

void init_motor_config(STEPPER_CONFIG *pConfig)
{
    // initialize the motor configuration parameters
    pConfig->mc_ioaddr = RA_GP_OUTPUT0_B;

    //800 PPS is Pull In Torque, 탈조가 발생하지 않는 최대한 안정적인 PPS
#ifdef MOTOR_PHASE_1_2
    pConfig->steps_in_ramp = RAMP_DELAY_MAX*0.8;
#else
    pConfig->steps_in_ramp = 96/2;
#endif
    pConfig->direction = FORWARD;
    pConfig->enable = POSITIVE;
    pConfig->enable_delay = 30;  // time in msec
	
	pStepper_config = pConfig;
}

void init_motor_driver_config(MOTOR_DRIVER_CONFIG *pConfig)
{
    pConfig->type = DRIVER_DRV8833;//DRIVER_LB1935;

    pConfig->direction = gMotorDirection;
    motor_period_count = gMotorVelocity;

    dc_log_printf("[MTR] %s %0u", (pConfig->direction == FORWARD) ? "FORWARD" : "BACKWARD", motor_period_count);

    if(pConfig->type == DRIVER_LB1935)
    {
        if (pConfig->direction == FORWARD)
        {
            pConfig->phase[0] = 0;
            pConfig->phase[1] = 1;
            pConfig->phase[2] = 3;
            pConfig->phase[3] = 2;
        }
        else if (pConfig->direction == BACKWARD)
        {
            pConfig->phase[0] = 0;
            pConfig->phase[1] = 2;
            pConfig->phase[2] = 3;
            pConfig->phase[3] = 1;
        }
        pConfig->enable = 0xc;
    }
    else if(pConfig->type == DRIVER_DRV8833)
    {
#ifdef MOTOR_PHASE_1_2
        if (pConfig->direction == FORWARD)
        {
            pConfig->phase[0] = 0x5;
            pConfig->phase[1] = 0x7;
            pConfig->phase[2] = 0x9;
            pConfig->phase[3] = 0xE;
            pConfig->phase[4] = 0xA;
            pConfig->phase[5] = 0xB;
            pConfig->phase[6] = 0x6;
            pConfig->phase[7] = 0xD;
        }
        else if (pConfig->direction == BACKWARD)
        {
            pConfig->phase[0] = 0xD;
            pConfig->phase[1] = 0x6;
            pConfig->phase[2] = 0xB;
            pConfig->phase[3] = 0xA;
            pConfig->phase[4] = 0xE;
            pConfig->phase[5] = 0x9;
            pConfig->phase[6] = 0x7;
            pConfig->phase[7] = 0x5;
        }
#else // MOTOR_PHASE_2_2
        if (pConfig->direction == FORWARD)
        {
            pConfig->phase[0] = 0x5;
            pConfig->phase[1] = 0x9;
            pConfig->phase[2] = 0xa;
            pConfig->phase[3] = 0x6;
        }
        else if (pConfig->direction == BACKWARD)
        {
            pConfig->phase[0] = 0x5;
            pConfig->phase[1] = 0x6;
            pConfig->phase[2] = 0xa;
            pConfig->phase[3] = 0x9;
        }
#endif
        pConfig->enable = 0x0;
     }
	 
	 pmdc = pConfig;
}

void motor_init_and_start(void)
{
    STEPPER_CONFIG motor_config;
    MOTOR_DRIVER_CONFIG mdc;

    init_motor_config(&motor_config);
    init_motor_driver_config(&mdc);
    RESULT_RPT(init_motor(&motor_config, &mdc));

    dc_log_printf("GOT MOTOR START");
    do_serial(0, motor_config.mc_ioaddr);
}

static TX_TIMER ips_control_timer;
#define IPS_CONTROL_TIME (int)(TIME_FOR_ONE_LINE*DUMMY_TOP_HEIGHT + 1 + 500)
static void ips_control_event(ULONG id)
{
    UINT16 print_period;

    ds_log_printf("%s %d", __func__, IPS_CONTROL_TIME);

#ifdef MOTOR_PHASE_1_2
    print_period = getRefPeriod(FALSE)-(UINT16)(0.32/(t_IPS*t_IPS));
#else
    print_period = getRefPeriod(FALSE)-(UINT16)(0.85/(t_IPS*t_IPS));
#endif

    if (gMotorStepChange != NULL)
        setMotorVelOnTheFly(gMotorStepChange, 1);
    else
        setMotorVelOnTheFly(print_period, 1);

    RESULT_DIE(tx_timer_change(&ips_control_timer, IPS_CONTROL_TIME, 0));
}

UINT16 move2PSensor(UINT8 channel)
{
    UINT32 startAverage = 0;
    UINT16 current = 0, motorPosition = 0;
    UINT16 diff = 0, reach_count = 0;

    while (motorPosition < SAMPLE_NUMBER)
    {
        tx_thread_sleep(POSITION_SLEEP_INT);
        current = ADC_readCurrentAvg(channel, 10);
        startAverage += current;
        motorPosition++;
    }
    startAverage = startAverage / SAMPLE_NUMBER;

    if (channel == PSENA_CHANNEL)
        diff = 2 * ((65535 - startAverage) / 1000.0) * ((65535 - startAverage) / 1000.0);
    else if (channel == PSENB_CHANNEL)
        diff = ((65535 - startAverage) / 100.0) * ((65535 - startAverage) / 100.0);
    else
        dc_log_printf("%s invalid channel", __func__);

    dc_log_printf("startAverage %d, diff %d", startAverage, diff);

    while ((reach_count < SAMPLE_NUMBER) && (motorPosition < POSITION_TIMEOUT / POSITION_SLEEP_INT))
    {
        tx_thread_sleep(POSITION_SLEEP_INT);
        current = ADC_readCurrentAvg(channel, 10);
        if (abs(startAverage - current) > diff)
            reach_count++;
        motorPosition++;
    }
    dc_log_printf("!!PSEN[%d] = %d, pos = %d %d", channel, current, motorPosition, dcreg16(RA_LM_POSITION0_LO));

    return motorPosition;
}

#define FEED_SPEED_MULTIPLE 0.5
DC_RETURN_CODE motor_feeding(void)
{
    DC_RETURN_CODE result = DC_SUCCESS;
    UINT16 motorPosition = 0, motorPosition_printing = 0;
    float psenA_TPH_distance = 0.0, psenA_psenB_distance = 0.0;

    dc_log_printf("* STARTTT MOTORRR *");

    setMotorVelOnTheFly(FEED_SPEED_MULTIPLE*getRefPeriod(FALSE), 1);

    if (ADC_readCurrentAvg(PSENA_CHANNEL, 10) > PSENA_NO_PAPER_THRESHOLD)
    {
        motorPosition = move2PSensor(PSENA_CHANNEL);
    }
    else
    {
        motorPosition = move2PSensor(PSENB_CHANNEL);

        psenA_psenB_distance = 2.0;
        if(pBufHasDummy)
        {
            psenA_psenB_distance = PSENSORA_TPH_DISTANCE - DUMMY_TOP_HEIGHT_MM;
        }
    }

    //  we can reset the motor counts AFTER we get to PsensA, can't we???
    dcreg(RA_LM_POS0UPDATE_LO) = 0;   // reset the counter
    dcreg(RA_LM_POS0UPDATE_HI) = 0;   // reset the counter

    if (motorPosition >= POSITION_TIMEOUT / POSITION_SLEEP_INT)
    {
#ifdef RETRY_MISSFEED
        if(Retry_MissFeeding)
        {
            Retry_MissFeeding = 0;
        }
        else
        {
            Retry_MissFeeding = 1;
            dcreg16(RA_LM_POSITION0_LO) = 0;
            return DC_TX_START_ERROR;
        }
#endif
        FeedErrorFlag = 1;
        pPrintProcessFlag = 0;
        tx_event_flags_set(&motor_flags, FLAG_PAPER_FEED_ERROR, TX_OR);
        dc_log_printf("STOPPED - Feed Err");
        // Temporarily Disabled. It will be moved to test.c

        //paper check
        ds_StatusMonitor_set(DS_ERROR_REASON_MEDIA_LOAD);
        SetMotorStop();
        //tph_power(0);
        return !DC_SUCCESS;
    }

    /* pz - The DSG app is turning off the Psens LED?  Likely the source of the 'late printing' */
    if(Psensor_Threshold.ds_Tuning_Mode) {
        task_test_set_flag();
    }
    PSENS_LEDCHK() ;

    dcreg(RA_LM_POS0UPDATE_LO) = 0;   // reset the counter
    dcreg(RA_LM_POS0UPDATE_HI) = 0;   // reset the counter

    dc_log_printf("PsensA = %d current @%d", readCurrentADC(PSENA_CHANNEL), dcreg16(RA_LM_POSITION0_LO));
    // TODO: the length between Psensor A and TPH is about 12.5,mm, I need to adjust the sleep timing.
    // paper_detect_sleep_ms =  ((float)motor_period_count/REF_PERIOD_CNT)*3800;
    // From Keith's spread sheet there are 8893.19 gear degrees to move the paper 3.3mm and 18 degrees
    // per motor step. So we need to wait for 13703.29/18 * 12.5/3.3 = 2884 motor pulses
    // current #define is 1500, so assume we are getting every other pulse on the counter

    psenA_TPH_distance = PSENSORA_TPH_DISTANCE + 0.5;
    if(pBufHasDummy)
    {
        psenA_TPH_distance = PSENSORA_TPH_DISTANCE - DUMMY_TOP_HEIGHT_MM;
    }
    motorPosition_printing  = ((psenA_TPH_distance - psenA_psenB_distance) * TARGET_PPMM) ;

    dc_log_printf("LM pos should be 0?=%d - delay til %f %d", dcreg16(RA_LM_POSITION0_LO), psenA_TPH_distance, motorPosition_printing);

    while (dcreg16(RA_LM_POSITION0_LO) < motorPosition_printing)
    {
        //dc_log_printf("waiting to print pos=%d", dcreg16(RA_LM_POSITION0_LO));
        tx_thread_sleep(2);
    }

    dc_log_printf("sending START_PRINTING flag @ %d", dcreg16(RA_LM_POSITION0_LO));

    tx_event_flags_set(&motor_flags, FLAG_PAPER_FEED_START_PRINTING, TX_OR);
    // need to figure out a graceful exit for the motor later, but killing power will stop it

    if(pBufHasDummy)
    {
        RESULT_DIE(tx_timer_activate(&ips_control_timer));
    }
    else
    {
        ips_control_event(0);
    }

    return (result);
}

#ifdef RETRY_MISSFEED
void motor_retryFeed(UINT16 ref_period)
{
    ds_log_printf(DC_LOG_NOTICE"%s!! start", __func__);

    dcreg(RA_LM_POS0UPDATE_LO) = 0;   // reset the counter
    dcreg(RA_LM_POS0UPDATE_HI) = 0;   // reset the counter

    setMotorDir(BACKWARD);
    motor_init_and_start();
    setMotorVelOnTheFly(ref_period*FEED_SPEED_MULTIPLE, 1);
    while (dcreg16(RA_LM_POSITION0_LO) < RETRYFEED_DIST)
    {
        tx_thread_sleep(10);
    }
    ds_log_printf(DC_LOG_NOTICE"%s!! backward", __func__);

    setMotorVelOnTheFly(ref_period, 1);
    while (dcreg16(RA_LM_POSITION0_LO) < 2*RETRYFEED_DIST)
    {
        tx_thread_sleep(10);
    }
    SetMotorStop();
    ds_log_printf(DC_LOG_NOTICE"%s!! forward", __func__);

    setMotorDir(FORWARD);
    motor_init_and_start();
    setMotorVelOnTheFly(ref_period*FEED_SPEED_MULTIPLE, 1);
    while (dcreg16(RA_LM_POSITION0_LO) < RETRYFEED_DIST)
    {
        tx_thread_sleep(10);
    }
    SetMotorStop();

    ds_log_printf(DC_LOG_NOTICE"%s!! end", __func__);
}
#endif

static void app_test_main(ULONG ignore)
{
    DC_RETURN_CODE ret;
    ULONG actual_flags;
    DC_RETURN_CODE result;

    gMotorDirection = FORWARD;
    gMotorVelocity = getRefPeriod(FALSE);

    dc_log_printf("App Thread");

    RESULT_RPT(dc_int_register(DC_INT_MC0_BUFFER, MC0_isr, 18));
    RESULT_RPT(dc_int_register(DC_INT_LM_MOTOR0, LM_MOTOR0_isr, 18));

    gMotorThdStarted = 1;

    result = tx_timer_create(
        &ips_control_timer,
        "ips_control_timer",
        ips_control_event,
        0,                      /* Input to pass to expiration function when timer expires */
        IPS_CONTROL_TIME,       /* initial_ticks */
        0,                      /* one-shot timer */
        TX_NO_ACTIVATE);
    RESULT_DIE(result);

    while (1)
    {
        ret = tx_event_flags_get (&motor_flags, FLAG_START_MOTOR, TX_OR_CLEAR, &actual_flags, TX_WAIT_FOREVER);
        if (ret != TX_SUCCESS)
            dc_log_printf("FATAL ERROR");

        motor_init_and_start();

        EngineInitStallFlag = 0;

        if (isSimpleMotorCall == 0 && gIsEjectJob == 0)
            ret = motor_feeding();

        dc_log_printf("Done with test");
#ifdef RETRY_MISSFEED
        if(ret == DC_TX_START_ERROR)
        {
            motor_retryFeed(getRefPeriod(FALSE));
            tph_power(1);
            tx_event_flags_set(&motor_flags, FLAG_START_MOTOR, TX_OR);
        }
#endif

    } /* while(1) */
}


UINT interrupt_test(void)
{
   UINT32 result;
   tx_event_flags_create (&motor_flags, "mot_state");

   result = tx_thread_create(&app_test_thr, "app_test", app_test_main, 0UL,
                             app_test_stack, sizeof(app_test_stack),
                             thread_priority_get(DC_THREAD_PRIORITY_MEDIUM),
                             thread_priority_get(DC_THREAD_PRIORITY_MEDIUM),
                             1,   //time slice
                             TX_AUTO_START);

   if (result != TX_SUCCESS)
   {
      dc_log_printf("ERR app thread create");
   }
   return result;
}



void pi_setup ( UINT32 addr )
{

   // Set up the LM registers for acquire/snooping

   //----------------------------------------
   //RA_LM_SNOOP0_CONTROL set to Increment position by 1
   //----------------------------------------
   // LM_SnoopX_Control
   //		Enable(0) : When this field is 1, motor motion is determined
   //					by snooping the register bus for writes of a stepper motor control vector
   //		Mode(2:1) : how writes to the stepper motor control vector are interpreted
   //					00 - Each write increments print head position
   //					01 - Each write decrements print head position
   //					10 - Bit6 of LM_SnoopX_Address write data increments print head position
   //						 Bit7 of LM_SnoopX_Address write data decrements print head position
   //		StepSize(4:3) : defines the value added or subtracted from the print head position
   //						in response to a snooped update of the mortor control vecotr
   //					00 - increment or decrement by 1
   //					01 - increment or decrement by 2
   //					10 - increment or decrement by 4
   //					11 - increment or decrement by 8
   regwr(RA_LM_SNOOP0_CONTROL, 0x01);          //
   //
   //----------------------------------------
   //RA_LM_SNOOP0_ADDRESS_HI set to monitor the motor
   //----------------------------------------
   // LM_SnoopX_Address : defines the register address to snoop when monitoring for stepper motor control vector writes
   // RA_GP_OUTPUT0_B(0xE4B == 1110 0100 1010)
   regwr16(RA_LM_SNOOP0_ADDRESS_LO, addr);

   //----------------------------------------
   //RA_LM_SNOOP0_MASK setting
   //;means don't monitor that bit (It is not Motor phase I/O bit)
   //----------------------------------------
   // LM_SnoopX_Mask : The step indicated by LM_SnoopX_Control is recorded when a write to the snoop address is detected,
   //					and the new value differs from the old value in bit positions that are not masked.
   //					The snoop mask logic is not used when you set the Mode field in LM_SnoopX_Control to 10.
   // 0xFE = 1111 1110
   regwr(RA_LM_SNOOP0_MASK, 0xFE);

   dc_log_printf("LM snoop init done");


   return ;

}

void pi_go(void)
{
   //UINT8 idx;

   // The LM motor move interrupts are handled by the hardware block.

   // The LM_DISTANCE interrupt register is only a seven bit field so a max value of 127 can be used
   ramp_cycles_remaining = ramp_cycles;

   // LM_IntDistanceX : can be used to monitor the position and velocity of the motor
   // 		Each time the motor moves a distance defined by LM_IntDistanceX, an interrupt will be flagged
   if (ramp_cycles <= LM_DISTANCE_MAX_VALUE)
   {
      dcreg(RA_LM_INTDISTANCE0) = ramp_cycles;     // after ramp_cycles counts an interrupt will be generated
      ramp_cycles_remaining = 0;
   }
   else
   {
      dcreg(RA_LM_INTDISTANCE0) = LM_DISTANCE_MAX_VALUE;     // after 127 counts an interrupt will be generated. This is the max value
      ramp_cycles_remaining -= LM_DISTANCE_MAX_VALUE;
   }

   dc_int_flag_clear(DC_INT_LM_MOTOR0);

   // LM_IntFlagX_Clear : To clear a particular interrupt flag, write the corresponding bit positin with a 1.
   // 	0	Distance		determines that the motor has moved an amount >= LM_IntDistanceX
   //		1	Reverse			determines that the motor has reversed direction
   //		2	DetectPosition	detects that the motor has reached the detect position
   dcreg(RA_LM_INTFLAG0_CLEAR) = 0x03; // clear any pending interrupts

   RESULT_RPT(dc_int_irq_enable(DC_INT_LM_MOTOR0));

   // LM_IntEnableX : enable the associated interrupt flags to create a motorX interrupt
   // 	0 	Distance
   //		1	Reverse
   //		2	DetectPosition
   dcreg(RA_LM_INTENABLE0) = 0x01; // interrupt on distance

   RESULT_RPT(dc_int_register(DC_INT_MC0_BUFFER, MC0_isr, 18));

   dc_int_flag_clear(DC_INT_MC0_BUFFER);

   RESULT_RPT(dc_int_irq_enable(DC_INT_MC0_BUFFER));

   DC_LOG_PRINTF("start the motor");

   DC_LOG_PRINTF("val INT0ENABLE : %d", dcreg(RA_IC_INT0ENABLE_XLO));
   DC_LOG_PRINTF("val INT0STATUS : %d", dcreg(RA_IC_INT0STATUS_XLO));

   dc_log_printf("pre MC0 %d / %d", dcreg(RA_MC0_VOLUME), dcreg(RA_MC0_CONTROL) );

   // start the motor
   // MCx_Control
   //		0	Local_Reset		1 = force reset, 0 = normal operation
   //		1	Enable			provides a mechanism by which the section can be activated and paused
   //		2	VolumeControl	MC_Volume register is checked to verify that a valid value/delay pair is present
   //		3	Log2Delay		delay period is interpreted as a 3.5 fixed point log2 of the delay value
   //		6	CountDown		read-only,
   //		7	Busy			read-only, indicates when the motor control section is busy
   if(!EngineInitStallFlag)
   	  dcreg(RA_MC0_CONTROL) = 0x06;     // 0110 will run till volume is zero
   else
   {
   	  dcreg(RA_MC0_CONTROL) = 0x00;     // Force Reset
   	  EngineInitStallFlag = 0;
   }
}

int pi_done(void)
{
   if (GetScanState())
   {	/* checks if MTR_STATE_STOP */
      return 1;
   }
   return 0;
}

// ***************************************************************************
// Test sequences.
//

void do_serial(int primeOnly, UINT32 addr)
{
   UINT32 start_time;
   UINT32 pi_p_time = 0;

   dc_log_printf("run_cycles = %d, ramp_cycles = %d", run_cycles, ramp_cycles);

   if (!primeOnly)
   {
      pi_setup(addr);
      start_time = tx_time_get();
      pi_go();
      pi_p_time = dc_time_diff_msec(start_time);
   }


   //  dc_log_printf("Complete serial operations\n");
   dc_log_printf("Complete serial operations (primeOnly %d\n", primeOnly);
   dc_log_printf("pi time %d msec\n", pi_p_time);

   dc_log_printf("run_cycles = %d, ramp_cycles = %d", run_cycles, ramp_cycles);
   //  ramp_cycles = ramp_cycles_remaining = stepper_config->steps_in_ramp;
   run_cycles = run_cycles_initial;
   ds_log_printf("%s run_cycles=%d", __func__, run_cycles);
}


int GetScanState(void)
{
   if (mtrState == MTR_STATE_STOP)
   {
      return 1;
   }

   return 0;
}


void SetMotorStop(void)
{
   dc_log_printf("[MTR] SETSTOP");

   mtrState = MTR_STATE_STOP;
   tx_thread_sleep(100);
   regwr( RA_IC_FLAGSET_XLO, 0x40);
   tph_power(FALSE);
}


int GetMotorState(void)
{
  return mtrState;
}


void dumpMCregs(void)
{
  dc_log_printf(">======REGS==========<");
  dc_log_printf("MC CTRL:%x", dcreg(RA_MC0_CONTROL) );
  dc_log_printf("IOADDR: %x", dcreg32(RA_MC0_IOADDR_LO) );
  dc_log_printf("MC0PER: %x", dcreg32(RA_MC0_PERIOD_LO) );
  dc_log_printf("WR PTR: %x", dcreg(RA_MC0_WRITEPTR) );
  dc_log_printf("VOL: %x", dcreg(RA_MC0_VOLUME) );
  dc_log_printf("======================");
}


DC_RETURN_CODE init_motor(STEPPER_CONFIG *stepper_config, MOTOR_DRIVER_CONFIG *mdc)
{
   DC_RETURN_CODE ret = DC_SUCCESS;
   UINT32 main_hz;
   UINT32 k, k1, k2, tmp, runningVal, i;
   float step;
   /* change ramps to be smooth/continuous instead of stepped */
   UINT32 delay_cycles, delay_count;

   RESULT_CHK(dc_clock_freq( &main_hz, RA_CK_MAIN_LO));

   DC_LOG_PRINTF("main clock (x2) = %d", main_hz);
   DC_LOG_PRINTF("io_addr = 0x%x", stepper_config->mc_ioaddr);
   DC_LOG_PRINTF("steps in ramp = %d", stepper_config->steps_in_ramp);
    /* pz - ramps are now linear  1 cycle per speed */
   DC_LOG_PRINTF("direction = %d", stepper_config->direction);
   DC_LOG_PRINTF("enable = %d", stepper_config->enable);

   ramp_cycles = ramp_cycles_remaining = stepper_config->steps_in_ramp;
   run_cycles = run_cycles_initial = 2*((PRINTABLE_HEIGHT*TARGET_PPS)/(ONE_INCH*t_IPS));
   DC_LOG_PRINTF("%s run_cycles = %d", __func__, run_cycles);

   runningVal = (double)main_hz / (TARGET_PPS * getRefPeriod(FALSE))+1;
   step = (float)(RAMP_DELAY_MAX - runningVal) / (float)stepper_config->steps_in_ramp;
   DC_LOG_PRINTF("running Value delay = %d, step %.3f", runningVal,step);

   /* Calculate the ramp delay table */
   /* first calculate the starting entry */

   /* calculate the enable hold time*/
   delay_cycles = main_hz / (stepper_config->enable_delay * 1000);
   DC_LOG_PRINTF ("delay_cycles = %d", delay_cycles);

   /* Build the ramp table */

   /* pz - FIXME ... why is the hold being done with discrete interrupts??? Why not just turn it on then wait? */
   /* first enable the motor and wait for the hold time */
   delay_count = 0;
   while (delay_cycles > RAMP_DELAY_MAX)
   {
      mtr1_ramp[delay_count++] = mdc->enable;
      mtr1_ramp[delay_count++] = RAMP_DELAY_MAX;
      delay_cycles -= RAMP_DELAY_MAX;
   }
   mtr1_ramp[delay_count++] = mdc->enable;
   mtr1_ramp[delay_count++] = (RAMP_DELAY_MAX-1);  // pz - does not hurt to extend hold - mark as 254
   //mtr1_ramp[delay_count++] = delay_cycles;   /* pz FIXME - doesn't this need a min check? */

   /* make sure this the ramp ends on a modulo 8 boundary. This will enable all the entries to remain in phase */

   tmp = delay_count % 8;
   dc_log_printf("delay_count = %d, mod_8 = %d", delay_count, tmp);

   for (; tmp < 8; tmp += 2)
   {
      dc_log_printf("tmp = %d", tmp);
      mtr1_ramp[delay_count++] = mdc->enable;
      mtr1_ramp[delay_count++] = RAMP_DELAY_MAX-2;  // pz - does not hurt to extend hold - mark as 253
      //mtr1_ramp[delay_count++] = 25;
   }

   dc_log_printf("After padding delay_count = %d, mod 8 is %d", delay_count, delay_count % 8);

   /* pz FIXME - why quantized steps x 8 rather than a smooth ramp??? */

   /* start at 255 and ramp down to running_val - smoothly!!! */
   for (k1 = 0; k1 < stepper_config->steps_in_ramp; )
   {
        for (i=0; i<MOTOR_PHASE_COUNT; i++)
        {
            mtr1_ramp[delay_count++] = mdc->phase[i] | mdc->enable;
            mtr1_ramp[delay_count++] = (RAMP_DELAY_MAX-step*k1++);
        }
   }

   // add the running parameters to the buffer so that the interrupt will always be pulling good data.
   // It is doubtful that the ramp will be a multiple of the MC interrupt.

   // determine the current offset into the array
   end_of_ramp = delay_count - 1;
   dc_log_printf("end of ramp is %d", end_of_ramp);

   /* pz FIXME - last value should be 'runningVal' ... but it is not!!! 1 step short! */
   for (k2 = 0; k2 < MCx_WRITE_SIZE/2; k2++)
   {
      for (i=0; i<MOTOR_PHASE_COUNT; i++)
      {
          mtr1_ramp[delay_count++] = mdc->phase[i] | mdc->enable;
          mtr1_ramp[delay_count++] = runningVal;
      }
   }
   
   pMCT_RampUp = mtr1_ramp;
   
   dc_log_printf("last index in ramp = %d", delay_count - 1); // actually bigger, but bounds checking so its ok
   if (delay_count >= RAMP_COUNT)
   {
      dc_log_printf("ERR: MOTOR RAMP TOO LONG");
   }
   /* build the down ramp, mtr1_down_ramp*/
   /* pz FIXME - this was warped because there are 9 increments in tmp every time through! */
   for (tmp = 0; tmp < end_of_ramp; )
   {
      for (i=0; i<MOTOR_PHASE_COUNT; i++)
      {
          mtr1_down_ramp[tmp++] = mdc->phase[i] | mdc->enable;
          mtr1_down_ramp[tmp] = mtr1_ramp[end_of_ramp - (tmp+1)];
          tmp++;
      }
   }
   dc_log_printf("Initial Motor Ramp, runningVal[0x%x]", runningVal);
   // dump the ramp in pairs
   for (k2 = 0; k2 < RAMP_COUNT/4; k2++)
   {
      DC_LOG_PRINTF("  1val[%d]=%x, delay=%d", k2 * 2, mtr1_ramp[k2 * 2], mtr1_ramp[k2 * 2 + 1]);
   }
   
   pMCT_RampDown = mtr1_down_ramp;

   /* build the steady state motor signal */
   /* FIXME pz - note that the ramp is not extending out to running_val ( 1 short ) */
   tmp = MOTOR_PHASE_COUNT*2;
   for (k2 = 0; k2 < tmp; k2++)
   {
      for (i=0; i<MOTOR_PHASE_COUNT; i++)
      {
          mtr1_steady_state[(tmp * k2) + 2*i] = mdc->phase[i] | mdc->enable;
          mtr1_steady_state[(tmp * k2) + 2*i + 1] = runningVal;
          DC_LOG_PRINTF("  1.5val[%d]=%x, delay=%d", (tmp * k2) + 2*i, mtr1_steady_state[(tmp * k2) + 2*i], mtr1_steady_state[(tmp * k2) + 2*i + 1]);
      }
   }
   dc_log_printf("Steady State Motor Signals");
   // dump the ramp in pairs
   for (k2 = 0; k2 < MCx_WRITE_SIZE/2; k2++)
   {
      DC_LOG_PRINTF("  2val[%d]=%x, delay=%d", k2 * 2, mtr1_steady_state[k2 * 2], mtr1_steady_state[k2 * 2 + 1]);
   }
   
   pMCT_Steady = mtr1_steady_state;
   
   dc_log_printf("Slow down Motor Ramp");
   // dump the ramp in pairs
   for (k2 = 0; k2 < RAMP_COUNT/4; k2++)
   {
      DC_LOG_PRINTF("  3val[%d]=%x, delay=%d", k2 * 2, mtr1_down_ramp[k2 * 2], mtr1_down_ramp[k2 * 2 + 1]);
   }

   /* set up the hardware */
   dc_log_printf("motor setup");

   dcreg(RA_MC0_CONTROL) = 0x01; // force reset
   dc_time_wait_cycles(200);
   dcreg(RA_MC0_CONTROL) = 0x00; // normal operation

   while (dcreg(RA_MC0_CONTROL) & 0x80) // motor control section is busy
   {
      dc_log_printf("wait for busy flag");
      tx_thread_sleep(10);
   }

   /* drive register make enable and motor signal outputs */
   // stepper_config->mc_ioaddr - 1 == RA_GP_DRIVE0_B
   // stepper_config->mc_ioaddr == RA_GP_OUTPUT0_B
   // mdc->phase[0] | mdc->phase[1] | mdc->phase[2] | mdc->phase[3] | MOTOR_ENABLE == 0011(0x3) + 1100(0xC) == 1111(0xF)
   // GP_Drive0_x : control the directin of the associated gpio collection.
   //				   Writing 0xF value to RA_GP_DRIVE0_B sets all GP_B pin as output.
   // GP_Output0_x : control the logic level driven onto associated gpio.
   //				   Writing 0x00 value to RA_GP_OUTPUT0_B sets all GP_B's output as 0.
   dcreg(stepper_config->mc_ioaddr - 1) = mdc->enable;
   for (i=0; i<MOTOR_PHASE_COUNT; i++)
   {
        dcreg(stepper_config->mc_ioaddr - 1) |= mdc->phase[i];
   }
   dcreg(stepper_config->mc_ioaddr) = 0x00;
   // MCx_IOAddr
   // 	defines the I/O address to write the value portion of the value/delay pair
   //		The motor control logic also includes logic to deliver control vectors to 1 or 2 external motor driver ASICs via a serial channel
   //		Motor vectors written to certain registers will automatically initiate serial transfers to the analog ASICs
   dcreg32(RA_MC0_IOADDR_LO) = stepper_config->mc_ioaddr;

   dcreg32(RA_MC0_PERIOD_LO) = (UINT32)motor_period_count; // base period in main clocks for computing the delay between wirtes
   dc_log_printf("progammed period = %d", dcreg32(RA_MC0_PERIOD_LO));

   // MCx_WritePtr
   //		the write buffer location to be written by the next write to MC0_Write
   dc_log_printf("pre val MC0_WRITEPTR : %d", dcreg(RA_MC0_WRITEPTR));
   dcreg(RA_MC0_WRITEPTR) = 0;
   dc_log_printf("post val MC0_WRITEPTR : %d", dcreg(RA_MC0_WRITEPTR));

   // MCx_Write
   //		loads a byte into the write buffer location pointed to by MC0_WritePtr
   //		MC0_WritePtr register moves to the next write buffer location
   //		write buffer volume tracked by MC_Volume is incremented
   for (k = 0; k < MCx_WRITE_SIZE; k++)
   {
      dcreg(RA_MC0_WRITE) = mtr1_ramp[k];
   }

   // Stepper Motor Control Write Buffers
   // 	Support for controlling stepper motors is provided by a set of two specialized wirte buffers.
   // 	Each write buffer can be programmed with up to 32 value/delay pairs.
   //		The write buffer logic processes these pairs
   //			by wirting the value (consisting of from one to four bytes) to a specified I/O location
   //			and then waiting the delay period (specified by a single byte).
   //		By using these write buffers, the firmware can reduce the microcontroller interrupt period
   //			from once every motor step to once every N motor steps.
   //		The separate delay period for each value supports ramp tables for motor acceleration and deceleration.

   dc_log_printf("pre val MC0_vol : %d", dcreg(RA_MC0_VOLUME));
   // MCx_Volume : keeps track of the number of valid bytes in the write buffer
   // 		Each time the MC_Write_* register is written, the volume is incremented.
   //			Each time the write buffer logic processes a value/delay pair, the volume is decremented by two.
   //			When the MC_Volume register is 0, the interrupt flag MC_Buffer is set
   dcreg(RA_MC0_VOLUME) = 0x40; //
   dc_log_printf("post val MC0_vol : %d", dcreg(RA_MC0_VOLUME));

   dumpMCregs();


   return ret;
}


DC_RETURN_CODE FindMC0period(int ppsStart, int ppsRun, UINT32 *pPeriodMC0)
{
	DC_RETURN_CODE status = DC_SUCCESS;
	
	int ppsGreater = ppsRun;
	int ppsLesser  = ppsStart;
	
	UINT32 main_hz = 0;
	UINT32 periodMC0 = 0;
	
	float Tmain = 0;
	float Ttarget = 0;
	float Tmc0 = 0;
	
	dc_log_printf("---FindMC0period--- ");

    if (dcreg(RA_MC0_CONTROL) & 0x02) { // motor active
      *pPeriodMC0 = dcreg32(RA_MC0_PERIOD_LO);
      dc_log_printf(" active periodMC0:%d", *pPeriodMC0);
      return status;
    }
	
	dc_clock_freq(&main_hz, RA_CK_MAIN_LO);
	Tmain = 1.0/(float)main_hz;
	
	dc_log_printf(" Tmain:%1.9f ", Tmain);
	
	if (ppsStart > ppsRun) {
		ppsGreater = ppsStart;
		ppsLesser = ppsRun;
	}
	Ttarget = 1.0/(float)ppsGreater;
	
	Tmc0 = (Ttarget * 3.0) / RAMP_DELAY_MAX;
    periodMC0 = Tmc0 / Tmain;
	dc_log_printf("Tmco:%1.9f", Tmc0);
	dc_log_printf("perMC0:%d", periodMC0);
	*pPeriodMC0 = periodMC0;
	
	//check ok for other pps
	if (ppsLesser > 0) {
		float Tother = 1.0/(float)ppsLesser;
		UINT32 perOther = Tother / Tmc0;
		if (perOther < RAMP_DELAY_MAX) {
			status = DC_SUCCESS;
		} else {
			dc_log_printf("Dly too big: %d ", perOther);
			status = DC_FAILURE;
		}
	}
	
	return status;
}


DC_RETURN_CODE MakeRamp(int ppsStart, int ppsRun, UINT32 periodMC0, UINT8 *pMCTramp, UINT8 direction)
{
    DC_RETURN_CODE status = DC_FAILURE;	
	UINT32 main_hz = 0;
	
	float Tmain  = 0.0;
	float Tmc0   = 0.0;
	float Tstart = 0.0;
	float Trun   = 0.0;
	float step   = 0.0;
	
	UINT32 perStart = 0;
	UINT32 perRun = 0;
	
	int i = 0;
	int phase = 0;
	int num_max_delays = 0;
    int ramp_extent = ( ( ((pStepper_config->steps_in_ramp * 2)/MCx_WRITE_SIZE) + 1 ) * 2 ) * MCx_WRITE_SIZE; 
	
	UINT8 *pRamp = pMCTramp;
	
	dc_log_printf("-----MakeRamp-----");
    dc_log_printf(" Ramp Extent:%d", ramp_extent);
	
	// determine Tmc0
	dc_clock_freq(&main_hz, RA_CK_MAIN_LO);
	Tmain = 1.0/(float)main_hz;
	Tmc0 = (float)periodMC0 * Tmain;
	
	//dc_log_printf("");
	//dc_log_printf(" perMC0:%d", periodMC0);
	//dc_log_printf(" Tmc0:%1.9f", Tmc0);
	
	// get periods in mc0's
	if (ppsStart != 0) {
	    Tstart = 1.0 / (float) ppsStart;
	    perStart = Tstart / Tmc0;
	} else {
		perStart = RAMP_DELAY_MAX;
	}
	if (ppsRun != 0) {
		Trun = 1.0 / (float) ppsRun;
		perRun = Trun / Tmc0;
	} else {
		perRun = RAMP_DELAY_MAX;
	}
	
	//dc_log_printf(" Trun:%1.9f", Trun);
	//dc_log_printf(" perRun:%u, perStart:%u", perRun, perStart);
	
	// calc step, can be negative
	step = ( (float)perRun - (float)perStart ) / (float)pStepper_config->steps_in_ramp;
	
	//dc_log_printf(" step:%3.5f", step);
	
	//start delays, enable_delay in ms
	if ((pStepper_config->enable_delay > 0) && (perStart == RAMP_DELAY_MAX) ) {
		float Thold = (float)pStepper_config->enable_delay * 0.001;// convert to ms
		
		num_max_delays =  Thold / ((float)RAMP_DELAY_MAX * Tmc0);
		
		//dc_log_printf(" num max dlys:%d", num_max_delays);
		
		for (i = 0; i <= num_max_delays; i++) {
			*pRamp++ = pmdc->enable;
			*pRamp++ = RAMP_DELAY_MAX;
		}
	}

    if (MOVE_FORWARD == direction) { // MOVE_FORWARD
        // fill ramping section of ramp table
        for (i = 0; i <= pStepper_config->steps_in_ramp; i++) {
            if (phase >= MOTOR_PHASE_COUNT) {
                phase = 0;
            }
            *pRamp++ = pmdc->phase[phase++] | pmdc->enable;
            *pRamp++ = (UINT8)((float)perStart + (float)i * step);
        }	

        //fill running portion of ramp //ramp_extent
        //for (i = pStepper_config->steps_in_ramp + num_max_delays + 1; i < RAMP_COUNT/2; i++) {
        for (i = pStepper_config->steps_in_ramp + num_max_delays + 1; i < ramp_extent/2; i++) {
		  if (phase >= MOTOR_PHASE_COUNT) {
            phase = 0;
          }
          *pRamp++ = pmdc->phase[phase++] | pmdc->enable;
          *pRamp++ = perRun;
        }
        
    } else { // MOVE_BACKWARD i.e. fill step phase values in reverse order, with same delays
        phase = MOTOR_PHASE_COUNT - 1;
        for ( i = 0; i <= pStepper_config->steps_in_ramp; i++) {
            if (phase < 0) {
                phase = MOTOR_PHASE_COUNT - 1;
            }
            *pRamp++ = pmdc->phase[phase--] | pmdc->enable;
            *pRamp++ = (UINT8)((float)perStart + (float)i * step);
        }

        //for (i = pStepper_config->steps_in_ramp + num_max_delays + 1; i < RAMP_COUNT/2; i++) {
        for (i = pStepper_config->steps_in_ramp + num_max_delays + 1; i < ramp_extent/2; i++) {
            if (phase < 0) {
                phase = MOTOR_PHASE_COUNT - 1;
            }
            *pRamp++ = pmdc->phase[phase--] | pmdc->enable;
            *pRamp++ = perRun;
        }
    }

	status = DC_SUCCESS;
	
	return status;
}


DC_RETURN_CODE MakeRunningTable(int ppsRun, UINT32 periodMC0, UINT8 *pMCTsteady, UINT8 direction)
{
	DC_RETURN_CODE status = DC_FAILURE;
	UINT8 *pMCT = pMCTsteady;
	
	UINT32 main_hz = 0;
	UINT32 perRun = 0;
	
	float Tmain = 0.0;
	float Tmc0 = 0.0;
	float Trun = 0.0;
	
	int i = 0;
	int phase = 0;
	
	// dc_log_printf(" ---- Make Stdy table ----");
	// determine Tmc0
	dc_clock_freq(&main_hz, RA_CK_MAIN_LO);
	Tmain = 1.0/(float)main_hz;
	Tmc0 = (float)periodMC0 * Tmain;
	Trun = 1.0 / (float) ppsRun;
	perRun = Trun / Tmc0;

	dc_log_printf(" perRun:%d", perRun);

    if (MOVE_FORWARD == direction) { // MOVE_FORWARD
    
        //for (i = 0; i < RAMP_COUNT/2; i++) {
        for (i = 0; i < MCx_WRITE_SIZE/2; i++) {
            if (phase >= MOTOR_PHASE_COUNT) {
                phase = 0;
            }
            *pMCT++ = pmdc->phase[phase++] | pmdc->enable;
            *pMCT++ = perRun;
        }
    } else { // MOVE_BACKWARD
        phase = MOTOR_PHASE_COUNT - 1;
        //for (i = 0; i < RAMP_COUNT/2; i++) {
        for (i = 0; i < MCx_WRITE_SIZE/2; i++) {
            if ( phase < 0 ) {
                phase = MOTOR_PHASE_COUNT - 1;
            }
            *pMCT++ = pmdc->phase[phase--] | pmdc->enable;
            *pMCT++ = perRun;
        }
    }
	
	status = DC_SUCCESS;

	return status;
}

#define RAMP_SZ 96/2 // see init_motor_config()
#define MCT_BUF_SIZE 2 * ( ( ((RAMP_SZ * 2)/MCx_WRITE_SIZE) + 1 ) * 2 ) * MCx_WRITE_SIZE + MCx_WRITE_SIZE

UINT8 aMCTbuffer1[MCT_BUF_SIZE];
UINT8 aMCTbuffer2[MCT_BUF_SIZE];

//
// stepperStart()
//    ppsStart - starting step rate
//               if 0 then will ramp down to 0 at end
//    ppsRun   - running step rate
//    direction- MOVE_FORWARD or MOVE_BACKWARD
//    pBuffer - 2 * ( ( ((pStepper_config->steps_in_ramp * 2)/MCx_WRITE_SIZE) + 1 ) * 2 ) * MCx_WRITE_SIZE
//              + MCx_WRITE_SIZE
//              e.g. for step_in_ramp == 48, pBuffer m/b >= 2 * 256 + 64 == 576
//
DC_RETURN_CODE stepperStart(int ppsRun,
                             UINT16 steps,
                             int direction,
                             void (*call_back)(int)
                             )
{
	DC_RETURN_CODE status = DC_FAILURE;
	UINT32 periodMC0 = 0;
	int i = 0;
    int ramp_extent = ( ( ((pStepper_config->steps_in_ramp * 2)/MCx_WRITE_SIZE) + 1 ) * 2 ) * MCx_WRITE_SIZE; 
	
	// divy up the buffer passed
    UINT8* pMCTramp = aMCTbuffer1;
    UINT8* pMCTsteady = pMCTramp + ramp_extent+64;// let ramp finish despite holds
    UINT8* pMCTdone = pMCTsteady + MCx_WRITE_SIZE;
    int state_motor = GetMotorState();
    static int ppsRunning = 0;
    int ppsStart = 0;
	
	dc_log_printf("---stepperStart---%d", direction);
    dc_log_printf("-- %d, %d", ppsRun, steps);

    if (NULL != callback) {
        if (run_cycles) {
            (*callback)(run_cycles);//move incomplete
        } else {
            (*callback)(1);// 0 == move incomplete
        }
        callback = NULL;
    }
    callback = call_back;

    dc_log_printf("--mst:%d", state_motor);
    dc_log_printf("Ramp Rm:%d", ramp_cycles_remaining);
    dc_log_printf("pos=%d", dcreg16(RA_LM_POSITION0_LO));

    // ensure motor is not ramping MTR_STATE_RAMP_UP
    if ( (state_motor != MTR_STATE_IDLE)
         && (state_motor != MTR_STATE_RUNNING) ) {
        dc_log_printf("Can't start mtr st:%d", state_motor);
        return DC_FAILURE;
    }
    
    if (state_motor == MTR_STATE_RUNNING) {
        ppsStart = ppsRunning;
    }
    
    // find a mc0 period that satisfies both these ips
	status = FindMC0period(ppsStart, ppsRun, &periodMC0);
	
	// make a ramp from ppsStart to ppsRun
	if(DC_SUCCESS == status) {
      status = MakeRamp(ppsStart, ppsRun, periodMC0, pMCTramp, direction);
	}
#if (0)
    //print ramp
    dc_log_printf("--Ramp to speed--");
    for (i=0; i < (ramp_extent/2+20); i+=2) {
      dc_log_printf("  %d: %d : %d", i, pMCTramp[i], pMCTramp[i+1]);
    }
    dc_log_printf("-----------------");
#endif
	
    // make a ramp to finish the trap move to stop
    if(DC_SUCCESS == status) {
      status = MakeRamp(ppsRun, 0, periodMC0, pMCTdone, direction);
    }

	//dc_log_printf("st: perMC0: %d", periodMC0);
    // ramp_cycles_remaining are the # steps in Ramps of trapezoidal move
	ramp_cycles = ramp_cycles_remaining = pStepper_config->steps_in_ramp;
    
    // run_cycles are the # of steps in the flat of the trap move
    //run_cycles = run_cycles_initial = 2*((PRINTABLE_HEIGHT*TARGET_PPS)/(ONE_INCH*t_IPS));
    run_cycles = run_cycles_initial = steps - ( 2 * ramp_cycles );
    
 //	if (state_motor == MTR_STATE_IDLE ){// && !((dcreg(RA_MC0_CONTROL) & 0x02))) {
    if (DC_SUCCESS == status) {
    // local reset
		dcreg(RA_MC0_CONTROL) = 0x01; // force reset
        dc_time_wait_cycles(200);// hold rst per reg spec
        dcreg(RA_MC0_CONTROL) = 0x00; // normal operation
		
		while (dcreg(RA_MC0_CONTROL) & 0x80) // motor control section is busy
	    {
		    dc_log_printf("wait MC0 busy clr");
		    tx_thread_sleep(10);
	    }
		
		// config gpios as outputs
		dcreg(  pStepper_config->mc_ioaddr - 1) = pmdc->enable;
	    for (i=0; i<MOTOR_PHASE_COUNT; i++)
	    {
		 	dcreg(pStepper_config->mc_ioaddr - 1) |= pmdc->phase[i];
	    }
	    dcreg(pStepper_config->mc_ioaddr) = 0x00;
	   
	    // point MC at GP reg stepper is connected to
	    dcreg32(RA_MC0_IOADDR_LO) = pStepper_config->mc_ioaddr;
		
		dcreg32(RA_MC0_PERIOD_LO) = periodMC0;
		
		dcreg(RA_MC0_WRITEPTR) = 0;
		for (i = 0; i < MCx_WRITE_SIZE; i++)
	    {
          dcreg(RA_MC0_WRITE) = pMCTramp[i];
 	    }
		dcreg(RA_MC0_VOLUME) = 0x40;

		//dcreg(RA_MC0_CONTROL) = 0x02;// go
        //dcreg(RA_MC0_CONTROL) = 0xc6;// go
        do_serial(0, pStepper_config->mc_ioaddr);

        
	} else {
      dc_log_printf("StepperStart fail, make ramp");
      status = DC_FAILURE;
      return status;
    }
	
	// motor running: copy in tables to pointers and set state to RAMP
	pMCT_RampUp = pMCTramp;
	pMCT_Steady = pMCTsteady;
    pMCT_RampDown = pMCTdone;

    mtrState = MTR_STATE_RAMP_UP;

    dcreg(RA_IC_FLAGSET_XLO) = 0x40; // kick MC0 isr
    MakeRunningTable(ppsRun, periodMC0, pMCTsteady, direction);
    ppsRunning = ppsRun;
	
	return status;
}// stepperStart

void printRamp(UINT8* pTable)
{
  int i = 0;
  for (i = 0; i < 128; i+=2) {
    dc_log_printf(" val[%d]:%d, dly:%d", i, pTable[i], pTable[i+1]);
  }
}


void mycallback(int steps_remain)
{
  dc_log_printf("MyCallback: %d", steps_remain);
}

void TMH_test(void)//
{
    STEPPER_CONFIG motor_config;
	MOTOR_DRIVER_CONFIG mdc;

	init_motor_config(&motor_config);
	init_motor_driver_config(&mdc);

    stepperStart(2000, 3000, MOVE_FORWARD, &mycallback);

    // wait move to finish
    /* while( MTR_STATE_IDLE != GetMotorState() ){ */
    /*     tx_thread_sleep(200); */
    /* } */

    tx_thread_sleep(1000);
    
    stepperStart(1000, 2000, MOVE_FORWARD, &mycallback);

	//SetMotorStop();
}


void TMH_test2(void)
{
    STEPPER_CONFIG motor_config;
	MOTOR_DRIVER_CONFIG mdc;
    UINT16 positionDriveToA = 0;

	init_motor_config(&motor_config);
	init_motor_driver_config(&mdc);

    dcreg16(RA_LM_POS0UPDATE_LO) = 0;//rst motor position

    stepperStart(1000, 10000, MOVE_FORWARD, NULL);

    
    //wait for PSENA to be interrupted
    // has a dpf with LM motor position and positionDriveToA
    positionDriveToA = move2PSensor(PSENA_CHANNEL);
    // excessive positionDriveToA indicates no feed situation

    dc_log_printf("got to PSENA! %d", positionDriveToA);

    dcreg16(RA_LM_POS0UPDATE_LO) = 0;
    //wait continue move to nip, 12mm

    while( dcreg16(RA_LM_POSITION0_LO) < 2515 ){//wait for nip position
      tx_thread_sleep(1);
    }
    
    dc_log_printf("Got to NIP:%d", dcreg16(RA_LM_POSITION0_LO));

    // switch to print speed
    stepperStart(800, 20000, MOVE_FORWARD, NULL);

    numBarCodeSamples = 0;
    flagBarcodeActive = 1;

    while( dcreg16(RA_LM_POSITION0_LO) < 18500 ){//wait for finish
      tx_thread_sleep(10);
    }

    flagBarcodeActive = 0;
    dc_log_printf("array:0x%x", samplesBarCode);
    dc_log_printf("num samples: 0x%x", numBarCodeSamples);
    
    dc_log_printf("DONE:%d", dcreg16(RA_LM_POSITION0_LO));
    
	SetMotorStop();
}
#if (1)
void TMH_test3(void)
{
    STEPPER_CONFIG motor_config;
	MOTOR_DRIVER_CONFIG mdc;

    init_motor_config(&motor_config);
	init_motor_driver_config(&mdc);

    dcreg16(RA_LM_POS0UPDATE_LO) = 0;//rst motor position

    stepperStart( 3 * MM_PER_SEC_PPS, // 3mm per sec running vel, in PPS
                  9 * MM_STEPS,       // move 9mm
                  MOVE_FORWARD, NULL);
  
}
#else
void TMH_test3(void)
{
    STEPPER_CONFIG motor_config;
	MOTOR_DRIVER_CONFIG mdc;
    bool bDone = FALSE;

    
    

}// TMH_test3
#endif

void eject_test(void)
{
    UINT16 psenB_counts;
    UINT16 psenA_counts;
    STEPPER_CONFIG motor_config;
	MOTOR_DRIVER_CONFIG mdc;
    int tail_count = 0;
    int retry = 0;

    BOOL bDone = FALSE;
    int stateEject = 0;

    while (!bDone) {
      switch(stateEject){
      case 0:
          psenA_counts = ADC_readCurrentAvg(PSENA_CHANNEL, 1);

          if ( psenA_counts < PSENA_PAPER_LTLEVEL ) {// paper in path
              dc_log_printf("--Paper in path: A:%d", psenB_counts);
              stateEject++;
          } else {
              bDone = TRUE;
              dc_log_printf("No Paper in path, A:%d", psenB_counts);
          }
          break;
      case 1:// move back 1mm
          init_motor_config(&motor_config);
          init_motor_driver_config(&mdc);
          stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                        1 * MM_STEPS,       // move 1mm
                        MOVE_BACKWARD, NULL);
          stateEject++;
          break;
      case 2:// wait move to cmplete
          if (MTR_STATE_IDLE == GetMotorState()) {
              stateEject++;
          }
          break;
      case 3:// move fwd 2mm
          stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                        2 * MM_STEPS,       // move 2mm
                        MOVE_FORWARD, NULL);
          stateEject++;
          break;
      case 4:// wait move complete
          if (MTR_STATE_IDLE == GetMotorState()) {
              stateEject++;
          }
          break;
      case 5:// move back 1mm
          stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                        1 * MM_STEPS,       // move 1mm
                        MOVE_BACKWARD, NULL);
          stateEject++;
          break;
      case 6:// wait move complete
          if (MTR_STATE_IDLE == GetMotorState()) {
              stateEject++;
              dc_log_printf("--Unstuck");
          }
          break;
      case 7:// start final move 
          stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                        (80+80) * MM_STEPS,      // worst case move to eject +100% for slippage if not in nip yet
                        MOVE_FORWARD, NULL);
          stateEject++;
          break;
      case 8:// wait for tail to pass psenA (not sure how far sheet was in to know exact distance)
          psenA_counts = ADC_readCurrentAvg(PSENA_CHANNEL, 1);
          if (psenA_counts > PSENA_NOPAPER_GTLEVEL) {
              dc_log_printf("--Clrd Sensors: %d", tail_count);
              dc_log_printf("--Counts A:%d, B:%d", psenA_counts, psenB_counts);
              dcreg16(RA_LM_POS0UPDATE_LO) = 0;
              stateEject++;
          } else if ( (MTR_STATE_IDLE == GetMotorState())
                       && (retry < 1) ) {
              // retry move once in case of excessive slippage
              dc_log_printf("--retry move");
              retry++;
              stateEject = 7;
          }
          //dc_log_printf("--Cts A:%d", psenA_counts);
          break;
      case 9:// wait for tail to clear nip 8 more mm
        if ( (dcreg16(RA_LM_POSITION0_LO) > (8 * MM_STEPS) )
             ||(MTR_STATE_IDLE == getMotorState()) ) {
              SetMotorStop();
              psenA_counts = ADC_readCurrentAvg(PSENA_CHANNEL, 1);
              if (psenA_counts < PSENA_PAPER_LTLEVEL) {
                  dc_log_printf("--PsenA retry move, %d", psenA_counts);
                  stateEject = 7;
              } else { // paper still blocking psensA
                  dc_log_printf("--Eject Done");
                  stateEject++;
              }
          }
          break;
      default:
          bDone = TRUE;
          break;
      }//switch
      tx_thread_sleep(2 * MM_PER_SEC_PPS / 8);// loop every 1/4 mm
    }//while

}// eject_test()

void read_Paper_test(void)
{
  UINT16 reading = 0;

  reading = ADC_readCurrentAvg(PSENA_CHANNEL, 1);
  dc_log_printf("ADC PSEN A: %d", reading);

  reading = ADC_readCurrentAvg(PSENB_CHANNEL, 1);
  dc_log_printf("ADC PSEN B: %d", reading);

  dc_log_printf("=mst:%d", GetMotorState() );
  dc_log_printf("=Ramp Rem:%d", ramp_cycles_remaining);
  dc_log_printf("=pos=%d", dcreg16(RA_LM_POSITION0_LO));

}


// smartsheet can not be jammed when this is callled
void smartsheet_test(void)
{
    STEPPER_CONFIG motor_config;
	MOTOR_DRIVER_CONFIG mdc;
    UINT16 PsenC_reading = 0;
    UINT16 PsenB_reading = 0;
    UINT16 minPsenB = 65535;
    
    DC_RETURN_CODE status = DC_FAILURE;

    BOOL bDone = FALSE;
    int stateSmart = 0;
    int barcode_found = 0;
    
    

    while (!bDone) {
        switch(stateSmart) {
        case 0:
            PsenC_reading = ADC_readCurrentAvg(PSENC_CHANNEL, 1);
            //ensure B not interrupted
            if ( PsenC_reading > PSENC_SMARTSHEET_GTLEVEL ) {
                dc_log_printf("==Start Bar Code, C:%d", PsenC_reading);
                stateSmart++;
            } else {
                dc_log_printf("==No SmartSheet present, C:%d", PsenC_reading);
                bDone = TRUE;
            }
            break;
        case 1: // start move to psenB
            init_motor_config(&motor_config);
            init_motor_driver_config(&mdc);
            
            //move to psenB
            stepperStart( 3 * MM_PER_SEC_PPS, // 3mm per sec running vel in PPS
                          (85+40) * MM_STEPS, // move 85mm + more for slippage
                          MOVE_FORWARD, NULL);
            stateSmart++;
            break;
        case 2:// wait arrival to psenB
            PsenB_reading = ADC_readCurrentAvg(PSENB_CHANNEL, 1);
            if ( (PsenB_reading < PSENB_PAPER_LTLEVEL)
                 ||(MTR_STATE_IDLE == GetMotorState()) ) {
                stateSmart++;
                dcreg16(RA_LM_POS0UPDATE_LO) = 0;
                dc_log_printf("==got to PSENB");
            }
            break;
        case 3:// wait 10mm for nip
          if ( (dcreg16(RA_LM_POSITION0_LO) > (10 * MM_STEPS))
                 ||(MTR_STATE_IDLE == GetMotorState()) ){
              stateSmart++;
              dc_log_printf("==Got past NIP:%d", dcreg16(RA_LM_POSITION0_LO));
              // trigger acquisition
              numBarCodeSamples = 0;
              flagBarcodeActive = 1;
            }
            break;
        case 4:// wait for barcode to finish
            if ( (dcreg16(RA_LM_POSITION0_LO) > (40 * MM_STEPS) )
                 || ( MTR_STATE_IDLE == GetMotorState() ) ) {
                flagBarcodeActive = 0;
                if (barcode_found) {
                    dc_log_printf("==Finished barcode");
                
                    dc_log_printf("==Bar Samples:%d", numBarCodeSamples);
                    dc_log_printf("==Barbuf:0x%x", &samplesBarCode);
                } else {
                    dc_log_printf("== BARCODE NOT FOUND?");
                }
                stateSmart++;
            }

            if (!barcode_found) {
                PsenB_reading = ADC_readCurrentAvg(PSENB_CHANNEL, 1);
                if(PsenB_reading < minPsenB) {
                    // finding white value
                    minPsenB = PsenB_reading;
                }
                // barcode found if there is a 20% slew from white
                if (PsenB_reading > (minPsenB+(65535-minPsenB)/5) ) {
                    barcode_found = 1;
                    dc_log_printf("==BARCODE FOUND");
                }
                dc_log_printf("==PSENB:%d", PsenB_reading);
            }
            
            break;
        case 5:// wait for tailing edge to pass psenB
            PsenB_reading = ADC_readCurrentAvg(PSENB_CHANNEL, 1);
            if ( (PsenB_reading > PSENB_NOPAPER_GTLEVEL)
                 ||(MTR_STATE_IDLE == GetMotorState()) ) {
                dc_log_printf("==Passed PSEN B: %d", PsenB_reading);
                SetMotorStop();
                dcreg16(RA_LM_POS0UPDATE_LO) = 0;
                stateSmart++;
            }
            break;
        case 6:// move back (1st wipe)
            stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                          1 * MM_STEPS,       // move 1mm
                          MOVE_BACKWARD, NULL);
            stateSmart++;
            break;
        case 7:// wait move complete
            if (MTR_STATE_IDLE == GetMotorState()) {
              stateSmart++;
            }
            break;
        case 8:// start move fwd (wipe 2)
            stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                          2 * MM_STEPS,       // move 2mm
                          MOVE_FORWARD, NULL);
            stateSmart++;
            break;
        case 9:// wait done
            if (MTR_STATE_IDLE == GetMotorState()) {
              stateSmart++;
            }
            break;
        case 10:// start move back (wipe 3)
            stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                          1 * MM_STEPS,       // move 1mm
                          MOVE_BACKWARD, NULL);
            stateSmart++;
            break;
        case 11:// wait done
            if (MTR_STATE_IDLE == GetMotorState()) {
                dc_log_printf("==WIPED");
                dcreg16(RA_LM_POS0UPDATE_LO) = 0;
                stateSmart++;
            }
            break;
        case 12:// drive tail out
            stepperStart( 2 * MM_PER_SEC_PPS, // 2mm per sec running vel, in PPS
                          50 * MM_STEPS,      // start 50mm move
                          MOVE_FORWARD, NULL);
            stateSmart++;
            break;
        case 13:// wait for sheet to finish
          if ( (dcreg16(RA_LM_POSITION0_LO) > (8 * MM_STEPS) ) 
                 || (MTR_STATE_IDLE == GetMotorState()) ) {
                SetMotorStop();
                stateSmart++;
            }
            break;
        case 14:// parse smartsheet data
            if (barcode_found) {
                status = zink_barcode_test(samplesBarCode, numBarCodeSamples);
                dc_log_printf("barcode test:%d", status);
            } else {
                dc_log_printf("Barcode Not Found");
            }
            stateSmart++;
             // fall thru to be done
        default:
            bDone = TRUE;
            break;
        }//switch

        tx_thread_sleep(45);
    }// while

} // smartsheet_test


/* This func is called by smartSheet() for TMD replacement */
DC_RETURN_CODE decode_barcode_func(void)
{
   DC_RETURN_CODE result = DC_SUCCESS;

   /* pz - Check if barcode Acquiring is done, and process barcode here */
   if ((gBarBuf) && (gBarCount > 512))
   {
      dc_log_printf(DC_LOG_NOTICE "zink_barcode_test launched");
      result = zink_barcode_test(gBarBuf, gBarCount);
      RESULT_RPT(result);
   }

   return result;
}

UINT8 checkPaperTray(void)
{
   UINT8 result = TRAY_IDLE;
   UINT16 psenA_voltage, psenB_voltage, psenC_voltage;
#ifndef FEATURE_BOARD_ONLY
   DSG_Engine_status = PRINT_STATE_PAPERFEED;

   /* Step 1.  EJECT MOTOR */
   while (1) //To check paper jam between power on and printing start..
   {
      psenA_voltage = readCurrentADC(PSENA_CHANNEL);
      psenB_voltage = readCurrentADC(PSENB_CHANNEL);

      dc_log_printf("[EJT]SEN_A - [%d], SEN_B - [%d]", psenA_voltage, psenB_voltage);
      if (trayAB_Stat(psenA_voltage, psenB_voltage) != PAPER_EXIST)
      {
         break;
      }
      else
      {
#if (0)        
         dc_log_printf("CALL EJTMTR!");
         SetAutoTimeFlag = 1;
         dcreg(RA_AC_SCRATCH20) = SPL_MOTOR_EJECT;

         while (dcreg(RA_AC_SCRATCH20) != 0x0)
         {
            tx_thread_sleep(10);
         }
#else
         eject_test();
         gReturnCode = TRAY_IDLE;
#endif         
         if (gReturnCode != TRAY_IDLE)
         {
            // Keith : Add here some notification to Phone if the jam is occurred after ejecting.
            //STOP AND NOTIFY
            //if(gReturnCode==TRAY_JAM)
            {
               result = TRAY_JAM;
               dc_log_printf( "Paper JAM!!");
               goto tray_exception;
            }
         }
      }
   }


   /* Step 2. SMART SHEET */
   while (1) // To call smartsheet process if it been inserted before print..
   {
      psenC_voltage = readCurrentADC(PSENC_CHANNEL);
      dc_log_printf("[EJT]SEN_C - [%d]", psenC_voltage);
      if (trayStat(psenC_voltage) == PAPER_EXIST)
      {
         break;
      }
      else if (trayStat(psenC_voltage) == PAPER_EMPTY)
      {
         result = TRAY_PAPER_EMPTY;
         ds_log_printf("Tray_paper_empty");
         goto tray_exception;
      }
      else
      {
         PowerLevelinPrint = ds_DisplayPwLevel();
         pPrintProcessFlag = 1;
         dc_log_printf("CALL SMTSHT!!");
         SetAutoTimeFlag = 1;
#if (0)
         dcreg(RA_AC_SCRATCH20) = SPL_MOTOR_SMART;
         while (dcreg(RA_AC_SCRATCH20) != 0x0)
         {
            tx_thread_sleep(10);
         }

         if (gReturnCode != TRAY_IDLE)
         {
            //STOP AND NOTIFY
            // Keith : Add here some notification to Phone if paper is mismatched.
            result = TRAY_JAM;
            break;
         }
#else
         smartsheet_test();
         gReturnCode = TRAY_IDLE;
#endif         
      }
   }


   /************************************************
     * To Check again status of the paper tray.    *
     * It could  notify "EMPTY PAPER" to the PHONE *
     ***********************************************/
tray_exception:
#endif
   DSG_Engine_status = PRINT_STATE_IDLE;
   //TMDC_COMPARE();
   //block 170207 ysb
   return result;
}


