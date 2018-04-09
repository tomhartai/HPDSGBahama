
// snoop.h

//
// DEFINITIONS
//
#define MOVE_FORWARD 1
#define MOVE_BACKWARD 2


#define PSENA_PAPER_LTLEVEL 27000   // paper    at PSENA if < this #
#define PSENA_NOPAPER_GTLEVEL 29000 // no paper at PSENA if > this #

#define PSENB_PAPERWMARK_GTLEVEL 32000
#define PSENB_PAPER_LTLEVEL 40000   // smartsheet orange present if  < this #
#define PSENB_NOPAPER_GTLEVEL 52000 // no paper if > this #
#define PSENB_BLACKSS_GTLEVEL 60000 // smartshet black present if > this #

#define PSENC_SMARTSHEET_GTLEVEL 50000 // smartsheet at PSENC if > this #

#define STEPS_PER_MM          351
#define MM_PER_SEC_PPS  STEPS_PER_MM // mult mm per sec by this to get corresponding pulses per sec (PPS)
#define MM_STEPS        STEPS_PER_MM // mult mm by this to get steps


//
// PROTOTYPES
//

//
// stepperStart()
//    ppsStart - starting step rate (pulses per sec)
//               if 0 then will ramp down to 0 after 'steps' pulses
//    ppsRun   - running step rate (pulses per sec)
//    direction- MOVE_FORWARD or MOVE_BACKWARD
//    pBuffer - 2 * ( ( ((pStepper_config->steps_in_ramp * 2)/MCx_WRITE_SIZE) + 1 ) * 2 ) * MCx_WRITE_SIZE
//              + MCx_WRITE_SIZE
//              e.g. for step_in_ramp == 48, pBuffer m/b >= 2 * 256 + 64 == 576
//
//    note: while motor is 'running' i.e. at ppsRun rate, stepperStart() can be called again with
//          a different buffer to ramp to the new velocity and run there.
//

//DC_RETURN_CODE stepperStart( int ppsStart, int ppsRun, UINT16 steps, int direction, UINT8 *pBuffer);

DC_RETURN_CODE stepperStart(int ppsRun,
                             UINT16 steps,
                             int direction,
                             void (*call_back)(int)
                             );

void smartsheet_test(void);

void eject_test(void);
