#include "grbl.h"

volatile uint32_t overflow_offset;
volatile uint32_t timer_tic_count_base;

// triggers a real time status report, used for debugging development
void signal_report_realtime_status()
{
  system_set_exec_state_flag(EXEC_STATUS_REPORT);		// initializes a real time report
}

// overflow interrupt for timekeeper - counts timer ticks beyond 8 bits
ISR(TIMER2_OVF_vect) {
  overflow_offset+=(1UL<<TIMER_COUNTER_WIDTH); //use of offset saves calculation time because get_timer_tics is called more often then the overflow interrupt
  //signal_report_realtime_status();
}
//initializes the timekeeper by enabling the timer2 overflow interrupt and resetting the timer
void timekeeper_init() {
  // Configure Timer 2: spindle timer for interrupt on overflow
  SPINDLE_TIMER_INTERRUPT_MASK_REGISTER|=SPINDLE_TIMER_INTERRUPT_MASK_INIT; //Enable the timer2 overflow
  TCNT2=0;                                                                  // reset the counter
  overflow_offset = 0;                                                      // set the offset to zero
  bit_false(TIFR2,(1<<TOV2));                                               // clear any pending interrupts
  timekeeper_reset();
}
//Reset the timekeeper so counting starts at zero
void timekeeper_reset()
{
  timer_tic_count_base=get_timer_ticks();
}
// Get the tics since the last reset
uint32_t get_timer_ticks_passed()
{
	return(get_timer_ticks()-timer_tic_count_base);
}
//Read the timer and check and correct is a overflow occured during reading
uint32_t get_timer_ticks() {
  uint32_t ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
	 ticks = ((uint32_t) TCNT2) + overflow_offset;				// read the timer register and add the overflow offset
	 if (bit_istrue(TIFR2,(1<<TOV2))) {							// during reading and calculating, a new timer overflow occurred that is not handled. read again and add 0x10000 to overflow_offset
       ticks = ((uint32_t) TCNT2) + overflow_offset + (1UL<<TIMER_COUNTER_WIDTH); // read again and add 0x10000 to overflow_offset
	 }
  }
  return ticks;
}

