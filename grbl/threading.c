/*
  threading.c - Handles threading command G33
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

volatile uint8_t threading_exec_flags;                // Real time executor bitflag variable for spindle synchronization.
volatile uint32_t threading_step_pulse_count;         // Step pulse counter
volatile uint8_t threading_sync_pulse_count;          // Synchronization pulse counter
volatile uint8_t threading_index_pulse_count;         // index pulse counter
volatile uint32_t threading_sync_Last_timer_tics;     // Time at last sync pulse
volatile uint32_t threading_sync_timer_tics_passed;   // Time passed sync pulse
volatile uint32_t threading_index_Last_timer_tics;    // Time at last index pulse
volatile uint32_t threading_index_timer_tics_passed;  // Time passed index pulse
volatile uint32_t spindle_rpm;                        // The spindle speed calculated from the spindle index pulses. Used for displaying the real spindle speed.
volatile float threading_synchronization_millimeters_error;     // The threading feed error calculated at every synchronization pulse
float threading_mm_per_synchronization_pulse;         // Z-axis motion at each sync pulse. Is not declared as volatile because it is not updated by an ISR routine.
float threading_mm_per_index_pulse;                   // Z-axis motion at each index pulse. Is not declared as volatile because it is not updated by an ISR routine.
float threading_feed_rate_calculation_factor;         // Factor is used in plan_compute_profile_nominal_speed() and is calculated at threading start for performance reasons.

// Initializes the G33 threading pass by resetting the timer, spindle counter,
// setting the current z-position as reference and calculating the (next) target position.
void threading_init(float K_value)
{
	threading_mm_per_synchronization_pulse= K_value / (float) settings.sync_pulses_per_revolution;								// Calculate the global mm feed per synchronization pulse value.
	threading_mm_per_index_pulse= K_value;																						// Calculate the global mm feed per synchronization pulse value.
	threading_feed_rate_calculation_factor  = ((float) TIMER_TICS_PER_MINUTE / (float) settings.sync_pulses_per_revolution);	// Calculate the factor to speedup the planner during threading
	timekeeper_reset();																											// Reset the timekeeper to avoid calculation errors when timer overflow occurs (to be sure)
	threading_reset();																											// Sets the target position to zero and calculates the next target position
}
// Reset variables to start the threading
void threading_reset()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    threading_index_pulse_count=0;
    threading_sync_pulse_count=0;
    system_clear_threading_exec_flag(0xff);		// Clear all the bits to avoid executing
  }
}

//Returns the time in tics since the last index pulse
//To avoid updates while reading, it is handle atomic
uint32_t timer_tics_passed_since_last_index_pulse()
{
	uint32_t tics;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){tics= get_timer_ticks()-threading_index_Last_timer_tics;}				// Use atomic to avoid errors due to timer updates
	return tics;
}

// This calculated spindle speed is used for showing the actual spindle speed in the report. It is not done in irq time
void calculate_spindle_rpm()
{
    spindle_rpm = TIMER_TICS_PER_MINUTE / threading_index_timer_tics_passed;	// calculate the spindle speed  at this place (not in the report) reduces the CPU time because a GUI will update more frequently
}
// This routine processes the spindle index pin hit by increasing the index pulse counter and calculating the time between pulses
// This calculated spindle speed is used for showing the actual spindle speed in the report
void process_spindle_index_pulse()
{
	threading_index_timer_tics_passed=get_timer_ticks()-threading_index_Last_timer_tics;		// Calculate the time between index pulses
	threading_index_Last_timer_tics+=threading_index_timer_tics_passed;							// adjust for calculating the next time
	threading_index_pulse_count++;																// Increase the pulse count
  system_set_threading_exec_flag(EXEC_SPINDLE_INDEX_PULSE);	// pin is index pulse
}

// Processes the synchronization pulses by increasing the synchronization counter and calculating the time between the synchronization pulses
void process_spindle_synchronization_pulse()
{
	threading_sync_timer_tics_passed=get_timer_ticks()-threading_sync_Last_timer_tics;		// Calculate the time between synchronization pulses
	threading_sync_Last_timer_tics+=threading_sync_timer_tics_passed;						          // adjust for calculating the next time
	threading_sync_pulse_count++;															                            // Increase the synchronization pulse count
	system_set_threading_exec_flag(EXEC_PLANNER_SYNC_PULSE);	                            // Signal the detection of a synchronization pulse.
}

// This routine does the processing needed to keep the Z-axis in sync with the spindle during a threading pass G33
// This is done only, if the current planner block is a G33 motion indicated by the planner condition PL_COND_FLAG_FEED_PER_REV
// Recalculates the feed rates for all the blocks in the planner as if a new block was added to the planner que
// If the current block isn't a G33 motion, the synchronization_millimeters_error will be set to zero
void update_planner_feed_rate() {
  plan_block_t *plan = plan_get_current_block();
  if ((plan!=NULL) && (plan->condition & PL_COND_FLAG_FEED_PER_REV)) {	// update only during threading
    plan_update_velocity_profile_parameters();							// call plan_compute_profile_nominal_speed() that wil calculate the requested feed rate
	plan_cycle_reinitialize();											// update the feed rates in the blocks
  }
  else threading_synchronization_millimeters_error=0;								// set the error to zero so it can be used in reports
}

// returns true if Spindle sync is active otherwise false
bool spindle_synchronization_active()
{
	plan_block_t *plan = plan_get_current_block();
	if ((plan!=NULL) && (plan->condition & PL_COND_FLAG_FEED_PER_REV)) return true;
	return false;
}

// Returns true if the index pulse is active
bool index_pulse_active()
{
  return (limits_get_state(LIMIT_MASK_Y_AXIS)); 	// This is the lathe version, Y-axis limit pin hits are spindle index pulses so handle them and do not reset controller
}

// Returns true if the sync pulse is active
bool sync_pulse_active()
{
  uint8_t pin = system_control_get_state();
  if (settings.sync_pulses_per_revolution==1)										  // If there is just an index pulse, return the index pulse status.
    return index_pulse_active();
	return (bit_istrue(pin,CONTROL_PIN_INDEX_SPINDLE_SYNC)); 				// Detected a G33 spindle synchronization pulse. Beware, this is not the spindle index pulse
}