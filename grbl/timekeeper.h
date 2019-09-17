#ifndef timekeeper_h
#define timekeeper_h

#include "grbl.h"
void signal_report_realtime_status();
void timekeeper_init();
void timekeeper_reset();

uint32_t get_timer_ticks();

#endif