/**
 * @file    fault.h
 * @brief   Last-resort stop for faults the firmware can't recover from.
 */
#ifndef FAULT_H
#define FAULT_H

/* Disables interrupts, switches the motors off and flashes the LEDs fast
 * forever. reason and detail are kept in faultReason / faultDetail so the
 * debugger can show them. */
void fault_halt(const char *reason, const char *detail) __attribute__((noreturn));

#endif /* FAULT_H */
