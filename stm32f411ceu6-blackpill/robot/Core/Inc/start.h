#ifndef _START_H
#define _START_H

#include <stdint.h>


/* waits timeout_ms milliseconds for the start button to be pressed.
 * Returns the remaining timeout (0 if it timed-out before the button
 * was pressed) */
uint32_t wait_for_start(uint32_t timeout_ms);

#endif /* _START_H */
