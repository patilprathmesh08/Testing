#ifndef CNR_OVERCURRENT_H
#define CNR_OVERCURRENT_H

#include <stdint.h>
#include <stdbool.h>


#define OVERCURRENT_FAULT_A        1.5f



void cnr_overcurrent_enable(float limit_a);   
void cnr_current_sign_fix_enable(void);

#endif /* CNR_OVERCURRENT_H */
