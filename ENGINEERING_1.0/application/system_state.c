#include "system_state.h"

infantry_state_e infantry_state;

void system_state_set(infantry_state_e state)
{
    infantry_state = state;
}

infantry_state_e system_state_return(void)
{
    return infantry_state;
}
