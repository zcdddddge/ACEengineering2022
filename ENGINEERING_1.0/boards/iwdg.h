#ifndef __WDG_H
#define __WDG_H
#include "struct_typedef.h"

void iwdg_init(u8 prer, u16 rlr);
void iwdg_feed(void);

#endif
