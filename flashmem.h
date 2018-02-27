#ifndef _FlASHMEM_H_
#define _FlASHMEM_H_

#include "ch.h"
#include "hw_conf.h"

//Memory adresses
#define TOTAL_ACTIVEHOURS ((uint32_t)0x0004)
#define TOTAL_AH_CONS ((uint32_t)0x0008)

void flashmem_init(void);
void flashmem_update(void);

uint32_t flashmem_get_workinghours(void);
uint32_t flashmem_get_totalAmpHours(void);
void flashmem_write_workinghours(uint32_t value);
void flashmem_write_totalAmpHours(uint32_t value);

#endif /* _FlASHMEM_H_ */
