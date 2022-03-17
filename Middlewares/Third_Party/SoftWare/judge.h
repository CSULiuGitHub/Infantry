#ifndef _JUDGE_H
#define _JUDGE_H

#include "judge_rx.h"
#include "dma_unpack.h"
#include "usart.h"

void Judge_InitData(void);
void Judge_Proccess(void);

/* Ä¦²ÁÂÖµÈ¼¶ */
extern uint8_t JudgeUI_lid_state;

/* Ğ¡ÍÓÂİ×´Ì¬ */
extern uint8_t JudgeUI_spin_state;

/*´ò·û×´Ì¬*/
extern uint8_t JudgeUI_vision_state;
#endif
