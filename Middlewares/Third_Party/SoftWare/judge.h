#ifndef _JUDGE_H
#define _JUDGE_H

#include "judge_rx.h"
#include "dma_unpack.h"
#include "usart.h"

void Judge_InitData(void);
void Judge_Proccess(void);

/* Ħ���ֵȼ� */
extern uint8_t JudgeUI_lid_state;

/* С����״̬ */
extern uint8_t JudgeUI_spin_state;

/*���״̬*/
extern uint8_t JudgeUI_vision_state;
#endif
