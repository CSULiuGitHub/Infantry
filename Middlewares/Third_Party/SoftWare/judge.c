#include "judge.h"
#include "judge_tx.h"
#include "judge_rx.h"
#include "usart.h"
#include "cmsis_os.h"

#define JUDGE_FIFO_BUFLEN 500

uint8_t JudgeUI_lid_state;
uint8_t JudgeUI_spin_state;
uint8_t JudgeUI_vision_state;

static unpack_data_t judge_unpack_obj;
static uart_dma_rx_t judge_rx_obj;
static fifo_s_t judge_rxdata_fifo;
static uint8_t judge_rxdata_buf[JUDGE_FIFO_BUFLEN];

void Judge_InitData(void)
{
    fifo_s_init(&judge_rxdata_fifo, judge_rxdata_buf, JUDGE_FIFO_BUFLEN);

    judge_rx_obj.hdma_usart_rx = &hdma_usart3_rx;
    judge_rx_obj.p_fifo = &judge_rxdata_fifo;
    judge_rx_obj.buff_size = 1024;
    judge_rx_obj.buff[0] = uart3_m0_buf;
    judge_rx_obj.buff[1] = uart3_m1_buf;

    judge_unpack_obj.p_fifo = &judge_rxdata_fifo;
    judge_unpack_obj.p_header = (frame_header_t *)judge_unpack_obj.protocol_packet;
    judge_unpack_obj.index = 0;
    judge_unpack_obj.data_len = 0;
    judge_unpack_obj.unpack_step = STEP_HEADER_SOF;
	
	
	/*客户端UI初始化*/
	JudgeUI_lid_state='G';
	JudgeUI_spin_state='l';
	JudgeUI_vision_state='A';
}

static void Judge_Uart_Idle(void)
{
    dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT);
    judge_unpack_fifo_data(&judge_unpack_obj, SOF);    
}

static void Judge_Uart_UI(void)
{
    static judge_txpoll_e judge_txpoll = _Lid_St;
    uint8_t robot_id = JUDGE_u8GetRobotId();
    switch(judge_txpoll)
    {
        case _Lid_St:
            JudgeUI_LidState((uint16_t)(robot_id&0x00FF),JudgeUI_lid_state);
			osDelay(5);
            judge_txpoll++;
            break;
        case _Spin_Mode:
            JudgeUI_SpinMode((uint16_t)(robot_id&0x00FF),JudgeUI_spin_state);
			osDelay(5);
            judge_txpoll++;
            break;
        case _SuperCap_Data:
            JudgeUI_CapData((uint16_t)(robot_id&0x00FF));
			osDelay(5);
            judge_txpoll++;
            break;
//        case _HolderPitch_Data:
//            JudgeUI_PitchData((uint16_t)(robot_id&0x00FF));
//			osDelay(5);
//            judge_txpoll++;
//            break;
        case _BackGround1:
            JudgeUI_SendBKG1((uint16_t)(robot_id&0x00FF),1);
			osDelay(5);
            judge_txpoll++;
            break;
        case _BackGround2_SuperCap_Line:
            JudgeUI_BK2_CapLine((uint16_t)(robot_id&0x00FF)); 
			osDelay(5);
            judge_txpoll = _Visionz_Data;
            break;        
        case _Visionz_Data:
			JudgeUI_VisionzData((uint16_t)(robot_id&0x00FF)); 
			osDelay(5);
            judge_txpoll = _Vision_State;
            break;    
		case _Vision_State:
			JudgeUI_Vision_State((uint16_t)(robot_id&0x00FF), JudgeUI_vision_state);
			osDelay(5);
            judge_txpoll = _Lid_St;
            break;    
    }
}
void Judge_Proccess(void)
{
    uint8_t robot_id = JUDGE_u8GetRobotId();
    if(uart3_idle_flag == 1)
    {
        uart3_idle_flag = 0;
        Judge_Uart_Idle();
    }
    if(robot_id != 0)
    {
        Judge_Uart_UI();
    }
}


