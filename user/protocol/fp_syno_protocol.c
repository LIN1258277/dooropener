/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "fp_syno_protocol.h"
#include "drive_usart.h"
#include "cpu.h"
#include "syn_pro_task.h"

#define LOG_TAG     "[fp_pro]"
#define LOG_ERROR_ENABLE
#define LOG_DEBUG_ENABLE
#define LOG_INFO_ENABLE
#define LOG_DUMP_ENABLE
#define LOG_CLI_ENABLE
#include "debug.h"
const unsigned char syno_fixed_cmd_match_capture_image[]  = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x03, 0x01, 0x00, 0x05};
const unsigned char syno_fixed_cmd_enroll_capture_image[] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x03, 0x29, 0x00, 0x2D};
const unsigned char syno_fixed_cmd_general_templete[]     = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x03, 0x05, 0x00, 0x09};
const unsigned char syno_variable_cmd_read_index_table[]  = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x04, 0x1F, 0x00, 0x00, 0x00};
const unsigned char syno_fixed_cmd_empty_templete[]       = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x03, 0x0D, 0x00, 0x11};
const unsigned char syno_fixed_cmd_sleep[]                = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x03, 0x33, 0x00, 0x37};

const unsigned char syno_variable_cmd_general_extract[]   = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00};
const unsigned char syno_variable_cmd_search_templete[]   = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x08, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char syno_variable_cmd_store_templete[]    = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00};

const unsigned char syno_variable_cmd_del_templete[]      = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x07, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

const unsigned char syno_rgb_ctr[]      = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0x00, 0x08, SYNO_CMD_RGB_CTRL, FP_SYNO_RGB_BREATH, FP_SYNO_RGB_COLOR_B, FP_SYNO_RGB_COLOR_B,0x00,0x0f,0x00,0x00};
//const u8 handshake_cmd[]={0xef,0x01,0xff,0xff,0xff,0xff,0x07,0x00,0x03,0x00,0x00,0x0a};
//#ifdef FP_SYNO_AUTO_MATCH_MOD
//const unsigned char syno_variable_cmd_auto_search[]= {0xEF,0x01,0xFF,0xFF,0xFF,0xFF,0x01,0x00,0x08,0x32,0x02,0xFF,0xFF,0x00,0x04,0x02,0x3F };// 1:N自动验证,不返回关键步骤,安全等级2
//#endif
void pend_syn_cmd_sem(u8 pend_post);
static uint16_t fp_syno_cmd_cal_sum(uint8_t *data, uint16_t size) {
	uint16_t sum = 0;

	for (int pos=0; pos<size; pos++) {
		sum += data[pos];
	}

	return (uint16_t)sum;
}



 /* 进入休眠
 */
void fp_syno_cmd_into_sleep(void) {
    fp_op_info.cmd_size = sizeof(syno_fixed_cmd_sleep);
    memcpy(fp_op_info.cmd_buf, syno_fixed_cmd_sleep, fp_op_info.cmd_size);

    fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_INTO_SLEEP);
	fp_op_info.op_timeout 	= FP_SYNO_SLEEP_TIMEOUT;

}



//static void cmd_send_timeout_call(void)
//{
//	printf("cmd_send_timeout_call\n");
//	fp_op_info.send_cmd_now=0;
//	fp_op_info.fp_rgb_cmd_timer=0;
//}
//void del_rgb_cmd_timeout_timer(void)
//{
//	printf("del_rgb_cmd_timeout_timer\n");
//	if(fp_op_info.fp_rgb_cmd_timer){
//		sys_timeout_del(fp_op_info.fp_rgb_cmd_timer);
//		fp_op_info.fp_rgb_cmd_timer=0;
//		fp_op_info.send_cmd_now=0;
//	}
//}
//void add_rgb_cmd_timeout_timer(void)
//{
//	fp_op_info.fp_rgb_cmd_timer=sys_timeout_add(NULL,cmd_send_timeout_call,200);
//}
void os_time_dly(int time_tick);

 /* 灯控
 */
void fp_syno_cmd_rgb_ctrl(ENUM_FP_SYNO_RGB_FUNC_CODE func_code,ENUM_FP_SYNO_RGB_COLOR s_color,uint8_t e_color_duty,uint8_t loop_times,uint8_t cycle) {
	uint16_t sum;
	os_time_dly(1);//让FP_CONTINUE_SND_PKT_EVENT先执行再赋值cmd_size和cmd_buf(去掉后设置键恢复出厂半路松开后rgb有问题)
	fp_op_info.cmd_size = sizeof(syno_rgb_ctr);
	memcpy(fp_op_info.cmd_buf, syno_rgb_ctr, fp_op_info.cmd_size);
	fp_op_info.cmd_buf[VARIABLE_FILED_START_POS]=func_code;
	fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+1]=s_color;
	fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+2]=e_color_duty;
	fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+3]=loop_times;
	fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+4]=cycle;
	sum = fp_syno_cmd_cal_sum(&fp_op_info.cmd_buf[CALC_SUM_START_POS], (fp_op_info.cmd_size - 8));
    fp_op_info.cmd_buf[fp_op_info.cmd_size-2] = (uint8_t)(sum >> 8);
    fp_op_info.cmd_buf[fp_op_info.cmd_size-1] = (uint8_t)(sum);
	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_RGB_CTRL);
	fp_op_info.op_timeout 	= FP_SYNO_RGBCTRL_TIMEOUT;
//	if(fp_op_info.send_cmd_now)
//	{
//		add_rgb_cmd_timeout_timer();
//		void usart_send_buf(uint8_t* buf,uint32_t len);
//		usart_send_buf(fp_op_info.cmd_buf, fp_op_info.cmd_size);		
		////printf("fp_syno_cmd_rgb_ctrl wait ack!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//		while(fp_op_info.send_cmd_now);//确保这条指令应答不被FP_RCV_PKT_EVENT接收(没有经历FP_CONTINUE_SND_PKT_EVENT所以也不能经历FP_RCV_PKT_EVENT)
		////printf("fp_syno_cmd_rgb_ctrl over!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//	}
//	else 
//	log_info("cmdsize0:%d\n",fp_op_info.cmd_size);
	if(fp_op_info.sleep_states)fp_op_info.rgb_cmd_cache=1;
	else tysl_mul_fp_sched_event_post_only_evt(FP_CONTINUE_SND_PKT_EVENT);//休眠唤醒时先等指纹模组启动成功在发灯控指令(app蓝牙开锁，app恢复出厂，设置键常开切换等)
//	log_info("cmdsize1:%d\n",fp_op_info.cmd_size);
}

 /* 采集指纹图像
 */
void fp_syno_cmd_capture_image(uint8_t operate_cmd) {
    if (operate_cmd == SYNO_CMD_ENROLL_GETIMAGE) {
        fp_op_info.cmd_size = sizeof(syno_fixed_cmd_enroll_capture_image);
        memcpy(fp_op_info.cmd_buf, syno_fixed_cmd_enroll_capture_image, fp_op_info.cmd_size);
    } else {
        fp_op_info.cmd_size = sizeof(syno_fixed_cmd_match_capture_image);
        memcpy(fp_op_info.cmd_buf, syno_fixed_cmd_match_capture_image, fp_op_info.cmd_size);
    }
	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_CAPTURE_IMAGE);
	fp_op_info.op_timeout 	= FP_SYNO_CAPTURE_TIMEOUT;
}

 /* 生成特征
 */
static void fp_syno_cmd_general_extract(uint8_t buffer_id) {
    uint16_t sum;

    fp_op_info.cmd_size = sizeof(syno_variable_cmd_general_extract);
    memcpy(fp_op_info.cmd_buf, syno_variable_cmd_general_extract, fp_op_info.cmd_size);

    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS] = buffer_id;
    sum = fp_syno_cmd_cal_sum(&fp_op_info.cmd_buf[CALC_SUM_START_POS], (fp_op_info.cmd_size - 8));
    fp_op_info.cmd_buf[fp_op_info.cmd_size-2] = (uint8_t)(sum >> 8);
    fp_op_info.cmd_buf[fp_op_info.cmd_size-1] = (uint8_t)(sum);

	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_GENERAL_EXTRACT);

	fp_op_info.op_timeout 	= FP_SYNO_COMMON_TIMEOUT;
}

 /* 生成模板
 */
void fp_syno_cmd_general_templete(void) {
    fp_op_info.cmd_size = sizeof(syno_fixed_cmd_general_templete);
    memcpy(fp_op_info.cmd_buf, syno_fixed_cmd_general_templete, fp_op_info.cmd_size);

	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_GENERAL_TEMPLETE);

	fp_op_info.op_timeout 	= FP_SYNO_COMMON_TIMEOUT;
}

 /* 存储模板
 */
static void fp_syno_cmd_store_templete(uint8_t buffer_id, uint16_t page_id) {
    uint16_t sum;

    fp_op_info.cmd_size = sizeof(syno_variable_cmd_store_templete);
    memcpy(fp_op_info.cmd_buf, syno_variable_cmd_store_templete, fp_op_info.cmd_size);

    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+0] = buffer_id;
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+1] = (uint8_t)(page_id >> 8);
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+2] = (uint8_t)(page_id);
    sum = fp_syno_cmd_cal_sum(&fp_op_info.cmd_buf[CALC_SUM_START_POS], (fp_op_info.cmd_size - 8));
    fp_op_info.cmd_buf[fp_op_info.cmd_size-2] = (uint8_t)(sum >> 8);
    fp_op_info.cmd_buf[fp_op_info.cmd_size-1] = (uint8_t)(sum);

	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_STORE_TEMPLETE);

	fp_op_info.op_timeout 	= FP_SYNO_COMMON_TIMEOUT;
}

 /* 1:N匹配
 */
static void fp_syno_cmd_match1n(uint8_t buffer_id, uint16_t start_page, uint16_t page_num) {
    uint16_t sum;

    fp_op_info.cmd_size = sizeof(syno_variable_cmd_search_templete);
    memcpy(fp_op_info.cmd_buf, syno_variable_cmd_search_templete, fp_op_info.cmd_size);

    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+0] = buffer_id;
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+1] = (uint8_t)(start_page >> 8);
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+2] = (uint8_t)(start_page);
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+3] = (uint8_t)(page_num >> 8);
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+4] = (uint8_t)(page_num);
    sum = fp_syno_cmd_cal_sum(&fp_op_info.cmd_buf[CALC_SUM_START_POS], (fp_op_info.cmd_size - 8));
    fp_op_info.cmd_buf[fp_op_info.cmd_size-2] = (uint8_t)(sum >> 8);
    fp_op_info.cmd_buf[fp_op_info.cmd_size-1] = (uint8_t)(sum);

	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_MATCH1N);

	fp_op_info.op_timeout 	= FP_SYNO_MATCH_TIMEOUT;//注意超过30枚版本时间要加大
}

 /* 删除模板
 */
void fp_syno_cmd_del_templete(uint16_t templete_id, uint16_t templete_nums) {
    uint16_t sum;

    fp_op_info.cmd_size = sizeof(syno_variable_cmd_del_templete);
    memcpy(fp_op_info.cmd_buf, syno_variable_cmd_del_templete, fp_op_info.cmd_size);

    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+0] = (uint8_t)(templete_id >> 8);
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+1] = (uint8_t)(templete_id);
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+2] = (uint8_t)(templete_nums >> 8);
    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+3] = (uint8_t)(templete_nums);
    sum = fp_syno_cmd_cal_sum(&fp_op_info.cmd_buf[CALC_SUM_START_POS], (fp_op_info.cmd_size - 8));
    fp_op_info.cmd_buf[fp_op_info.cmd_size-2] = (uint8_t)(sum >> 8);
    fp_op_info.cmd_buf[fp_op_info.cmd_size-1] = (uint8_t)(sum);
    log_info("[mul] fp del_templete, id=[%d], nums=[%d]\r\n", templete_id, templete_nums);

	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_ERASE_ONE);

	fp_op_info.op_timeout 	= FP_SYNO_COMMON_TIMEOUT;
}

 /* 清空模板
 */
void fp_syno_cmd_empty_templete(void) {
    fp_op_info.cmd_size = sizeof(syno_fixed_cmd_empty_templete);
    memcpy(fp_op_info.cmd_buf, syno_fixed_cmd_empty_templete, fp_op_info.cmd_size);

	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_ERASE_ALL);

	fp_op_info.op_timeout 	= FP_SYNO_EMPTY_ACK_TIMEOUT;
}
#include "password.h"



/* 获取可用的指纹id
 */
static void fp_syno_cmd_get_id_availability(uint8_t index) {
    uint16_t sum;

    fp_op_info.cmd_size = sizeof(syno_variable_cmd_read_index_table);
    memcpy(fp_op_info.cmd_buf, syno_variable_cmd_read_index_table, fp_op_info.cmd_size);

    fp_op_info.cmd_buf[VARIABLE_FILED_START_POS+0] = (uint8_t)(index);
    sum = fp_syno_cmd_cal_sum(&fp_op_info.cmd_buf[CALC_SUM_START_POS], (fp_op_info.cmd_size - 8));
    fp_op_info.cmd_buf[fp_op_info.cmd_size-2] = (uint8_t)(sum >> 8);
    fp_op_info.cmd_buf[fp_op_info.cmd_size-1] = (uint8_t)(sum);

	fp_op_info.cmd_type 	= (EUNM_FP_CMD_TYPE)(FP_GET_ID_AVAILABILITY);

	fp_op_info.op_timeout 	= FP_SYNO_COMMON_TIMEOUT;
}

// /* 搜索.寻找可用的指纹id
// */
static int fp_syno_search_availability_id(uint8_t *index_table, uint8_t size, uint16_t *id) {
    uint8_t tmp;
    int index, bit, ret = -1;
    uint16_t available_id = 0xFFFF;

    for (index=0; index<size; index++) {
		for (bit=0; bit<8; bit++) {
			tmp = (1 << bit);
			if ((index_table[index] & tmp) == 0) {
				available_id = (8*index) + bit;
                ret = 0;
				break;
			}
		}

		if (ret != -1)
			break;
	}

    // Available id rang: 0~~(FP_SYNO_SUPPORT_ENROLL_NUMS-1)
    if (available_id >= FP_SYNO_SUPPORT_ENROLL_NUMS)
        ret = -1;
    else
        *id = available_id;

    log_info("[mul] fp search availability id, ret=[%d], id=[%d]\r\n", ret, available_id);
    return ret;
}

//#endif
 /* 重置默认命令-匹配
 */
//void fp_syno_reset_default_cmd(void) {
//    fp_op_info.work_mode 		= FP_WKM_MATCH;
//	fp_op_info.match_retry 		= 0;

//	fp_op_info.sleep_cmd_retry=0;
//	fp_syno_cmd_capture_image(SYNO_CMD_MATCH_GETIMAGE);
//}

 /* 重置协议解析
 */
void fp_syno_reset_protocol_parse(void) {
	fp_rcv_info.sum = 0;
	fp_rcv_info.rcv_size = 0;
	fp_rcv_info.rcv_pkg_dlen = 0;
	fp_rcv_info.rcv_fsm_state = FP_SYNO_RCV_FIRST_HEAD;
}



#include "os/os_api.h"
extern u16 g_sys_timer_cnt;
void del_wait_fp_up_timer(void);
//void led_light_off(uint8_t mode);
//void add_resend_msg_timer(fp_msg_t *fp_report);
uint8_t judge_enroll_by_app(void);
uint8_t judge_factory_status(void);
//void retry_cap_fail_fun(void )
//{
//	fp_msg_t fp_report={.op_cmd=FP_INTO_SLEEP,0,0,0};
//	printf("retry cap fail\n");
//	if(judge_enroll_by_app()&&fp_op_info.work_mode == FP_WKM_ENROLL)common_send_tuya_msg(FP_INTO_SLEEP,fp_report.user_id);
//	else add_resend_msg_timer(&fp_report);
//}
void send_fp_press_event(void)
{
	os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, FP_FINGER_PRESS_EVENT);
}
static void query_finger_call(void)
{
	if(fp_op_info.query_finger_stu==FP_QUERY_FINGER)send_fp_press_event();//tysl_mul_fp_sched_event_post_only_evt(FP_FINGER_PRESS_EVENT);
}

void add_query_finger_timer(void)
{
    if(!fp_op_info.communication_timeout_timer)fp_op_info.communication_timeout_timer=sys_timer_add(NULL,query_finger_call,300);
    else sys_timer_modify(fp_op_info.communication_timeout_timer,300);//采图失败后300ms后继续采图
}
void del_query_finger_timer(void)
{
    if(fp_op_info.communication_timeout_timer)
    {
    	sys_timer_del(fp_op_info.communication_timeout_timer);
    	fp_op_info.communication_timeout_timer=0;
    }
}

void tysl_pdl_fp_uart_send(uint8_t *data, uint16_t size) {
	if(FP_RGB_CTRL 	!= fp_op_info.cmd_type&&FP_GO_SLEEP!=fp_op_info.query_finger_stu)fp_op_info.query_finger_stu=FP_WAIT_ACK;//当处于查询手指状态时发送任何指令之前应该先清为等待应答状态
	#ifdef CONFIG_DEBUG_ENABLE
	if(get_pend_cmd_sem()==0)
	{
		printf("cmd again!!!!!!!!!!!!!!!:%x,%x,%d\n",fp_op_info.ack_cmd,fp_op_info.cmd_buf[CMD_CODE_START_POS],fp_op_info.cmd_size);
	}
	#endif
	pend_syn_cmd_sem(1);
	fp_op_info.ack_cmd=fp_op_info.cmd_buf[CMD_CODE_START_POS];
	USART_PutString(data,size);
	#ifdef CONFIG_DEBUG_ENABLE
	if(SYNO_CMD_MATCH_GETIMAGE!=fp_op_info.ack_cmd&&SYNO_CMD_ENROLL_GETIMAGE!=fp_op_info.ack_cmd)
	{
		log_info("fp uart send:%x",fp_op_info.ack_cmd);
		log_info_hexdump( data, size);
	}
	#endif
	if(FP_GO_SLEEP==fp_op_info.query_finger_stu)fp_op_info.query_finger_stu=FP_WAIT_ACK;
}
void tysl_mul_fp_sched_event_post_only_evt(uint8_t cmd)
{
	os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, cmd);
	if(FP_CONTINUE_SND_PKT_EVENT==cmd&&(strcmp(os_current_task(), YZ_MAIN_TASK_NAME) == 0))os_time_dly(1);//防止上条(比如灯控）指令还没发出cmd_buf就重新被赋新值从而导致上条指令发送错误
}
void tysl_mul_fp_sched_event_post_with_data(uint8_t cmd,void *data,uint8_t len)
{
	os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 3,cmd,data,len);
}
 /* 协议解析
 */
void fp_syno_protocol_parse(uint8_t *p_data, uint16_t len) {
    uint8_t data;

    for (int i=0; i<len; i++) {
        data = p_data[i];

    	switch (fp_rcv_info.rcv_fsm_state) {
    		case FP_SYNO_RCV_FIRST_HEAD:
    			if (data == FP_SYNO_FIRST_HEAD) {
    				fp_rcv_info.sum = 0;
    				fp_rcv_info.rcv_size = 0;
    				fp_rcv_info.rcv_pkg_dlen = 0;
    				fp_rcv_info.rcv_buf[fp_rcv_info.rcv_size++] = data;
    				fp_rcv_info.rcv_fsm_state = FP_SYNO_RCV_SECOND_HEAD;
    			}
    			break;

            case FP_SYNO_RCV_SECOND_HEAD:
                if (data == FP_SYNO_SECOND_HEAD) {
                    fp_rcv_info.rcv_buf[fp_rcv_info.rcv_size++] = data;
                    fp_rcv_info.rcv_fsm_state = FP_SYNO_RCV_PKG_SIZE;
                } else {
                    fp_rcv_info.rcv_size = 0;
    				fp_rcv_info.rcv_fsm_state = FP_SYNO_RCV_FIRST_HEAD;
                }
    			break;

    		case FP_SYNO_RCV_PKG_SIZE:
    			fp_rcv_info.rcv_buf[fp_rcv_info.rcv_size++] = data;
    			if (fp_rcv_info.rcv_size >= 9) {
    				fp_rcv_info.rcv_pkg_dlen = ((fp_rcv_info.rcv_buf[7] << 8) + fp_rcv_info.rcv_buf[8]);

    				fp_rcv_info.rcv_fsm_state = FP_SYNO_RCV_DATAS;
    			}
    			break;

    		case FP_SYNO_RCV_DATAS:
    			fp_rcv_info.rcv_buf[fp_rcv_info.rcv_size++] = data;
    			if (fp_rcv_info.rcv_size >= (9 + fp_rcv_info.rcv_pkg_dlen)) {
                    fp_rcv_info.rcv_pkg_dlen = 0;
    				fp_rcv_info.sum = fp_syno_cmd_cal_sum(&fp_rcv_info.rcv_buf[CALC_SUM_START_POS], (fp_rcv_info.rcv_size-8));
    				if (fp_rcv_info.rcv_buf[fp_rcv_info.rcv_size-2] == (uint8_t)(fp_rcv_info.sum >> 8) && \
                        fp_rcv_info.rcv_buf[fp_rcv_info.rcv_size-1] == (uint8_t)(fp_rcv_info.sum)) {
//                        if(fp_op_info.send_cmd_now)del_rgb_cmd_timeout_timer();
//						else
						{
							fp_op_info.sleep_states=0;
	                        fp_pwr_supply_timeout_stop();//中断唤醒后等待应答采图，如果应答则关闭等待超时定时器
	                        fp_uart_timeout_stop();
//							tysl_mul_fp_sched_event_post_with_data(FP_RCV_PKT_EVENT, fp_rcv_info.rcv_buf, fp_rcv_info.rcv_size);//测到一次收到数据时被app打断导致这里回调的执行延迟了很久，导致通信超时回调被执行
							fp_syno_pkg_handle(fp_rcv_info.rcv_buf,fp_rcv_info.rcv_size);
							
							if(fp_op_info.rgb_cmd_cache){
								fp_op_info.rgb_cmd_cache=0;
								log_info("rgb_cmd_cache!!!!\n");
								os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1,FP_CONTINUE_SND_PKT_EVENT);
							}
							else if(fp_op_info.msg_cache[0])
							{
								log_info("msg_cache!!!!:%d\n",fp_op_info.msg_cache[0]);
								os_taskq_post_event(YZ_MAIN_TASK_NAME, 3, fp_op_info.msg_cache[0], fp_op_info.msg_cache[1], fp_op_info.msg_cache[2]);//恢复发起模组启动应答采图失败之前产生的事件
								fp_op_info.msg_cache[0]=0;
							}
							
						}
    				} else {
                        log_info("[mul] fp sum err, calc->[%04X]\r\n", fp_rcv_info.sum);
    				}

    				fp_syno_reset_protocol_parse();
    			}
    			break;

    		default:
    			fp_rcv_info.rcv_fsm_state = FP_SYNO_RCV_FIRST_HEAD;
    			break;
    	}

    	if (fp_rcv_info.rcv_size >= FP_COMMUNICATE_BUF_MAXSIZE)
    		fp_syno_reset_protocol_parse();
    }
}
fp_msg_t fp_report;
 /* 数据包处理
 */
void fp_syno_pkg_handle(uint8_t *data, uint16_t size) {
    (void)(size);
    uint16_t page_id;
	fp_syno_pkg_list_t *pkg = (fp_syno_pkg_list_t *)data;
//    fp_msg_t fp_report = {0,0,0,0,0};
	uint16_t serch_score=0;
	uint8_t ack_cmd=fp_op_info.ack_cmd;
	pend_syn_cmd_sem(0);//解析完对应ack命令并处理之前post
    fp_report.op_result = ((pkg->data[0]) == SYNO_ACK_SUCCESS) ? (0) : (1);
	if(fp_op_info.query_finger_stu==FP_GO_SLEEP){printf("ack cmd:%x before sleep\n",ack_cmd);ack_cmd=0xff;}
		
		
	
	switch (ack_cmd) {
		case SYNO_CMD_MATCH_GETIMAGE:               // 验证用获取图像
			if(fp_report.op_result){
				if(!get_setkeywarn_flag()&&(fp_op_info.query_finger_stu==FP_WAIT_ACK))
				{
					fp_op_info.query_finger_stu=FP_QUERY_FINGER;//采图失败继续采图定时器扫描或者中断检测否则等松开手指后再恢复采图定时器扫描和中断检测
					add_query_finger_timer();
				}
			}
			if (fp_report.op_result == SYNO_ACK_SUCCESS) {
                g_sys_timer_cnt=0;
				fp_op_info.msg_cache[0]=0;
				fp_op_info.rgb_cmd_cache=0;
				if(fp_op_info.fp_hold_flag!=FP_SUSPEND)
				{
					printf("go extract by match capture\n");
					fp_syno_cmd_general_extract(1);
				}
				#ifdef CONFIG_DEBUG_ENABLE
				else printf("finger hold!!!!\n");
				#endif
				os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, FP_CONTINUE_SND_PKT_EVENT);
            }
			else if(SYNO_ACK_ERR_NO_FINGER==pkg->data[0]&&fp_op_info.fp_hold_flag==FP_SUSPEND)//注册期间松开继续下次注册
			{
				del_wait_fp_up_timer();
				fp_up_fun();
				printf("finger up\n");
			}
            break;
        case SYNO_CMD_ENROLL_GETIMAGE:              // 注册用获取图像
            if (fp_report.op_result == SYNO_ACK_SUCCESS) {
				g_sys_timer_cnt=0;
				fp_op_info.msg_cache[0]=0;
				fp_op_info.rgb_cmd_cache=0;
				if(!fp_op_info.cur_enroll_cnt)fp_syno_cmd_get_id_availability(0);
				else
				{
					printf("go extract by enroll\n");
	                fp_syno_cmd_general_extract((fp_op_info.cur_enroll_cnt+1));
				}
				os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, FP_CONTINUE_SND_PKT_EVENT);
			} 
			else if(!get_setkeywarn_flag()&&(fp_op_info.query_finger_stu==FP_WAIT_ACK)){
				fp_op_info.query_finger_stu=FP_QUERY_FINGER;//采图失败继续采图定时器扫描或者中断检测否则等松开手指后再恢复采图定时器扫描和中断检测
				add_query_finger_timer();
			}
            break;
		case SYNO_CMD_READ_TEMPLETE_INDEX_TABLE:    // 读索引表
	 		if(pkg->data_size==0X2300)
	 		{
	            if (fp_report.op_result != SYNO_ACK_SUCCESS||fp_syno_search_availability_id(&(pkg->data)[1], 32, &page_id)) {
					fp_report.op_cmd 	= FP_ENROLL_MID;
					fp_report.err_code 	= PS_ADDRESS_OVER;//注册已满
					if(fp_report.op_result)printf("SYNO_CMD_READ_TEMPLETE_INDEX_TABLE ack err\n");
					if(judge_enroll_by_app())common_send_tuya_msg(fp_report.err_code,fp_report.user_id);
					else os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);//add_resend_msg_timer(&fp_report);//有漏msg的风险一旦漏了就会同时漏了打开中断导致系统死机,所以最好一直发msg,收到立马关掉
	            } 
	            else {
	                fp_op_info.enroll_id = (uint8_t)(page_id);
					fp_op_info.work_mode        = FP_WKM_ENROLL;
					printf("go extract by enroll\n");
	                fp_syno_cmd_general_extract((fp_op_info.cur_enroll_cnt+1));
					os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, FP_CONTINUE_SND_PKT_EVENT);
	            }
	 		}
            break;
        case SYNO_CMD_GEN_EXTRACT:                  // 生成特征
        	g_sys_timer_cnt=0;
            if (fp_op_info.work_mode == FP_WKM_MATCH) {
                if (fp_report.op_result == SYNO_ACK_SUCCESS) {
					fp_op_info.extract_fail_cnt=0;
					if(judge_factory_status()&&tuya_ble_band_status_get()==0)
					{
						fp_report.op_cmd  = FP_MATCH1N;
						fp_report.err_code=PS_OK;
						fp_report.user_id=FACTORY_MATCH_FP_ID;//出厂默认id0
						printf("match over go app fun:%d\n",fp_report.err_code);
						os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);
//						add_resend_msg_timer(&fp_report);//有漏msg的风险一旦漏了就会同时漏了打开中断导致系统死机,所以最好一直发msg,收到立马关掉
					}
					else
					{
						printf("go match by extract\n");
	                    fp_syno_cmd_match1n(1, 0, FP_SYNO_SUPPORT_ENROLL_NUMS);
						os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, FP_CONTINUE_SND_PKT_EVENT);
//	    				tysl_mul_fp_sched_event_post_only_evt(FP_CONTINUE_SND_PKT_EVENT);
					}
                } else {
                	fp_op_info.extract_fail_cnt++;
                	if(fp_op_info.extract_fail_cnt==FP_ENROLL_FAIL_RETRY)
                	{
						fp_opinfo_clear();
						fp_op_info.query_finger_stu=FP_QUERY_FINGER;
						fp_op_info.fp_hold_flag=0;
//						CMD_OSSemSet(&syn_cmd_ack_sem,1);
						fp_op_info.ack_cmd=SYNO_CMD_MATCH_GETIMAGE;
						log_info("extract fail 3times clear all status go recapture\n");
                	}
                	else if(fp_op_info.extract_fail_cnt>FP_ENROLL_FAIL_RETRY)
                	{
						fp_op_info.extract_fail_cnt=0;
						fp_op_info.query_finger_stu=FP_GO_SLEEP;
						set_sleep_flag_fun();
						log_info("extract fail 4times go sleep\n");
                	}
					else
					{
						printf("extract fail:%d,%d\n",pkg->data[0],fp_op_info.extract_fail_cnt);
	                    fp_report.op_cmd 	= FP_CAPTURE_AND_EXTRACT;
	    				fp_report.err_code  = pkg->data[0];
						os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);
					}
                }
            }
			else if (fp_op_info.work_mode == FP_WKM_ENROLL) {
				fp_report.err_code=pkg->data[0];
				fp_report.op_cmd = FP_ENROLL_MID;
                if (fp_report.op_result == SYNO_ACK_SUCCESS) {
					fp_op_info.extract_fail_cnt=0;
    				fp_op_info.cur_enroll_cnt += 1;
					fp_report.user_id=fp_op_info.enroll_id;
					printf("enroll cnt:%d,%d\n",fp_op_info.cur_enroll_cnt,judge_enroll_by_app());
                } else {
                	fp_op_info.extract_fail_cnt++;
                    printf("enroll extract fail:%d,%d\n",pkg->data[0],fp_op_info.extract_fail_cnt);
                }
				if(judge_enroll_by_app()){
					if(fp_report.err_code!=0&&fp_report.err_code!=PS_HANGOVER_UNREMOVE)fp_report.err_code=PS_COMMUNICATE_TIMEOUT;//这里是为了让enroll_cnt=0xff,app直接退出注册
					common_send_tuya_msg(fp_report.err_code,fp_op_info.enroll_id);
				}
				else os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);//add_resend_msg_timer(&fp_report);//有漏msg的风险一旦漏了就会同时漏了打开中断导致系统死机,所以最好一直发msg,收到立马关掉
            }
			break;
        case SYNO_CMD_SEARCH_TEMPLETE:              // 搜索指纹-匹配
            fp_report.op_cmd  = FP_MATCH1N;
			fp_report.user_id = (uint16_t)((pkg->data[1]<<8) + pkg->data[2]);
			serch_score=((uint16_t)pkg->data[3]<<8)|pkg->data[4];
			if(pkg->data_size!=0x0700||serch_score<10||fp_report.user_id>=FP_SYNO_SUPPORT_ENROLL_NUMS)fp_report.err_code=PS_NOT_SEARCHED;
			else fp_report.err_code=pkg->data[0];
			printf("match over go app fun:%d,%x,%d,%d\n",fp_report.err_code,pkg->data_size,serch_score,fp_report.user_id);
//			memcpy(&fp_report1,&fp_report,sizeof(fp_msg_t));
			os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);
//			add_resend_msg_timer(&fp_report);//有漏msg的风险一旦漏了就会同时漏了打开中断导致系统死机,所以最好一直发msg,收到立马关掉
            break;

        case SYNO_CMD_GEN_TEMPLETE:                 // 合并特征, 生成模板
            if (fp_report.op_result == SYNO_ACK_SUCCESS) {
				printf("go save templete:%d\n",fp_op_info.enroll_id);
				fp_syno_cmd_store_templete(1, fp_op_info.enroll_id);
				os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, FP_CONTINUE_SND_PKT_EVENT);
//                tysl_mul_fp_sched_event_post_only_evt(FP_CONTINUE_SND_PKT_EVENT);
            } else {
            	
				fp_report.op_cmd 	= FP_ENROLL_MID;
				fp_report.err_code 	= pkg->data[0];
				printf("SYNO_CMD_GEN_TEMPLETE fail:%x\n",fp_report.err_code);
				if(judge_enroll_by_app()){
					fp_report.err_code=PS_COMMUNICATE_TIMEOUT;//这里是为了让enroll_cnt=0xff,app直接退出注册
					common_send_tuya_msg(fp_report.err_code,fp_op_info.enroll_id);
				}
				else os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);//add_resend_msg_timer(&fp_report);//有漏msg的风险一旦漏了就会同时漏了打开中断导致系统死机,所以最好一直发msg,收到立马关掉
            }
            break;

        case SYNO_CMD_STORE_TEMPLETE:               // 存储模板
            fp_op_info.work_mode        = FP_WKM_MATCH;
			fp_report.op_cmd 	= FP_ENROLL_MID;
			fp_report.err_code 	= pkg->data[0];
			fp_report.user_id 	= fp_op_info.enroll_id;
			if(judge_enroll_by_app())common_send_tuya_msg(fp_report.err_code,fp_op_info.enroll_id);
			else os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);//add_resend_msg_timer(&fp_report);//有漏msg的风险一旦漏了就会同时漏了打开中断导致系统死机,所以最好一直发msg,收到立马关掉
			log_info("fp enroll result->[%d] [%d]\r\n", fp_report.op_result, fp_report.err_code);
            break;

    	case SYNO_CMD_DEL_TEMPLETE:                 // 删除模板
    	case SYNO_CMD_EMPTY_TEMPLETE:               // 清空指纹库
//    		if(fp_op_info.cur_stu == FP_OP_CUR_STU_WAIT_ACK){
//				fp_op_info.cur_stu =FP_OP_CUR_STU_COMMUNICATE;
//				release_del_fp_sem();//删除时应答必须清零信号量否则主任务会永远死机
//    		}
//			if(SYNO_CMD_EMPTY_TEMPLETE==fp_op_info.cmd_buf[CMD_CODE_START_POS])
			{
	            if (fp_report.op_result != SYNO_ACK_SUCCESS) {
	                log_info("[mul] fp empty templete failed->[%d]\r\n", pkg->data[0]);
	            }
				else printf("fp empty templete succ\n");
				fp_report.op_cmd = FP_ERASE_ALL;
				fp_report.err_code= pkg->data[0];
//				fp_op_info.work_mode        = FP_WKM_DEL_ALL;
				os_taskq_post_event(YZ_MAIN_TASK_NAME,1,&fp_report);//add_resend_msg_timer(&fp_report);//有漏msg的风险一旦漏了就会同时漏了打开中断导致系统死机,所以最好一直发msg,收到立马关掉
			}
            break;

 		

        case SYNO_CMD_INTO_SLEEP:
            if (fp_report.op_result != SYNO_ACK_SUCCESS) {
                log_info("[mul] fp into sleep failed->[%d]\r\n", pkg->data[0]);
				os_taskq_post_msg(YZ_SENSOR_TASK_NAME, 1, FP_CONTINUE_SND_PKT_EVENT);
//				tysl_mul_fp_sched_event_post_only_evt(FP_CONTINUE_SND_PKT_EVENT);
            } else
            {
				fp_op_info.sleep_cmd_retry=0;
                log_info("[mul] fp into sleep success\r\n");
				os_taskq_post_event(YZ_MAIN_TASK_NAME,1,FP_OP_FINISH_WITH_PWRDOWN_EVENT);
            }
            
            break;

		default:

			break;
	}
}
 
