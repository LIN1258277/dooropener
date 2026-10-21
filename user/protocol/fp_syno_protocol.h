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
#ifndef __FP_SYNO_PROTOCOL_H__
#define __FP_SYNO_PROTOCOL_H__
#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include "cpu.h"
/*
//1、删除未注册指纹，应答还是成功；
//2、自学习功能默认一般为打开，配置的话可以参考最后的补充说明里面，针对不同软件版本有些区别；
//3、注册和验证采集图像指令不同指令号；
//4、是的，模组上电后，中断脚不要做为中断使用

*/



#define FP_SYNO_WAIT_UP_TIME 6000

#define FP_SYNO_ENROLL_COUNTER      ( 5 )    // 指纹录入次数

#define FP_SYNO_EMPTY_ACK_TIMEOUT (2000)

#define FP_SYNO_COMMON_TIMEOUT     	(1000 ) // 通用的通信超时时间 unit: ms
#define FP_SYNO_MATCH_TIMEOUT 2300
#define FP_SYNO_DEFAULT_BAUD        ( 57600 )// 串口通信波特率
#define FP_SYNO_PWRON_WAIT_PERIOD   ( 300 )  // 模组上电并采图应答等待时长
#define FP_SYNO_SLEEP_TIMEOUT     	(400 ) // 休眠指令超时时间 unit: ms
#define FP_SYNO_CAPTURE_TIMEOUT     	(480 ) // 休眠指令超时时间 unit: ms
#define FP_SYNO_RGBCTRL_TIMEOUT     	(780 ) // 休眠指令超时时间 unit: ms




#define FP_SYNO_FIRST_HEAD			( 0xEF ) // 包头 0xEF01
#define FP_SYNO_SECOND_HEAD		    ( 0x01 ) // 包头 0xEF01

#define FP_SYNO_CMD_PKG             ( 0x01 ) // 命令包
#define FP_SYNO_DATA_PKG            ( 0x02 ) // 数据包
#define FP_SYNO_EOF_PKG             ( 0x08 ) // 结束包

#define CALC_SUM_START_POS          ( 6 )
#define CMD_CODE_START_POS          ( 9 )
#define VARIABLE_FILED_START_POS    ( 10 )


#define PS_OK                	0x00
#define PS_COMM_ERR          	0x01
#define PS_NO_FINGER         	0x02
#define PS_GET_IMG_ERR       	0x03
#define PS_FP_TOO_DRY        	0x04
#define PS_FP_TOO_WET       	0x05
#define PS_FP_DISORDER       	0x06
#define PS_LITTLE_FEATURE    	0x07
#define PS_NOT_MATCH         	0x08
#define PS_NOT_SEARCHED      	0x09
#define PS_MERGE_ERR         	0x0a
#define PS_ADDRESS_OVER      	0x0b
#define PS_READ_ERR          	0x0c
#define PS_UP_TEMP_ERR       	0x0d
#define PS_RECV_ERR          	0x0e
#define PS_UP_IMG_ERR        	0x0f
#define PS_DEL_TEMP_ERR      	0x10
#define PS_CLEAR_TEMP_ERR    	0x11
#define PS_SLEEP_ERR         	0x12
#define PS_INVALID_PASSWORD  	0x13
#define PS_RESET_ERR         	0x14
#define PS_INVALID_IMAGE     	0x15
#define PS_HANGOVER_UNREMOVE 	0X17
#define PS_FLASH_ERR         	0x18
#define PS_TRNG_ERR         	0x19
#define PS_INVALID_REG       	0x1a
#define PS_REG_SET_ERR        0x1b
#define PS_NOTEPAD_PAGE_ERR   0x1c
#define PS_ENROLL_ERR        	0x1e
#define PS_LIB_FULL_ERR      	0x1f
#define PS_DEVICE_ADDR_ERR   	0x20
#define PS_MUST_VERIFY_PWD   	0x21

#define PS_TMPL_NOT_EMPTY    	0x22//指纹模板非空
#define PS_TMPL_EMPTY    	 	  0x23//指纹模板为空
#define PS_LIB_EMPTY_ERR      0x24//指纹库为空
#define PS_TMPL_NUM_ERR		 	  0x25//录入次数设置错误
#define PS_TIME_OUT			 	    0x26//超时
#define PS_FP_DUPLICATION	 	  0x27//指纹已存在
#define PS_TMP_RELATION       0x28//指纹模板有关联
#define PS_CHECK_SENSOR_ERR   0x29//传感器初始化失败
#define PS_MOD_INF_NOT_EMPTY  0x2a//模组信息非空
#define PS_MOD_INF_EMPTY      0x2b//模组信息空
#define PS_ENROLL_CANCEL      0x2C//取消注册
#define PS_IMAGE_SMALL        0x33//图像面积太小
#define PS_IMAGE_UNAVAILABLE  0x34//图像不可用

#define PS_ENROLL_TIMES_NOT_ENOUGH 0x40 //注册次数不够
#define PS_COMMUNICATE_TIMEOUT 0X41
//#define APP_ENROLL_FLAG 0X42



typedef enum {
    // 通用类指令
 	SYNO_CMD_MATCH_GETIMAGE             = 0x01, // 验证用获取图像
    SYNO_CMD_ENROLL_GETIMAGE            = 0x29, // 注册用获取图像
	SYNO_CMD_GEN_EXTRACT 	            = 0x02, // 生成特征
	SYNO_CMD_SEARCH_TEMPLETE            = 0x04, // 搜索指纹
    SYNO_CMD_GEN_TEMPLETE               = 0x05, // 合并/生成模板
    SYNO_CMD_STORE_TEMPLETE             = 0x06, // 存储模板
	SYNO_CMD_DEL_TEMPLETE 	            = 0x0C, // 删除模板
	SYNO_CMD_EMPTY_TEMPLETE 		    = 0x0D, // 清空指纹库

    SYNO_CMD_WRITE_SYSPARA              = 0x0E, // 写系统寄存器
    SYNO_CMD_READ_SYSPARA               = 0x0F, // 读取模组基本参数
    SYNO_CMD_READ_VALID_TEMPLETE_NUMS   = 0x1D, // 读有效模板个数
    SYNO_CMD_READ_TEMPLETE_INDEX_TABLE  = 0x1F, // 读索引表

    // 模块指令集
    SYNO_CMD_AUTO_CANCEL  = 0x30,
    SYNO_CMD_AUTO_ENROLL  = 0x31,
    SYNO_CMD_AUTO_MATCH   = 0x32,
    SYNO_CMD_INTO_SLEEP   = 0x33,

    // 维护类指令
    SYNO_CMD_HANDSHAKE = 0x35,
    SYNO_CMD_RGB_CTRL = 0x3C,
    SYNO_CMD_CHECK_FINGER = 0x9d,
} FP_SYNO_CMD;

typedef enum {
	FP_SYNO_RGB_BREATH=0X01,
	FP_SYNO_RGB_FLICK,
	FP_SYNO_RGB_OPEN,
	FP_SYNO_RGB_OFF,
	FP_SYNO_RGB_GRADUADL_OPEN,
	FP_SYNO_RGB_GRADUADL_OFF,
	FP_SYNO_RGB_HORSE,
} ENUM_FP_SYNO_RGB_FUNC_CODE;//功能码
typedef enum {
	FP_SYNO_RGB_COLOR_OFF=0X00,
	FP_SYNO_RGB_COLOR_B,
	FP_SYNO_RGB_COLOR_G,
	FP_SYNO_RGB_COLOR_GB,
	FP_SYNO_RGB_COLOR_R,
	FP_SYNO_RGB_COLOR_RB,
	FP_SYNO_RGB_COLOR_RG,
	FP_SYNO_RGB_COLOR_RGB,
	FP_SYNO_RGB_COLOR_RGLOOP=0X16,
	FP_SYNO_RGB_COLOR_RBLOOP=0X15,
	FP_SYNO_RGB_COLOR_GBLOOP=0X13,
	FP_SYNO_RGB_COLOR_RGBLOOP=0X17,
	FP_SYNO_RGB_COLOR_RGALTINATE=0X26,
	FP_SYNO_RGB_COLOR_RBALTINATE=0X25,
	FP_SYNO_RGB_COLOR_GBALTINATE=0X23,
	
	
} ENUM_FP_SYNO_RGB_COLOR;//起始颜色
typedef enum {
	FP_SYNO_RGB_DUTY_RLT_SUCC=0X91,//成功提示用9:1亮灯
	FP_SYNO_RGB_DUTY_NORMAL=0X11,//正常提示用1:1亮灯
} ENUM_FP_SYNO_RGB_DUTY;//占空比
typedef enum {
	SYNO_ACK_SUCCESS 			= 0x00, // 指令执行ok
	SYNO_ACK_ERR_PKG_RCV	    = 0x01, // 数据包接收错误
	SYNO_ACK_ERR_NO_FINGER		= 0x02,	// 传感器上无指纹
} FP_SYNO_ACK;

typedef enum {
	FP_SYNO_RCV_FIRST_HEAD = 0x01,
	FP_SYNO_RCV_SECOND_HEAD,
	FP_SYNO_RCV_PKG_SIZE,
	FP_SYNO_RCV_DATAS,
	FP_SYNO_FRAME_CHECK,
} ENUM_FP_SYNO_FRAME_RCV_FSM_STATE;

#pragma pack(1)
typedef struct {
	uint16_t header;
	uint32_t dev_addr;
	uint8_t pkg_identification;
	uint16_t data_size;
	uint8_t data[];
} fp_syno_pkg_list_t;
#pragma pack()



extern void fp_syno_cmd_rgb_ctrl(ENUM_FP_SYNO_RGB_FUNC_CODE func_code,ENUM_FP_SYNO_RGB_COLOR s_color,uint8_t e_color_duty,uint8_t loop_times,uint8_t cycle);
 void tysl_pdl_fp_uart_send(uint8_t *data, uint16_t size);

//extern void fp_syno_reset_default_cmd(void);
extern void fp_syno_long_press(void);
extern void fp_syno_reset_protocol_parse(void);
extern void fp_syno_protocol_parse(uint8_t *p_data, uint16_t len);
extern void fp_syno_pkg_handle(uint8_t *data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif // __FP_SYNO_PROTOCOL_H__

/* [] END OF FILE */
