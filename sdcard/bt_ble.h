#ifndef __BT_BLE_H__
#define __BT_BLE_H__


#include "main.h"
#include "FatFs/ff.h"

#define UPLOAD_DATA_LEN            (4096)

typedef enum{
    UPLOAD_FILE_NONE,
    UPLOAD_FILE_BEGIN,
    UPLOAD_FILE_WRITING,
    UPLOAD_FILE_RESPONSE,
    UPLOAD_FILE_FINISH,
    UPLOAD_FILE_ERR,
}Upload_state_t;


#define BUFFER0_FULL  bit(0)
#define BUFFER1_FULL  bit(1)
#define BUFFER2_FULL  bit(2)

#define USE_BUF_COUNT   3

typedef struct uplode_file_t
{
    char           filename[64];
    FIL            fd;

    uint32_t    timeout;   //下载超时计数
    uint8_t     headortail[6];//用于缓冲头帧，尾帧


    uint8_t         upload_file_flag;   //下载文件标志
    uint32_t        rec_pos;            //串口接收缓冲区的偏移
    uint32_t        advance_ok;         //提前要给的OK计数
    uint8_t         buf_is_full;        //缓冲区满
    uint8_t         cur_rec_buf;        //当前接收的缓冲区
    uint8_t         cur_write_buf;        //当前写入的缓冲区
    char            buf[USE_BUF_COUNT][UPLOAD_DATA_LEN]; //双缓冲接收文件
    uint32_t        cur_size[USE_BUF_COUNT];   //当前缓冲区大小
    uint32_t        remain_size;             //剩余未接收的缓冲区大小
    uint32_t        next_packet_size;        //下一个缓冲区的接收的数据大小
    Upload_state_t state;                    //接收文件的状态

    uint8_t            Upload_ready_next;       //数据帧准备完成

    bool cancle_flag;

}Upload_file_t;
extern Upload_file_t uploadFile_t;



status_code_t uploadfile(char *parameter);//ESP486
void upload_file_rec(__IO uint8_t data);
// void stop_uploadFile(void);
void stop_uploadFile(bool need_delete);
void upload_file_loop();
void upload_timeout_check(void);//超时检测
void upload_handle(void);//下载文件处理
void uploadFile_init(void);

void reset_upload(void);


extern char bt_ver_msg[32];
extern bool bt_get_ver_flag;
void get_bt_ver(void);
status_code_t system_excute_esp_cmd(char *line);

#endif

