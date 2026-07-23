#include "sdcard.h"

#include "bt_ble.h"
#include "grbl/report.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/stream_file.h"
#include "grbl/strutils.h"
#include "grbl/vfs.h"
#include "grbl/task.h"
#include "grbl/motion_control.h"
#include "fs_stream.h"
#include "macros.h"
#include "FatFs/ff.h"

#if FS_ENABLE & FS_SDCARD
#include "fs_fatfs.h"
#endif

// static char uploadFileHard[3][2] = {{0x0A, 0xA0}, {0x1A, 0xA1}, {0x1A, 0xA2}};
// static char uploadFileTail[2] = {0xA0, 0x0A};

Upload_file_t uploadFile_t;

#define SD_FILE_PATH        "1:"
#define CONNECT_TIMEOUT     250    //10ms *   250   2.5s没有文件传输判定为传输中断

// #define ADVANCE_OK       //提前给OK给上位机

status_code_t uploadfile(char *parameter){//ESP486
    FRESULT fr = FR_OK;
    if(get_sd_state() != SD_Idle)
    {
		return Status_SDMountError;
    }

	if (state_get() == STATE_ALARM) {
		return Status_IdleError;
	}
	if (state_get() != STATE_IDLE) {
		return Status_IdleError;
	}

	memset(uploadFile_t.filename, 0, sizeof(uploadFile_t.filename));
    // strcpy(uploadFile_t.filename, SD_FILE_PATH);

	if (parameter[0] != '/') {
    	strcat(uploadFile_t.filename,"/");
	}
    strcat(uploadFile_t.filename,parameter);
    // strcat(uploadFile_t.filename,FILE_PATH);

	// f_unlink(uploadFile_t.filename);//删除文件

    fr = f_open(&uploadFile_t.fd, uploadFile_t.filename, FA_CREATE_ALWAYS|FA_WRITE);
    if(fr == FR_OK) 
    {
        // f_write(&uploadFile_t.fd, (const uint8_t*)"\r\n", 2, NULL);
        set_sd_state(SD_BusyUploading);

        uploadFile_t.rec_pos = 0;
        memset(uploadFile_t.headortail,0,sizeof(uploadFile_t.headortail));

        uploadFile_t.cur_rec_buf = 0;
        uploadFile_t.cur_write_buf = 0;
        uploadFile_t.buf_is_full = 0;
        uploadFile_t.rec_pos = 0;
        uploadFile_t.advance_ok = 0;

        uploadFile_t.state = UPLOAD_FILE_BEGIN;
        uploadFile_t.timeout = 0;
        uploadFile_t.upload_file_flag = 1;
        hal.stream.write("start upload\n");

        
        return Status_OK;
    }
    else {
        f_close(&uploadFile_t.fd);
        return Status_FsFailedOpenDir;
    }
}
uint32_t packet_cnt = 0;
void upload_file_rec(__IO uint8_t data){

    uploadFile_t.timeout = 0;
    if(uploadFile_t.cancle_flag) return;
    if(uploadFile_t.state == UPLOAD_FILE_WRITING){
        // if(uploadFile_t.buf_is_full == 0x07){
        //     uploadFile_t.Upload_ready_next++;
        //     uploadFile_t.state = UPLOAD_FILE_ERR;
        //     uploadFile_t.cancle_flag = 1;
        //     return;
        // }
        uploadFile_t.buf[uploadFile_t.cur_rec_buf][uploadFile_t.rec_pos++] = data;
        // if((1<<uploadFile_t.cur_rec_buf) & uploadFile_t.buf_is_full){//判定是否把该缓冲区的数据写到BUFFER里了，如果该缓冲区还是满的则报错
        //     uploadFile_t.state = UPLOAD_FILE_ERR;
        //     printPgmString(CLIENT_SERIAL,"upload file write SD data timeout\n");
        //     uploadFile_t.Upload_ready_next = 1;
        // }
        if(uploadFile_t.rec_pos >= uploadFile_t.next_packet_size){
            uploadFile_t.rec_pos = 0;
            uploadFile_t.buf_is_full |= (1<<uploadFile_t.cur_rec_buf);//当前缓冲区满了
            uploadFile_t.cur_size[uploadFile_t.cur_rec_buf] = uploadFile_t.next_packet_size;//记录当前缓冲区大小
            if(++uploadFile_t.cur_rec_buf >= USE_BUF_COUNT) uploadFile_t.cur_rec_buf = 0;//切换到另一个缓冲区

            /*
            需要注意，当最后一包数据比较少，接收的时间少于上一包的数据写卡时间，
            使用同一个字节长度缓冲当前的数据长度的话，有概率会丢失最后一包数据。
            同时，如果上一包数据还在处理中，uploadFile_t.Upload_ready_next还未清零，
            直接将uploadFile_t.Upload_ready_next写1，也会造成最后一包数据丢失。
            uploadFile_t.Upload_ready_next改为采用uint8_t类型，而不是bool.
            */
           packet_cnt++;
            if(uploadFile_t.remain_size){//还有数据没有接收完
                if(uploadFile_t.remain_size >= UPLOAD_DATA_LEN){//获取下一包的数据大小
                    uploadFile_t.remain_size -= UPLOAD_DATA_LEN;
                    uploadFile_t.next_packet_size = UPLOAD_DATA_LEN;
                }
                else{
                    uploadFile_t.next_packet_size = uploadFile_t.remain_size;
                    uploadFile_t.remain_size = 0;
                }
            }
            else{
                uploadFile_t.next_packet_size = 0;//接收完了
                uploadFile_t.state = UPLOAD_FILE_RESPONSE;//接收完，应答OK
            }
            uploadFile_t.Upload_ready_next++;
        }
    }
    else if(uploadFile_t.state == UPLOAD_FILE_BEGIN){//头帧 获取文件大小
        if(uploadFile_t.rec_pos < 6){
            uploadFile_t.headortail[uploadFile_t.rec_pos++] = data;
            if(uploadFile_t.rec_pos >= 6){
                if(uploadFile_t.headortail[0] == 0x0A && uploadFile_t.headortail[1] == 0xA0){//帧头
                    uploadFile_t.remain_size = (uint32_t)uploadFile_t.headortail[2] << 24;
                    uploadFile_t.remain_size += (uint32_t)uploadFile_t.headortail[3] << 16;
                    uploadFile_t.remain_size += (uint32_t)uploadFile_t.headortail[4] << 8;
                    uploadFile_t.remain_size += (uint32_t)uploadFile_t.headortail[5];
                    // printPgmString(CLIENT_SERIAL,"Upload file start\n");

                    packet_cnt = 0;
                }
                else{
                    uploadFile_t.state = UPLOAD_FILE_ERR;
                    hal.stream.write("upload file begin error\n");
                }
                uploadFile_t.Upload_ready_next++;
            }
        }
        else{
            hal.stream.write("head rec fail\n");
        }
    }
    else if(uploadFile_t.state == UPLOAD_FILE_FINISH){//尾帧
        if(uploadFile_t.rec_pos < 6){
            uploadFile_t.headortail[uploadFile_t.rec_pos++] = data;
            if(uploadFile_t.rec_pos >= 2){
                if(uploadFile_t.headortail[0] == 0xA0 && uploadFile_t.headortail[1] == 0x0A){//帧头
                        hal.stream.write("Upload file finish\n");
                }
                else{
                    hal.stream.write("upload file tail error\n");
                    // printf("tail fail : %02x %02x \n",uploadFile_t.headortail[0],uploadFile_t.headortail[1]);
                    uploadFile_t.state = UPLOAD_FILE_ERR;
                }
                uploadFile_t.Upload_ready_next++;
            }
        }
        else{
            hal.stream.write("tail rec fail\n");
        }
    }
}
void stop_uploadFile(bool need_delete)
{
    if(uploadFile_t.upload_file_flag)
    {
        f_close(&uploadFile_t.fd);
        set_sd_state(SD_Idle);

        uploadFile_t.cur_rec_buf = 0;
        uploadFile_t.cur_write_buf = 0;
        uploadFile_t.buf_is_full = 0;
        uploadFile_t.rec_pos = 0;
        uploadFile_t.advance_ok = 0;

        uploadFile_t.upload_file_flag = 0;
        uploadFile_t.state = UPLOAD_FILE_NONE;
        uploadFile_t.Upload_ready_next = 0;
        uploadFile_t.timeout = 0;
        if(need_delete)
        {
	        f_unlink(uploadFile_t.filename);//删除文件
        }
        
        uploadFile_t.cancle_flag = false;
        
    }
}

uint32_t send_ok_cnt = 0;
uint32_t need_ok_cnt = 0;
void upload_file_loop(){
    FRESULT fr = FR_OK;
    switch(uploadFile_t.state){
        case UPLOAD_FILE_NONE:
            break;
        case UPLOAD_FILE_BEGIN://头帧
            // get_fafts_info();
            if(uploadFile_t.remain_size >= (hal_sd.sd_free_size*1024)){//内存不足
                stop_uploadFile(1);

                grbl.report.status_message(Status_UploadFileFailed);
            }
            else{
                
                uploadFile_t.advance_ok = (uint32_t)((uploadFile_t.remain_size - 1)/4096);//计算需要提前给多少个ok

                if(uploadFile_t.remain_size >= UPLOAD_DATA_LEN){//获取下一包的数据大小
                    uploadFile_t.remain_size -= UPLOAD_DATA_LEN;
                    uploadFile_t.next_packet_size = UPLOAD_DATA_LEN;
                }
                else{
                    uploadFile_t.next_packet_size = uploadFile_t.remain_size;
                    uploadFile_t.remain_size = 0;
                }
                uploadFile_t.cur_write_buf = 0;

                uploadFile_t.cur_rec_buf = 0;
                uploadFile_t.rec_pos = 0;
                uploadFile_t.buf_is_full = 0;

                memset(uploadFile_t.headortail,0,sizeof(uploadFile_t.headortail));
                uploadFile_t.timeout = 0;
                send_ok_cnt = 0;
                need_ok_cnt = uploadFile_t.advance_ok +1;

                
                uploadFile_t.state = UPLOAD_FILE_WRITING;
                grbl.report.status_message(Status_OK);
                #ifdef ADVANCE_OK
                if(uploadFile_t.advance_ok){
                    HAL_Delay(50);
                    report_status_message(SD_client, ERR_OK);
										send_ok_cnt++;
                    uploadFile_t.advance_ok--;
                    HAL_Delay(50);
                }
                #endif
                // printf("sd_client:%d,head_rec_ok\n",SD_client);
            }
            break;
        case UPLOAD_FILE_WRITING://正在传输数据
            if(uploadFile_t.advance_ok){
                grbl.report.status_message(Status_OK);
										send_ok_cnt++;
                uploadFile_t.advance_ok--;
                // printf("ok\n");
            }
            if(uploadFile_t.buf_is_full & (1<<uploadFile_t.cur_write_buf)){
                uploadFile_t.buf_is_full &= ~(1<<uploadFile_t.cur_write_buf);
                fr = f_write(&uploadFile_t.fd, (const uint8_t *)uploadFile_t.buf[uploadFile_t.cur_write_buf], uploadFile_t.cur_size[uploadFile_t.cur_write_buf], NULL);
                if(fr != FR_OK){//写入到SD卡失败
                    grbl.report.status_message(Status_UploadFileFailed);
					// printPgmString(CLIENT_SERIAL,"write 0 file error\n");
                    uploadFile_t.cancle_flag = true;
                    HAL_Delay(2000);
                    stop_uploadFile(1);
                    grbl.report.status_message(Status_UploadFileFailed);
                    return;
                }
                if(++uploadFile_t.cur_write_buf >= USE_BUF_COUNT)   uploadFile_t.cur_write_buf = 0;
            }
            else{
                grbl.report.status_message(Status_UploadFileFailed);
                // printPgmString(CLIENT_SERIAL,"upload buf overflow\n");
                uploadFile_t.cancle_flag = true;
                HAL_Delay(2000);
                stop_uploadFile(1);
                grbl.report.status_message(Status_UploadFileFailed);
                mc_reset();
            }
            //report_status_message(SD_client, ERR_OK);
            break;
        case UPLOAD_FILE_RESPONSE:
            if(uploadFile_t.buf_is_full & (1<<uploadFile_t.cur_write_buf)){
                uploadFile_t.buf_is_full &= ~(1<<uploadFile_t.cur_write_buf);
                fr = f_write(&uploadFile_t.fd, (const uint8_t *)uploadFile_t.buf[uploadFile_t.cur_write_buf], uploadFile_t.cur_size[uploadFile_t.cur_write_buf], NULL);
                if(fr != FR_OK){//写入到SD卡失败
                    grbl.report.status_message(Status_UploadFileFailed);
					// printPgmString(CLIENT_SERIAL,"write 0 file error\n");
                    uploadFile_t.cancle_flag = true;
                    HAL_Delay(2000);
                    stop_uploadFile(1);
                    grbl.report.status_message(Status_UploadFileFailed);
                    return;
                }
                if(++uploadFile_t.cur_write_buf >= USE_BUF_COUNT)   uploadFile_t.cur_write_buf = 0;
            }
            uploadFile_t.state = UPLOAD_FILE_FINISH;
            grbl.report.status_message(Status_OK);
										send_ok_cnt++;
            break;
        case UPLOAD_FILE_FINISH://接收完成尾帧
            stop_uploadFile(0);
            grbl.report.status_message(Status_OK);
            break;
        case UPLOAD_FILE_ERR:
            grbl.report.status_message(Status_UploadFileFailed);
            uploadFile_t.cancle_flag = true;
            HAL_Delay(2000);
            stop_uploadFile(1);
            grbl.report.status_message(Status_UploadFileFailed);
            mc_reset();
            break;
        default:
            break;
    }
}
void upload_timeout_check(void){
    if(uploadFile_t.upload_file_flag){
        uploadFile_t.timeout++;
        if(uploadFile_t.timeout >= CONNECT_TIMEOUT){
            uploadFile_t.timeout = 0;
            f_close(&uploadFile_t.fd);
            set_sd_state(SD_Idle);

            grbl.report.status_message(Status_UploadFileTimeout);//超时

            uploadFile_t.upload_file_flag = false;
            uploadFile_t.state = UPLOAD_FILE_NONE;
            uploadFile_t.Upload_ready_next = 0;
            uploadFile_t.timeout = 0;
            uploadFile_t.rec_pos = 0;
            
	        f_unlink(uploadFile_t.filename);//删除文件
        }
    }
}
void reset_upload(void){
    if(uploadFile_t.upload_file_flag){
        
        stop_uploadFile(0);
        grbl.report.status_message(Status_UploadFileTimeout);
    }
}
void upload_handle(void)
{
    if(uploadFile_t.Upload_ready_next)
    {
        upload_file_loop();
        uploadFile_t.Upload_ready_next--;
    }
}
void uploadFile_init(void){
    memset(&uploadFile_t,0,sizeof(Upload_file_t));
    grbl.on_user_command = system_excute_esp_cmd;
}


char str_name[32];
char set_bt_buf[50];

status_code_t system_excute_esp_cmd(char *line) {
    char str_value[10];

    char *p;
    char *q;
    char *n;

    int letter;
    int count = 10;
    
    /*----------------------------------------------------------------------
      STEP 0:删除空格，将字母换成大写
    */
    // collapseGCode(line);

    /*----------------------------------------------------------------------
      STEP 1: 查找LG0的标志
    */
    p = strstr(line, "ESP");
    if(p == NULL){
      p = strstr(line, "esp");
    }

    if(p != NULL) { p = p + 3;  }
    else { return 100; }

    memset(str_value, '\0', sizeof(str_value));

    q = &str_value[0];
    n = &str_name[0];

    /*----------------------------------------------------------------------
      STEP 2: 获取编号
    */
    while(*p != ']' && count != 0 && *p != '\0') {
      *q = *p;
      p++;
      q++;
      count--;
      if(count == 0 ) return 100; 
    }
    if(*p == ']')
    {
      p++;
      if(*p != '\0')
      {
        while (*p != '\0')
        {
          *n = *p;
          n++;
          p++;
        }
        *n = '\0';
      }
    }
    /*----------------------------------------------------------------------
      STEP 3: 获取参数
              TODO
    */
    letter = atof(str_value);

    switch(letter) {
      case 486://[esp486]
        return (uploadfile(str_name));//下载文件
        break;
      default:
        hal.stream.write("No this ESP CMD\n");
      break;
      
    }
    return 0;  
}



