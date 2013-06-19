/*
     APP_808 .h
*/
#ifndef  _APP_808
#define   _APP_808

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include <App_moduleConfig.h>
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>


// 1  : connect  not find      0:  not  find      2: connect find  
#define  USB_NOTCONNECT                        0
#define  USB_CONNECT_NOTFIND              1
#define  USB_FIND                                     2 



typedef  struct  _APP_QUE
{
   u32  write_num;
   u32  read_num;
   u8    sd_enable_flag;   // 发送后 1  接收到应答后才置位为 0 
   u8    re_send_flag;  //   重新发送标志位
   u8    abnormal_counter;// 发送后不相等，且收不到应答


}APP_QUE;

extern APP_QUE  AppQue;
extern u8  Udisk_Test_workState;  //  Udisk 工作状态 
extern u32  sec_num;


extern   rt_device_t   Udisk_dev;
extern   u8  Udisk_filename[30];  
extern   int  udisk_fd;   
extern   u16   AD_Volte;
extern   u16   AD_2through[2]; //  另外2 路AD 的数值 





extern u8      Udisk_Find(void);
extern void    Protocol_app_init(void);
extern void  SensorPlus_caculateSpeed (void);
extern void  App_rxGsmData_SemRelease(u8* instr, u16 inlen,u8 link_num);

//          System  reset  related  
extern void  System_Reset(void);
extern  void  reset(void);  

extern void Udisk_write_buffer(u8 *Inbuf,u16 inlen);
extern void UDisk_Write_Test(void);

extern  void  MainPower_Recover_process(void);
extern  void  MainPower_cut_process(void); 

#endif
