/*
     App_808.C
*/

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>
//#include "usbh_usr.h"


/* 定时器的控制块 */
 static rt_timer_t timer_app; 




//----- app_thread   rx     gsm_thread  data  related ----- 	
ALIGN(RT_ALIGN_SIZE)
static MSG_Q_TYPE  app_rx_gsm_infoStruct;  // app   接收从gsm  来的数据结构
static  struct rt_semaphore app_rx_gsmdata_sem;  //  app 提供数据 给gsm发送信号量

//----- app_thread   rx     gps_thread  data  related ----- 	
//ALIGN(RT_ALIGN_SIZE)
//static  MSG_Q_TYPE  app_rx_gps_infoStruct;  // app   接收从gsm  来的数据结构
//static  struct rt_semaphore app_rx_gps_sem;  //  app 提供数据 给gps发送信号量

//----- app_thread   rx    485 _thread  data  related ----- 	
//ALIGN(RT_ALIGN_SIZE)
//static  MSG_Q_TYPE  app_rx_485_infoStruct;  // app   接收从gsm  来的数据结构
//static  struct rt_semaphore app_rx_485_sem;  //  app 提供数据 给gps发送信号量 

static  struct rt_semaphore SysRst_sem;  // system reset


static u8  OneSec_cout=0;

u8  Udisk_Test_workState=0;  //  Udisk 工作状态
u8  TF_test_workState=0;

rt_device_t   Udisk_dev= RT_NULL;
u8 Udisk_filename[30]; 
int  udisk_fd=0;

	
static  rt_device_t   TF_dev=RT_NULL;

static u8  TF_filename[30];

static  int  TF_fd=0;
u32   sec_num=2;
u32       WarnTimer=0; 
u16   ADC_ConvertedValue=0; //电池电压AD数值  
u16   AD_Volte=0;
u16   AD_2through[2]=0; //  另外2 路AD 的数值

u8   OneSec_CounterApp=0;
APP_QUE  AppQue; 



 //  1. MsgQueue Rx


void App_rxGsmData_SemRelease(u8* instr, u16 inlen,u8 link_num)
{
	/* release semaphore to let finsh thread rx data */
	//rt_sem_release(&app_rx_gsmdata_sem);
	Receive_DataFlag=1; 
	app_rx_gsm_infoStruct.info=instr;
	app_rx_gsm_infoStruct.len=inlen;  
	app_rx_gsm_infoStruct.link_num=link_num;  
}


void Device_RegisterTimer(void)
{
      if(0==JT808Conf_struct.Regsiter_Status)    //注册   
          {
             DEV_regist.Sd_counter++;
			 if(DEV_regist.Sd_counter>6)
			 	{
                   DEV_regist.Sd_counter=0;
				   DEV_regist.Enable_sd=1;  
			 	}
          }
}


void Device_LoginTimer(void)
{  
  if((1==DEV_Login.Operate_enable)&&(DEV_Login.Max_sd_num<2))
  {
     DEV_Login.Sd_counter++;
	if(DEV_Login.Max_sd_num==0)
	{
		 if(DEV_Login.Sd_counter>=20)   
		 {
	          DEV_Login.Sd_counter=0;
		   DEV_Login.Enable_sd=1;
		   DEV_Login.Max_sd_num++; 
		 }
	}
	else
	if(DEV_Login.Sd_counter>=44)      
	{
	          DEV_Login.Sd_counter=0;
		   DEV_Login.Enable_sd=1;
		   DEV_Login.Max_sd_num++; 
	}
  }
}

//          System  reset  related  
void  System_Reset(void)
{
      rt_sem_release(&SysRst_sem);
}

void   Reset_Saveconfig(void)
{
     #if  0
               //---------  重启前存储单位小时 每分钟 平均度  --------
            
	        DF_WriteFlashSector(DF_FlowNum_Page,0,(u8*)&Flowing_ID,2);
	        delay_ms(2);
			Save_AvrgSpdPerMin(AvrgSpdPerMin_write);
			AvrgSpdPerMin_write++;
			if(AvrgSpdPerMin_write>=Max_SPDSperMin)
			   AvrgSpdPerMin_write=0;					
			DF_Write_RecordAdd(AvrgSpdPerMin_write,AvrgSpdPerMin_Read,TYPE_AvrgSpdAdd); 
	  //-------存储单位分钟 每秒钟平均速度-------			
			Save_SpdPerSecond(AvrgSpdPerSec_write);
			AvrgSpdPerSec_write++;
			if(AvrgSpdPerSec_write>=Max_SPDerSec)
			   AvrgSpdPerSec_write=0;					
			DF_Write_RecordAdd(AvrgSpdPerSec_write,AvrgSpdPerSec_Read,TYPE_AvrgSpdSecAdd);
	   //   里程
	   DF_WriteFlashSector(DF_Distance_Page,0,(u8*)&Distance_m_u32,4); 	   
	   DF_Write_RecordAdd(Distance_m_u32,DayStartDistance_32,TYPE_DayDistancAdd); 
	   //------------------------------------------------------	
	   if(ISP_resetFlag==1)
		  	ISP_resetFlag=2;
   #endif

}
	
void  App808_tick_counter(void) 
{
    Systerm_Reset_counter++;
    if(Systerm_Reset_counter>Max_SystemCounter)
     {   	Systerm_Reset_counter=0;	
	        rt_kprintf("\r\n Sysem  Control   Reset \r\n"); 
               reset(); 
     }  

}



void Udisk_write_buffer(u8 *Inbuf,u16 inlen)
{
    int  fd_res=0;
   if((sec_num>0)&&(sec_num<350)) 
   {   //--- udisk
      if((Udisk_Test_workState)&&(udisk_fd))
         { 
                fd_res=write(udisk_fd,Inbuf,inlen);
	        rt_kprintf("\r\n  wr :%s  resualt=%d\r\n",Udisk_filename,fd_res);		
      	  }
   #if 1	  
      // ----sd	  
      if((TF_test_workState)&&(TF_fd))
        {
             fd_res=write(TF_fd,Inbuf,inlen);
              rt_kprintf("\r\n  TFwr :%s  resualt=%d\r\n",TF_filename,fd_res);	
      	}		 
   #endif	  
   }	  
}

void UDisk_Write_Test(void)
{ 
       rt_err_t res;
	 
      sec_num++; 
      if(sec_num==400)
	  	sec_num=0;  
	if(sec_num==0)
	{   
	      rt_kprintf("\r\n  Sec_num=0\r\n");  
	      //------ U disk
              Udisk_dev=rt_device_find("udisk");
	        if (Udisk_dev != RT_NULL)	    
		{       rt_kprintf("\r\n  Udiskopen");
                       
			  res=rt_device_open(Udisk_dev, RT_DEVICE_OFLAG_RDWR); 	
			 if(res==RT_EOK)
			 {
			    memset(Udisk_filename,0,sizeof(Udisk_filename));
			    RTC_TimeShow();	
			    sprintf((char*)Udisk_filename,"/udisk/ud%d%d%d.txt", time_now.hour,time_now.min, time_now.sec);  	
			    rt_kprintf("\r\n   UdiskFilename:%s\r\n",Udisk_filename);
			    udisk_fd=open((char*)Udisk_filename, O_RDWR|O_CREAT, 0); 
			     Udisk_Test_workState=1;   	 
			 }		 
		}  
	       else
		 { 
                      Udisk_Test_workState=0;
	        }
		  
             //----  TF---
          #if  1   
		/*  if(dfs_mount("spi_sd","/sd","elm",0,0))
		  	{
		  	     rt_kprintf("\r\n   TF errror\r\n");
		  	      return ;
		  	}
		  	*/
		  TF_dev=rt_device_find("spi_sd");
	        if (TF_dev != RT_NULL)	     
		{
                       rt_kprintf("\r\n  TFopen\r\n");  
			   res=rt_device_open(TF_dev, RT_DEVICE_OFLAG_RDWR); 
			  if(res==RT_EOK)
			 {
			   memset(TF_filename,0,sizeof(TF_filename));
			    RTC_TimeShow();	
			   sprintf((char*)TF_filename,"/sd/tf%d%d%d.txt",time_now.hour,time_now.min, time_now.sec); 
			    rt_kprintf("\r\n   TF_Filename:%s\r\n",TF_filename);       
			  TF_fd=open((char*)TF_filename, O_RDWR|O_CREAT, 0); 
			  TF_test_workState=1; 
			  }
		} 
		else
		  { 
		          rt_kprintf("\r\n  TF not find");   
                      TF_test_workState=0;
	        }
			
	  #endif	
	}


	if(sec_num==390)  
	{
	       //----  udisk
	      if((Udisk_Test_workState)&&(udisk_fd)) 
             {
                  close(udisk_fd);
		    rt_kprintf("\r\n   Udisk Filename:%s    close\r\n",Udisk_filename); 
			
	       }
	#if  1	  
              //  ----  tf 
            if((TF_test_workState)&&(TF_fd))
		{
		    close(TF_fd); 
		    rt_kprintf("\r\n  TF_ Filename:%s    close\r\n",TF_filename); 	   
            	}
	#endif		
	}
}

void  SensorPlus_caculateSpeed (void)
{
   u16 plus_reg=0;
     
         plus_reg=TIM2->CNT;
	  TIM_SetCounter(TIM2, 0);		
	  Delta_1s_Plus=plus_reg;
	  
#if 1	 
		total_plus+=plus_reg; 
	  
			  if(JT808Conf_struct.Speed_GetType)  // 通过速度传感器 获取速度
			  { 
					   //  K_AdjustUseGPS(sp,sp_DISP);	//	调整K值  
						GPS_speed=Speed_cacu;	   
			  }  
		   else
			  {
					if(Jia_GPS_spdFlag==0)
					  {
						  if(UDP_dataPacket_flag==0x03)
							   Speed_gps=0;
					  } 		  
					   GPS_speed=Speed_gps; 
			  }  
		
   

	 if(DispContent==4)  //  disp  显示   
	 { 
	//   if(DF_K_adjustState)
	 //	rt_kprintf("\r\n    自动校准完成!");
	  // else
	 	//rt_kprintf("\r\n    尚未自动校准校准!");
	 
	rt_kprintf("\r\n Jia=%d,GPS=%d km/h , Sensor=%d km/h Actual=%d km/h J1 zhenGps=%d km/h	Plus=%d\r\n",Jia_GPS_spdFlag,Speed_gps/10,Speed_cacu/10,GPS_speed/10,Speed_gps_jiashi/10,Delta_1s_Plus/10);   
	 } 
#endif


}

void  Emergence_Warn_Process(void)
{
  //----------- 紧急报警下拍照相关的处理  ------------------
  if(WARN_StatusGet()) 
  {
     //  拍照过程中和传输过程中不予处理                        
	 if((CameraState.status==other)&&(Photo_sdState.photo_sending==0)&&(0==MultiTake.Taking)&&(0==MultiTake.Transfering))
	  {
		  WarnTimer++;
		  if(WarnTimer>=4)
			  {
			      WarnTimer=0;    	
				       //----------  多路摄像头拍照 -------------
					 // MultiTake_Start();
					 // Start_Camera(1);  //先拍一路满足演示   
			  }   
	  }   
     //-------------------------------------------------------
     
	 fTimer3s_warncount++;
	 if(fTimer3s_warncount>=4)			 
	{
		 // fTimer3s_warncount=0;
		 if ( ( warn_flag == 0 ) && ( f_Exigent_warning == 0 )&&(fTimer3s_warncount==4) )
		 {
				 warn_flag = 1; 					// 报警标志位
				 Send_warn_times = 0;		  //  发送次数 

			  if((Key_MaskWord[3]&0x01)==0x00)
			  	{
			       if((Warn_MaskWord[3]&0x01)==0x01)
			       	{
	                      rt_kprintf( "紧急报警 触发不上报，屏蔽字使能  !");		

			       	}
				   	else
				   	{
						 rt_kprintf( "紧急报警 ");				 
						 StatusReg_WARN_Enable(); // 修改报警状态位
						 PositionSD_Enable();  
						 Current_UDP_sd=1;  
						 warn_msg_sd(); // 短息报警
				   	 }
			     }
			     else
			     	{
			     	      if((Warn_MaskWord[3]&0x01)==0x01)
					       	{
			                      rt_kprintf( "\r\n紧急报警 触发上报，屏蔽字使能 ,关键字使能就上报! ");		

					       	}
						   else
							   rt_kprintf( "\r\n紧急报警 触发上报，关键字使能就上报! ");	   
						   	    
						   	{
								 StatusReg_WARN_Enable(); // 修改报警状态位
								 PositionSD_Enable();  
								 Current_UDP_sd=1; 
								 warn_msg_sd();// 短息报警
						   	 }


			     	}
		 }
	}

	 //------------------------------------------------- 
  } 
   else
  { 	   
	   WarnTimer=0;
	   fTimer3s_warncount=0;  
	   //------------ 检查是否需要报警拍照 ------------
	 /*  if(CameraState.status==enable)
	   {		   
		 if((CameraState.camera_running==0)||(0==Photoing_statement))  // 在不传输报警图片的情况下执行
		 {
			CameraState.status=disable;
		 }
	   }
	   else*/
	   if(CameraState.camera_running==0)
			CameraState.status=other;
  }   


}



static void timeout_app(void *  parameter)
{     //  100ms  =Dur 
    u8  SensorFlag=0,i=0;

     //--------  CAN ID -------------------------
   //  if(CAN_trans.can1_trans_dur>0) 
    // {
        //    CAN_trans.canid_ID_enableGet=1;	 
     //} 	

	 
	 //============  状态检测 =======================
	 ACC_status_Check(); 
     //------------------------------------------	
      OneSec_CounterApp++;
     if(OneSec_CounterApp>=10)	 
	{
           /*   if(CAN_trans.can1_trans_dur>0) 
              {
                 if((OneSec_CounterApp%5)==0)  
                       CAN_trans.can1_enable_get=1;   
              } 
*/
             OneSec_CounterApp=0; 
	     //---------------------------------- 	
         	 	       // 行车记录仪数据发送
	     Recode_Obj.timer++;     
		  if((Recode_Obj.CountStep==2)&&(Recode_Obj.timer>=3))  
		   	{    
		   	       Recode_Obj.CountStep=1; 
				   Recode_Obj.SD_Data_Flag=1;
		           Recode_Obj.timer=0;
				   rt_kprintf( "\r\n 记录仪定时发送"); 
		    }  
	         //--------------------------------------
           
		 
	        if(DataLink_Status())
		    {
		          Device_RegisterTimer();
		          Device_LoginTimer();	      // 鉴权只发送一次
			   SendMode_ConterProcess(); 	  
		    }	 
		     SensorPlus_caculateSpeed();  	     
		     IO_statusCheck(); 	 
		     Emergence_Warn_Process();   
		     Meida_Trans_Exception();	 
		     Sensor_Keep_Process_Clear();	 
		      local_time();	 
	            SMS_Send_timer();		
		     App_mq_NoAck_counter(); 		
             gnss_timer();  

			      
			   
		            //车辆信号线状态指示  //刹车线//左转//右转//远光//近光灯//雾灯//车门//null
		     
			SensorFlag=0x80;
			for(i=1;i<8;i++)  
			{
				  if(Vehicle_sensor&SensorFlag)   
					    XinhaoStatus[i+10]=0x31;
				   else
					     XinhaoStatus[i+10]=0x30; 
				     SensorFlag=SensorFlag>>1;   
			} 
		       if(DispContent==3)
			       rt_kprintf("\r\n %s   \r\n",XinhaoStatus);     

            //  system timer
            App808_tick_counter(); 

                  //---------------------------------- 
                  if( ReadCycle_status==RdCycle_SdOver)
		   {	   
		        ReadCycle_timer++;	   
			 if(ReadCycle_timer>5)  //5s No resulat
			 {
			      ReadCycle_timer=0; 
		             Api_cycle_Update();	   
			 }		
                  }	   

	        //====疲劳驾驶===========================
		if(Car_Status[3]&0x01)	 // 疲劳驾驶是基于ACC开的情况下进行的 
		       Tired_Check();     

		 //  --- 超速报警  -----------
		  SpeedWarnJudge();	// 速度报警判断    
		  //-------------------------------------------------------		

   }	
         //   Media 
           if(OneSec_CounterApp>>1)   //  除以2 为1	   
	           Media_Timer_Service();        
	   //----------------------------------	
	   Exact_Test_time(); 

}

u8    Udisk_Find(void)
{
       rt_err_t res;

	if(  USB_Disk_RunStatus()==USB_CONNECT_NOTFIND)
	{
              Udisk_dev=rt_device_find("udisk");
	        if (Udisk_dev != RT_NULL)	    
		{     
		      rt_kprintf("\r\n  Udiskopen");                       
		      res=rt_device_open(Udisk_dev, RT_DEVICE_OFLAG_RDWR); 	
		      if(res==RT_EOK)
			 {
                      
	                   Udisk_Test_workState=1;
			     USB_DeviceFind();	
				rt_kprintf("\r\n Udisk Find ok\r\n");  	 
			      return       USB_FIND;		   
			 }	
			  return false;
		}  
	       else
		 { 
                      Udisk_Test_workState=0;
			 return  false;		  
	        }
	}
	else
	if( USB_Disk_RunStatus()==USB_FIND)	
		return  USB_FIND;
	else
		Udisk_Test_workState=0;
	return false; 
}

 void   MainPower_cut_process(void)
 {
	Powercut_Status=0x02;
	//----------断电了  ------------
	//---------------------------------------------------
	//        D4  D3
	//        1   0  主电源掉电 
	StatusReg_POWER_CUT(); 
	//----------------------------------------------------
	Power_485CH1_OFF;  // 第一路485的电			 关电工作
	//-----------------------------------------------------			    
	//------- ACC 不休眠 ----------
	Vehicle_RunStatus|=0x01;
	//   ACC ON		 打火 
	StatusReg_ACC_ON();  // ACC  状态寄存器   
	Sleep_Mode_ConfigExit(); // 休眠相关
	//-------  不欠压--------------
	Warn_Status[3]&=~0x80; //取消欠压报警  
	SleepCounter=0; 

 }


 void  MainPower_Recover_process(void)
 { 
	//----------------------------------------------
	StatusReg_POWER_NORMAL();
	//----------------------------------------------
	Power_485CH1_ON;  // 第一路485的电 		  开电工作
}

void  MainPower_Status_Check(void)
{
    



}

 ALIGN(RT_ALIGN_SIZE)
char app808_thread_stack[4096];      
struct rt_thread app808_thread;

static void App808_thread_entry(void* parameter) 
{
    MSG_Q_TYPE  AppMsgQ_struct;  
    MSG_Q_TYPE  MsgQ_gsmCome;	 	
    u8   Dialstr[40];  	
//    u32  a=0;	

     //  finsh_init(&shell->parser);
	  rt_kprintf("\r\n ---> app808 thread start !\r\n");  
  
	  TIM2_Configuration();	   
       //  step 1:  Init Dataflash
         DF_init();
         ProductAttribute_init();  
       //  step 2:   process config data   
         SysConfiguration();    // system config   	    
       //  step 3:    usb host init	   	    	//  step  4:   TF card Init    
       //  	 spi_sd_init();	    
          usbh_init();    
	   APP_IOpinInit();
          Init_ADC(); 
		  Recorder_init();
       //  	 tf_open();      // open device  
	// pos=dfs_mount("spi_sd","/sd","elm",0,0);	
       //   if(pos)
	//  	rt_kprintf("\r\n TF_result=%d\r\n",pos); 
	//rt_kprintf("\r\n tf ok\r\n");  

	   
        /* watch dog init */ 
	WatchDogInit();                        
       GNSS_RawDataTrans_Init(0);
	CAN_struct_init();     
	SMS_send_init(); 
	RAW_TransInit(); 

    AppQue.write_num=0;
	AppQue.read_num=0;
	AppQue.sd_enable_flag=0;  
	AppQue.abnormal_counter=0;
	AppQue.re_send_flag=0; 
	
	while (1)
	{
  
               //  system reset  
         /*       if(rt_sem_take(&SysRst_sem,2)==RT_EOK)
              {
                       Close_DataLink();  
			  //reset  	  	  
              }
              */   
		//--------------------------------------------------	
       #if 0
               if (rt_sem_take(&app_rx_gsmdata_sem, 2) == RT_EOK) 
               {
                       memcpy( UDP_HEX_Rx,app_rx_gsm_infoStruct.info,app_rx_gsm_infoStruct.len);
			  UDP_hexRx_len=app_rx_gsm_infoStruct.len;	 
			  if(app_rx_gsm_infoStruct.link_num)
			  	   rt_kprintf("\r\n Linik 2 info \r\n");   
                       TCP_RX_Process(app_rx_gsm_infoStruct.link_num);      
                }	
	#else
                if(Receive_DataFlag==1)
		   {
                        memcpy( UDP_HEX_Rx,app_rx_gsm_infoStruct.info,app_rx_gsm_infoStruct.len);
			  UDP_hexRx_len=app_rx_gsm_infoStruct.len;	 
			  if(app_rx_gsm_infoStruct.link_num)
			  	   rt_kprintf("\r\n Linik 2 info \r\n");    
                       TCP_RX_Process(app_rx_gsm_infoStruct.link_num);        
			  Receive_DataFlag=0;	 	   
                }	
             
	#endif 


			     
                //    ISP  service  
		  Api_CHK_ReadCycle_status();//   循环存储状态检测		
		  //-------- 808   Send data   		
             	  if(DataLink_Status()&&(CallState==CallState_Idle)&&(VocREC.Sate!=VOICEREC_DataRXing))   
		   {   
		     
		
		        //------- send
		           if(App_mq_Resend()==false)
                                  App_mq_Read_Process(); 

		             Do_SendGPSReport_GPRS();   
		   } 
                else
		   if(CallState==CallState_rdytoDialLis)
			  {
				  CallState=CallState_Dialing;
				  memset(Dialstr,0,sizeof(Dialstr));
				  memcpy(Dialstr,"ATD",3);
				  memcpy(Dialstr+3,JT808Conf_struct.LISTEN_Num,strlen((const char*)JT808Conf_struct.LISTEN_Num));
				  memcpy(Dialstr+3+strlen((const char*)JT808Conf_struct.LISTEN_Num),";\r\n",3);	
				  rt_kprintf((char*)Dialstr);
				  rt_hw_gsm_output((const char *)Dialstr); 	 
		  
		  }   
               //-----------------  顺序存储 GPS  -------------------		
               if( JT808Conf_struct.Close_CommunicateFlag==1)
               {
                           Save_GPS();    
                }
           #if 0        
		if(GPS_getfirst)	 //------必须搜索到经纬度
		{
			   // if(Current_SD_Duration>10)// 间隔大于10s 存储顺序上报， 小于10 不存储上报 
			    {                                                 //  拍照中暂不操作flash
					   Save_GPS();       
			    }
		} 
	   #endif	 
	     //---------------------------------------------------------------------------------------------	  
                rt_thread_delay(22);	  
	}
}



/* init app808  */
void Protocol_app_init(void)
{
        rt_err_t result;


        rt_sem_init(&SysRst_sem,"SysRst",0,0);  
        rt_sem_init(&app_rx_gsmdata_sem, "appRxSem", 0, 0);   		
       //---------  timer_app ----------
	         // 5.1. create  timer     100ms=Dur
	      timer_app=rt_timer_create("tim_app",timeout_app,RT_NULL,10,RT_TIMER_FLAG_PERIODIC); 
	        //  5.2. start timer
	      if(timer_app!=RT_NULL)
	           rt_timer_start(timer_app);      


       //------------------------------------------------------
	result=rt_thread_init(&app808_thread, 
		"app808", 
		App808_thread_entry, RT_NULL,
		&app808_thread_stack[0], sizeof(app808_thread_stack),  
		Prio_App808, 10); 

    if (result == RT_EOK)
    {
           rt_thread_startup(&app808_thread); 
   	    rt_kprintf("\r\n app808  thread initial sucess!\r\n");    // nathan add
    	}
    else
	    rt_kprintf("\r\napp808  thread initial fail!\r\n");    // nathan add	   
}



