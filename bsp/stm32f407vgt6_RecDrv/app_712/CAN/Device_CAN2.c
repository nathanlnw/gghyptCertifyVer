/*
       Device  CAN    2  
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
#include "Device_CAN.h"
#include <finsh.h>

u8   U3_Rx[100];
u8   U3_content[100];
u16   U3_content_len=0; 
u8   U3_flag=0;
u16   U3_rxCounter=0; 



#ifdef RT_USING_DEVICE 
 struct rt_device  Device_CAN2;  
#endif

//     小板通信波特率  调整至   115200bps

/*
        1.CAN 相关

         1    1       1       2    content                     ( IN  content   参考JT/T 808  北斗表12  Page 16)
        7E  33   Type    ID      n             7E

        Type:  01  <->  波特率设置   
                                    内容 2 bytes   HL    单位 KB <=> ID    (没有ID  ，直接是波特率内容)
                  02  <->  JT/T 808 北斗 ID   +  内容 

        2.  透传
        7E  34  01    content   7E
*/

u16  Protocol_808_Decode_Good(u8 *Instr ,u8* Outstr,u16  in_len);

void U3_RxProcess(void)  
{
   u8  iRX;
   uint16_t i,len;
   uint8_t fcs;
   uint8_t buf[32]; 
	 
     if(U3_content[1]==0x33)  // CAN 2
     	{
             if(U3_content[2]==0x03)  // CAN 数据接收
             	{  
             	     //-------------------------------------------------
             	     //7e 33 03 08 00 00 00 00 00 20 00 04 00 08 30 31 32 33 34 35 36 07 00 7e
             	     memcpy(&RxMessageData,U3_content+3,20);   
					 
             	     
                    //DataTrans.Data_Tx[DataTrans.Tx_Wr++]=(BD_EXT.CAN_2_ID>>24); //  BIT 7 can1  bit6 --1  CAN2 扩展 
		      //DataTrans.Data_Tx[DataTrans.Tx_Wr++]=BD_EXT.CAN_2_ID>>16;
		      //DataTrans.Data_Tx[DataTrans.Tx_Wr++]=BD_EXT.CAN_2_ID>>8;
		      //DataTrans.Data_Tx[DataTrans.Tx_Wr++]=BD_EXT.CAN_2_ID;     

	  //  for(iRX=0;iRX<RxMessageData.DLC;iRX++)
	      for(iRX=0;iRX<8;iRX++)
	     {		
	        DataTrans.Data_Tx[DataTrans.Tx_Wr++]=RxMessageData.Data[iRX]; 
	     }

               DataTrans.Data_TxLen=DataTrans.Tx_Wr;   
		 DataTrans.TYPE=0x01; //  类型	
			//--------------------------------------------------	
	    //if(DispContent!=1)		
		{
		  rt_kprintf("\r\n BDEXT_ID=%08X\r\n",BD_EXT.CAN_2_ID);	   
                rt_kprintf("\r\n CAN2 RxMessage=%2X,%2X,%2X,%2X,%2X,%2X,%2X,%2X",RxMessageData.Data[0],RxMessageData.Data[1],RxMessageData.Data[2],RxMessageData.Data[3],RxMessageData.Data[4],RxMessageData.Data[5],RxMessageData.Data[6],RxMessageData.Data[7]);
	    	}
             	}

     	}
	else if ( U3_content[1] == 0x35 ) /*连接到扩展板的Usart3同IC读卡器连接，9600bps*/
	{
	
		/*'5',1,<转义后的ic卡数据,以0x7e封包>*/
		len=Protocol_808_Decode_Good( U3_content+3, U3_content,U3_content_len-4); 
		rt_kprintf("\r\n******************\r\n");
		   for(i=0;i<len;i++) rt_kprintf("%02x ",U3_content[i]); 
		rt_kprintf("\r\n******************");

		if(U3_content[0]!=0x7e) return;
		if(U3_content[len-1]!=0x7e) return; 
		/*计算累加和*/
		fcs=0;
		//for(i=4;i<len-1;i++)  fcs+=U3_content[i]; 
		//if(fcs!=U3_content[1])
		//{
			//rt_kprintf("\r\n%s(line:%d)>",__func__,__LINE__);
		//}
		if(U3_content[6]!=0x0B) return; // IC 卡类型检查，不是则返回
		
		switch(U3_content[7])					
		  {	 
		      case         0x40:  // 首次插卡的信息
							{    // 40H   
							      switch(U3_content[8])
							      	{
					                           case 0x00:
											  rt_kprintf("\r\n IC 卡读卡成功\r\n");
											  IC_MOD.IC_Status=1;
											  JT808Conf_struct.Driver_Info.BD_IC_rd_res=0x00;
											  time_now=Get_RTC(); 
											  Time2BCD(JT808Conf_struct.Driver_Info.BD_IC_inoutTime);  
											//  SD_ACKflag.f_DriverInfoSD_0702H=1;
											  break;
								     case 0x01:
											  rt_kprintf("\r\n IC 卡未插入\r\n");
											   JT808Conf_struct.Driver_Info.BD_IC_rd_res=0x01;
											   IC_MOD.IC_Status=0;
											   time_now=Get_RTC(); 
											  Time2BCD(JT808Conf_struct.Driver_Info.BD_IC_inoutTime);  
                                                                                //  send back
											  buf[0]=0x03;
											  DeviceData_Encode_Send(0x0B,0x40,buf,1);
											  break;			  
					                          case 0x02:
											  rt_kprintf("\r\n IC 卡读取失败\r\n");
											   //  send back
											  buf[0]=0x03;
											  DeviceData_Encode_Send(0x0B,0x40,buf,1);
											  break;
								     case 0x03:
											  rt_kprintf("\r\n 非从业资格证卡\r\n");
											   //  send back
											  buf[0]=0x03;
											  DeviceData_Encode_Send(0x0B,0x40,buf,1);
											  break;		  
								     case 0x04:
  											  rt_kprintf("\r\n IC 卡被锁定\r\n"); 
  											  //  send back
											  buf[0]=0x03; 
											  DeviceData_Encode_Send(0x0B,0x40,buf,1); 
											  break;

							      	}
								  
					                     if(DataLink_Status()&&(TCP2_Connect))
					                     {  //  Online      Trans  64 Data  to  Centre  , wait for 1 or 25 byte result, 
					                     	    //  send resualt to  IC module , get  41H  info (Driver info),  then send 41H to  Centre
					                                if(U3_content[8]==0x00) 	/*读卡成功 返回应答,网络应答35s，会重发*/
								          {             
										 memset(IC_MOD.IC_Tx40H,0,sizeof(IC_MOD.IC_Tx40H));   
					                                    memcpy(IC_MOD.IC_Tx40H,U3_content+9,64);    //获取64个字节的卡片信息
					                                    IC_MOD.Trans_0900Flag=1; // 发送透传 
					                                    rt_kprintf("\r\n IC get 64 Bytes\r\n");  
					                                    return ;
					                               }					  
															  	
					                     }
								else		
									{     //Off line 
						                            buf[0]=0x7e;
										buf[1]='5';
										buf[2]=1;
										buf[3]=0x7d;
										buf[4]=0x2;  /*标识，转义*/										
										buf[5]=0x4c; /*校验*/
										buf[6]=0x0;
										buf[7]=0x0;
										buf[8]=0x0;
										buf[9]=0x0b;
										buf[10]=0x40;
										buf[11]=0x01;/*应答:  终端不在线*/
										buf[12]=0x7d;
										buf[13]=0x02;
										buf[14]=0x7e;
						                              //  Trans  Back
						                            rt_device_write(&Device_CAN2,0,( const char*)buf,15);  
										return;	 		  
									}
							}

           case    0x41: 	//  IC 内驾驶员的信息	
					    {
							 rt_kprintf("\r\n IC  驾驶员信息读取\r\n"); 
					                   if(U3_content[8]==0x00)   // 获取驾驶身份信息内容
					                   	{
					                   	      rt_kprintf("\r\n 读取到驾驶员身份信息\r\n");  
					                   	      memcpy(IC_MOD.IC_TX41H,U3_content+9,len-9); 
                                                              IC_MOD.IC_TX41H_len=len-9;
								      SD_ACKflag.f_DriverInfoSD_0702H=1; // 使能上报
					                      }
								 //  send back
								 DeviceData_Encode_Send(0x0B,0x41,NULL,0);	   
					                   return ;
					             	}  
	   case    0x42:     // 卡片拔出通知
                                       {
						        time_now=Get_RTC(); 
							 Time2BCD(JT808Conf_struct.Driver_Info.BD_IC_inoutTime);  		   	    
                                                  IC_MOD.IC_Status=0;
						       if(DataLink_Status())						  
                                                     SD_ACKflag.f_DriverInfoSD_0702H=1; // 使能上报
                                                  // send back
                                                  DeviceData_Encode_Send(0x0B,0x42,NULL,0);	 
                                                  return ;
	   	                          }				
	                           
	    }
         
	}


}

u16  Protocol_808_Decode_Good(u8 *Instr ,u8* Outstr,u16  in_len)  // 解析指定buffer :  UDP_HEX_Rx  
{
	//-----------------------------------
	  u16 i=0, decode_len=0;

    // 1.  clear  write_counter
	  decode_len=0;//clear DecodeLen

	// 2   decode process   
	for(i=0;i<in_len;i++)
	 {
		if((Instr[i]==0x7d)&&(Instr[i+1]==0x02))
		{
		   Outstr[decode_len]=0x7e;
		   i++;
		}
		else
		if((Instr[i]==0x7d)&&(Instr[i+1]==0x01))
		{
		   Outstr[decode_len]=0x7d;
		   i++;
		}
		else  
		{
		  Outstr[decode_len]=Instr[i];    
		}
	    decode_len++;
	 }	
    //  3.  The  End
        return decode_len;
}

void CAN2_RxHandler(unsigned char rx_data)
{
   /*
if(rx_data&0x01)
	dayin_ErrorStatus=1;
else if(rx_data&0x02)
	dayin_ErrorStatus=2;
else if(rx_data&0x04)
	dayin_ErrorStatus=3;
else if(rx_data&0x08)
	dayin_ErrorStatus=4;
	*/
      //    rt_kprintf("%c",rx_data);     //ADD for CAN 2 debug
    if(U3_flag)
    	{
           U3_Rx[U3_rxCounter++]=rx_data;
	    if(rx_data==0x7e)
	   {                 
	                     U3_content_len=Protocol_808_Decode_Good(U3_Rx,U3_content,U3_rxCounter);
				  
                            U3_RxProcess();
						   
				U3_flag=0;
				U3_rxCounter=0;
	     }		   
		   
    	}
     else
      if((rx_data==0x7e)&&(U3_flag==0))
      	{
      	   U3_Rx[U3_rxCounter++]=rx_data;
      	   U3_flag=1;
      	}
	else
	    U3_rxCounter=0;	
	
}

void CAN2_putc(char c)
{
	//USART_SendData(USART3,  c); 
	while (!(USART3->SR & USART_FLAG_TXE));  
	USART3->DR = (c & 0x1FF);    
	 //	USART_SendData( USART3, c );
	//	while( USART_GetFlagStatus( USART3, USART_FLAG_TC ) == RESET )  
	//	{
	//	}   
	//	while( USART_GetFlagStatus( USART3, USART_FLAG_TC )  == RESET )   
		//{
	//	}
		//USART_SendData( USART3, c );

}

static rt_err_t   Device_CAN2_init( rt_device_t dev )
{
      GPIO_InitTypeDef  GPIO_InitStructure;
      USART_InitTypeDef USART_InitStructure;    
     NVIC_InitTypeDef NVIC_InitStructure;	  


       //  1 . Clock	  
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable USART3 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

      //   2.  GPIO    
       	/* Configure USART3 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART3 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	    /* Connect alternate function */
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  

     //  3.  Interrupt
	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	
    //   4.  uart  Initial
       USART_InitStructure.USART_BaudRate = 115200;    //CAN2    
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 

	/* Enable USART */
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag( USART3, USART_FLAG_TC );     
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);           


	return RT_EOK;
}

static rt_err_t Device_CAN2_open( rt_device_t dev, rt_uint16_t oflag )  
{
         return RT_EOK;
}
static rt_err_t Device_CAN2_close( rt_device_t dev )
{
        return RT_EOK;
}

static rt_size_t Device_CAN2_read( rt_device_t dev, rt_off_t pos, void* buff, rt_size_t count )
{

        return RT_EOK;
}

static rt_size_t Device_CAN2_write( rt_device_t dev, rt_off_t pos, const void* buff, rt_size_t count )
 {
        unsigned int  Info_len485=0;
	 const char		*p	= (const char*)buff;
	

	Info_len485=(unsigned int)count; 
    	/* empty console output */
		//--------  add by  nathanlnw ---------
  while (Info_len485)
	{
		CAN2_putc (*p++);   
		Info_len485--;
	}
       //--------  add by  nathanlnw  --------	
        return RT_EOK;
  }
static rt_err_t Device_CAN2_control( rt_device_t dev, rt_uint8_t cmd, void *arg )
{
     return RT_EOK;
}


void  Device_CAN2_regist(void ) 
{
       Device_CAN2.type	= RT_Device_Class_Char;
	Device_CAN2.init	=   Device_CAN2_init;
	Device_CAN2.open	=  Device_CAN2_open; 
	Device_CAN2.close	=  Device_CAN2_close;
	Device_CAN2.read	=  Device_CAN2_read;
	Device_CAN2.write	=  Device_CAN2_write;
	Device_CAN2.control =Device_CAN2_control;

	rt_device_register( &Device_CAN2, "CAN2", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE );
	rt_device_init( &Device_CAN2 );
}
