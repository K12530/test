#include "stm32f4xx.h"


const char Local_ADDR[TX_ADR_WIDTH]={0xFF,0x20,0x02};
const char Device_ADDR[TX_ADR_WIDTH]={0xFF,0x10,0x01};

double pitch,roll,yaw;


char NRF24L01_Status = 0x00;//bit7：发送模式；bit6：发送完成；bit5：发送失败
							//bit3：接收模式；bit2：接收完成；bit1：数据解析完成；

//初始化24L01的IO口
void NRF24L01_Init(void)
{ 	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	
    //CSN	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;				
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//IRQ
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;				
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);

	//CE
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);
			
	NRF24L01_CE=0; 			//使能24L01
	NRF24L01_CSN=1;			//SPI片选取消 
	
	printf("运行到此\r\n");	 
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,0x01);//地址3位
	
	printf("123466\r\n");	 
	
	while(NRF24L01_Check());
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P1,(u8*)Local_ADDR,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P1,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
  	
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x03);     //使能通道0~3的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x03); //使能通道0~3的接收地址 
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x0F);//设置自动重发间隔时间:250us + 86us;最大自动重发次数:5次
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,10);       //设置RF通道为40;所有的模块要求一样
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   	
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;											  //PWR_UP=1,EN_CRC=1,16BIT_CRC,开启所有中断
	
	printf("NRF24L01 OK\r\n");	 	 
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//检测24L01是否存在
//返回值:0，成功;1，失败
//********************************************************************
u8 NRF24L01_Check(void)
{
	u8 buf[3]={0XA5,0XA5,0XA5},buf2[3];
	u8 i;   	 
	NRF24L01_Write_Buf( NRF_WRITE_REG + TX_ADDR,buf,3);//写入5个字节的地址.
	NRF24L01_Read_Buf( TX_ADDR,buf2,3); //读出写入的地址
	for(i=0;i<3;i++)
	{
		if(buf2[i]!=0XA5)break;
	}
	
	if(i!=3)return 1;//检测24L01错误	
	return 0;		 //检测到24L01
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
//********************************************************************
u8 NRF24L01_TxPacket(u8 *txbuf,const char *TX_addr)
{
	u8 sta;
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR, (u8*)TX_addr, TX_ADR_WIDTH);//写TX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)TX_addr,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
	
	delay_ms(1);
	NRF24L01_CE=0;
	delay_ms(1);
 	NRF24L01_CE=1;//启动发送	   
	
	while(NRF24L01_IRQ!=0);//等待发送完成
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	NRF24L01_CE=0;
	if(sta&MAX_TX)//达到最大重发次数
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //清除中断标志
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器 
//		printf("发送失败\r\n");
		NRF24L01_Status |= 0x20;
		return MAX_TX; 
	}
	if(sta&TX_OK)//发送完成
	{
//		printf("发送成功 \r\n");
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Status |= 0x40;
		return TX_OK;
	}
	return 0xff;//其他原因发送失败
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//启动NRF24L01接收一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
//********************************************************************
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
	NRF24L01_CE = 0;
	sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //清除TX_DS或MAX_RT中断标志
	if(sta&RX_OK)//接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); //清除TX_DS或MAX_RT中断标志
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
		NRF24L01_Status |= 0x04;	
		NRF24L01_CE = 1;	
		return 0; 
	}	
	return 1;//没收到任何数据
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了	
//********************************************************************
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  

	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //清除中断标志
	NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //清除RX FIFO寄存器
	NRF24L01_Write_Reg(FLUSH_TX,0xff);				  //清除RX FIFO寄存器
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x3f);//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 

	while(NRF24L01_IRQ==0)
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //清除中断标志
		NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //清除RX FIFO寄存器
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		printf("NRF24L01_IRQ==0\r\n");
	}
	
	NRF24L01_Status = 0x08;
	
  	NRF24L01_CE = 1; //CE为高,进入接收模式 
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
//PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了		   
//CE为高大于10us,则启动发送.
//********************************************************************
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE=0;	    
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //清除中断标志
	NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //清除RX FIFO寄存器
	NRF24L01_Write_Reg(FLUSH_TX,0xff);				  //清除RX FIFO寄存器
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x4e);    //0x0e配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	
	while(NRF24L01_IRQ==0)
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //清除中断标志
		NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //清除RX FIFO寄存器
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		printf("NRF24L01_IRQ==0\r\n");
	}

	NRF24L01_Status = 0x80;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//SPI写寄存器
//reg:指定寄存器地址
//value:写入的值
//********************************************************************
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //使能SPI传输
  	status =SPI2_send_receive_data( reg );//发送寄存器号
	printf("%d\r\n",status);
  	SPI2_send_receive_data(value);      //写入寄存器的值
  	delay_us(10);
	NRF24L01_CSN=1;                 //禁止SPI传输	
  	return(status);       			//返回状态值
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//读取SPI寄存器值
//reg:要读的寄存器
//********************************************************************
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //使能SPI传输		
  	SPI2_send_receive_data( reg );   //发送寄存器号
  	reg_val=SPI2_send_receive_data(0XFF);//读取寄存器内容
  	delay_us(10);
	NRF24L01_CSN = 1;          //禁止SPI传输
  	return(reg_val);           //返回状态值
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//在指定位置读出指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
//********************************************************************
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //使能SPI传输
  	status=SPI2_send_receive_data( reg );//发送寄存器值(位置),并读取状态值   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++) pBuf[u8_ctr]=SPI2_send_receive_data(0XFF);//读出数据
  	delay_us(10);
	NRF24L01_CSN=1;       //关闭SPI传输
  	return status;        //返回读到的状态值
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//在指定位置写指定长度的数据
//reg:寄存器(位置)
//*pBuf:数据指针
//len:数据长度
//返回值,此次读到的状态寄存器值
//********************************************************************
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //使能SPI传输
  	status = SPI2_send_receive_data( reg );//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++) SPI2_send_receive_data(*pBuf++); //写入数据	 
  	delay_us(10);
	NRF24L01_CSN = 1;       //关闭SPI传输
  	return status;          //返回读到的状态值
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
void NRF24L01_NVIC_Init()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);//PA12 连接到中断线12
	
	//外部中断
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;				//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//使能
	EXTI_Init(&EXTI_InitStructure);//配置

	//中断配置
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//外部中断0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//NRF24L01数据解析
//********************************************************************
void NRF24L01_Data_Parse(const char *data)
{
	if( data[0]=='K' )
	{
		
		char x,i,Temporary_data[10];
		int Send_Count;
		if(data[1]=='S')
		{
			for(i=2,x=0;data[i]!=',' && x<10 ;i++,x++)
			{
				Temporary_data[x] = data[i];
			}
			if(x<10)
			{
				Temporary_data[x]='\0';
				pitch = atof(Temporary_data);
			}
			
			for(i++,x=0;data[i]!=',' && x<10 ;i++,x++)
			{
				Temporary_data[x] = data[i];
			}
			if(x<10)
			{
				Temporary_data[x]='\0';
				roll = atof(Temporary_data);
			}
			
			for(i++,x=0;data[i]!='#' && x<10 ;i++,x++)
			{
				Temporary_data[x] = data[i];
			}
			if(x<10)
			{
				Temporary_data[x]='\0';
				yaw = atof(Temporary_data);
			}
			
//			DataScope_Get_Channel_Data(10, 1 );
//			DataScope_Get_Channel_Data(20, 2 );
//			DataScope_Get_Channel_Data(30, 3 );
//			Send_Count = DataScope_Data_Generate(3);
//			for( i = 0 ; i < Send_Count; i++) 
//			{
//				while((USART1->SR&0X40)==0);  
//				USART1->DR = DataScope_OutPut_Buffer[i]; 
//			}
			
			
		}		
		
		if(data[strlen(data)-1]=='#')
		{
//			printf("收到#");
			NRF24L01_Mode_Convert();
		}
		NRF24L01_Status |= 0x02;
	}
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//接收到数据进入中断
//分析接收到的数据
//********************************************************************
void EXTI15_10_IRQHandler()//不进入此中断
{
	extern unsigned char NRF_RX_Data[35],NRF_TX_Data[35];
//	printf("____________________________________\r\n");
	if(EXTI_GetITStatus(EXTI_Line12))
	{
		u8 sta;
		sta=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
		if(sta&RX_OK)//接收到数据
		{
			NRF24L01_RxPacket(NRF_RX_Data);
			printf("%s\r\n",NRF_RX_Data);
			NRF24L01_Data_Parse(NRF_RX_Data);
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line12);
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//TX RX模式转换
//超时：令NRF24L01_Status=0x08或0x80
//********************************************************************
void NRF24L01_Mode_Convert()
{
	if((NRF24L01_Status&0x80) == 0x80)//发送模式，且发送完成
	{
		NRF24L01_RX_Mode();
//		printf("转换为接收模式\r\n");
		TIM3->CNT=0;
	}
	else if((NRF24L01_Status&0x08) == 0x08)//接收模式，且接收完成
	{
		NRF24L01_TX_Mode();
//		printf("转换为发送模式\r\n");
		TIM3->CNT=0;
	}
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//100ms进入
//********************************************************************
void TIM3_Init_NRF24L01()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 50000 - 1; 	//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler = 168; 			//设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//设置时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 	//根据指定的参数初始化TIMx的时间基数单位
 
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM7中断,允许更新中断
	
	TIM_Cmd(TIM3,ENABLE);

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



void TIM3_IRQHandler()
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update))
	{
		while((NRF24L01_Status&0x08)!=0x08)//不处于发送模式
		{
			NRF24L01_RX_Mode();
			printf("RX模式\r\n");
		}
		while(NRF24L01_IRQ==0)
		{
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //清除中断标志
			NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //清除RX FIFO寄存器
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
			printf("NRF24L01_IRQ==0\r\n");
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
