#include "stm32f4xx.h"


const char Local_ADDR[TX_ADR_WIDTH]={0xFF,0x20,0x02};
const char Device_ADDR[TX_ADR_WIDTH]={0xFF,0x10,0x01};

double pitch,roll,yaw;


char NRF24L01_Status = 0x00;//bit7������ģʽ��bit6��������ɣ�bit5������ʧ��
							//bit3������ģʽ��bit2��������ɣ�bit1�����ݽ�����ɣ�

//��ʼ��24L01��IO��
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
			
	NRF24L01_CE=0; 			//ʹ��24L01
	NRF24L01_CSN=1;			//SPIƬѡȡ�� 
	
	printf("���е���\r\n");	 
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_AW,0x01);//��ַ3λ
	
	printf("123466\r\n");	 
	
	while(NRF24L01_Check());
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P1,(u8*)Local_ADDR,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
	NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P1,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
  	
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x03);     //ʹ��ͨ��0~3���Զ�Ӧ��
	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x03); //ʹ��ͨ��0~3�Ľ��յ�ַ 
	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x0F);//�����Զ��ط����ʱ��:250us + 86us;����Զ��ط�����:5��
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,10);       //����RFͨ��Ϊ40;���е�ģ��Ҫ��һ��
	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   	
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;											  //PWR_UP=1,EN_CRC=1,16BIT_CRC,���������ж�
	
	printf("NRF24L01 OK\r\n");	 	 
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//���24L01�Ƿ����
//����ֵ:0���ɹ�;1��ʧ��
//********************************************************************
u8 NRF24L01_Check(void)
{
	u8 buf[3]={0XA5,0XA5,0XA5},buf2[3];
	u8 i;   	 
	NRF24L01_Write_Buf( NRF_WRITE_REG + TX_ADDR,buf,3);//д��5���ֽڵĵ�ַ.
	NRF24L01_Read_Buf( TX_ADDR,buf2,3); //����д��ĵ�ַ
	for(i=0;i<3;i++)
	{
		if(buf2[i]!=0XA5)break;
	}
	
	if(i!=3)return 1;//���24L01����	
	return 0;		 //��⵽24L01
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
//********************************************************************
u8 NRF24L01_TxPacket(u8 *txbuf,const char *TX_addr)
{
	u8 sta;
	
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR, (u8*)TX_addr, TX_ADR_WIDTH);//дTX�ڵ��ַ
	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(u8*)TX_addr,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
	
	delay_ms(1);
	NRF24L01_CE=0;
	delay_ms(1);
 	NRF24L01_CE=1;//��������	   
	
	while(NRF24L01_IRQ!=0);//�ȴ��������
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	NRF24L01_CE=0;
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //����жϱ�־
		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
//		printf("����ʧ��\r\n");
		NRF24L01_Status |= 0x20;
		return MAX_TX; 
	}
	if(sta&TX_OK)//�������
	{
//		printf("���ͳɹ� \r\n");
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Status |= 0x40;
		return TX_OK;
	}
	return 0xff;//����ԭ����ʧ��
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
//********************************************************************
u8 NRF24L01_RxPacket(u8 *rxbuf)
{
	u8 sta;
	NRF24L01_CE = 0;
	sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&RX_OK)//���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); //���TX_DS��MAX_RT�жϱ�־
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
		NRF24L01_Status |= 0x04;	
		NRF24L01_CE = 1;	
		return 0; 
	}	
	return 1;//û�յ��κ�����
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//�ú�����ʼ��NRF24L01��RXģʽ
//����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
//��CE��ߺ�,������RXģʽ,�����Խ���������	
//********************************************************************
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE=0;	  

	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //����жϱ�־
	NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //���RX FIFO�Ĵ���
	NRF24L01_Write_Reg(FLUSH_TX,0xff);				  //���RX FIFO�Ĵ���
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x3f);//���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 

	while(NRF24L01_IRQ==0)
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //����жϱ�־
		NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //���RX FIFO�Ĵ���
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		printf("NRF24L01_IRQ==0\r\n");
	}
	
	NRF24L01_Status = 0x08;
	
  	NRF24L01_CE = 1; //CEΪ��,�������ģʽ 
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//�ú�����ʼ��NRF24L01��TXģʽ
//����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,�����ʺ�LNA HCURR
//PWR_UP,CRCʹ��
//��CE��ߺ�,������RXģʽ,�����Խ���������		   
//CEΪ�ߴ���10us,����������.
//********************************************************************
void NRF24L01_TX_Mode(void)
{														 
	NRF24L01_CE=0;	    
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //����жϱ�־
	NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //���RX FIFO�Ĵ���
	NRF24L01_Write_Reg(FLUSH_TX,0xff);				  //���RX FIFO�Ĵ���
	
	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x4e);    //0x0e���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
	
	while(NRF24L01_IRQ==0)
	{
		NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //����жϱ�־
		NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //���RX FIFO�Ĵ���
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		printf("NRF24L01_IRQ==0\r\n");
	}

	NRF24L01_Status = 0x80;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//SPIд�Ĵ���
//reg:ָ���Ĵ�����ַ
//value:д���ֵ
//********************************************************************
u8 NRF24L01_Write_Reg(u8 reg,u8 value)
{
	u8 status;	
   	NRF24L01_CSN=0;                 //ʹ��SPI����
  	status =SPI2_send_receive_data( reg );//���ͼĴ�����
	printf("%d\r\n",status);
  	SPI2_send_receive_data(value);      //д��Ĵ�����ֵ
  	delay_us(10);
	NRF24L01_CSN=1;                 //��ֹSPI����	
  	return(status);       			//����״ֵ̬
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//��ȡSPI�Ĵ���ֵ
//reg:Ҫ���ļĴ���
//********************************************************************
u8 NRF24L01_Read_Reg(u8 reg)
{
	u8 reg_val;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����		
  	SPI2_send_receive_data( reg );   //���ͼĴ�����
  	reg_val=SPI2_send_receive_data(0XFF);//��ȡ�Ĵ�������
  	delay_us(10);
	NRF24L01_CSN = 1;          //��ֹSPI����
  	return(reg_val);           //����״ֵ̬
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//��ָ��λ�ö���ָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
//********************************************************************
u8 NRF24L01_Read_Buf(u8 reg,u8 *pBuf,u8 len)
{
	u8 status,u8_ctr;	       
  	NRF24L01_CSN = 0;           //ʹ��SPI����
  	status=SPI2_send_receive_data( reg );//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++) pBuf[u8_ctr]=SPI2_send_receive_data(0XFF);//��������
  	delay_us(10);
	NRF24L01_CSN=1;       //�ر�SPI����
  	return status;        //���ض�����״ֵ̬
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//��ָ��λ��дָ�����ȵ�����
//reg:�Ĵ���(λ��)
//*pBuf:����ָ��
//len:���ݳ���
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
//********************************************************************
u8 NRF24L01_Write_Buf(u8 reg, u8 *pBuf, u8 len)
{
	u8 status,u8_ctr;	    
 	NRF24L01_CSN = 0;          //ʹ��SPI����
  	status = SPI2_send_receive_data( reg );//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++) SPI2_send_receive_data(*pBuf++); //д������	 
  	delay_us(10);
	NRF24L01_CSN = 1;       //�ر�SPI����
  	return status;          //���ض�����״ֵ̬
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
void NRF24L01_NVIC_Init()
{
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);//PA12 ���ӵ��ж���12
	
	//�ⲿ�ж�
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;				//LINE0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�ж��¼�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;	//
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//ʹ��
	EXTI_Init(&EXTI_InitStructure);//����

	//�ж�����
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			//�ⲿ�ж�0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//NRF24L01���ݽ���
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
//			printf("�յ�#");
			NRF24L01_Mode_Convert();
		}
		NRF24L01_Status |= 0x02;
	}
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//���յ����ݽ����ж�
//�������յ�������
//********************************************************************
void EXTI15_10_IRQHandler()//��������ж�
{
	extern unsigned char NRF_RX_Data[35],NRF_TX_Data[35];
//	printf("____________________________________\r\n");
	if(EXTI_GetITStatus(EXTI_Line12))
	{
		u8 sta;
		sta=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
		if(sta&RX_OK)//���յ�����
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
//TX RXģʽת��
//��ʱ����NRF24L01_Status=0x08��0x80
//********************************************************************
void NRF24L01_Mode_Convert()
{
	if((NRF24L01_Status&0x80) == 0x80)//����ģʽ���ҷ������
	{
		NRF24L01_RX_Mode();
//		printf("ת��Ϊ����ģʽ\r\n");
		TIM3->CNT=0;
	}
	else if((NRF24L01_Status&0x08) == 0x08)//����ģʽ���ҽ������
	{
		NRF24L01_TX_Mode();
//		printf("ת��Ϊ����ģʽ\r\n");
		TIM3->CNT=0;
	}
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



//********************************************************************
//100ms����
//********************************************************************
void TIM3_Init_NRF24L01()
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = 50000 - 1; 	//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler = 168; 			//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 		//����ʱ�ӷָ�
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 	//����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM7�ж�,��������ж�
	
	TIM_Cmd(TIM3,ENABLE);

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>END



void TIM3_IRQHandler()
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update))
	{
		while((NRF24L01_Status&0x08)!=0x08)//�����ڷ���ģʽ
		{
			NRF24L01_RX_Mode();
			printf("RXģʽ\r\n");
		}
		while(NRF24L01_IRQ==0)
		{
			NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,0x70); 	  //����жϱ�־
			NRF24L01_Write_Reg(FLUSH_RX,0xff);				  //���RX FIFO�Ĵ���
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
			printf("NRF24L01_IRQ==0\r\n");
		}
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
