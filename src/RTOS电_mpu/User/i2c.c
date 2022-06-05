#include "i2c.h"

static void i2c_Delay(void)
{
	uint8_t i;
	for (i = 0; i < 35; i++);	//ѭ������Ϊ20~250ʱ����ͨѶ����
}

//��ʼ��IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(GPIO_CLK, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_SCLK_Pin | GPIO_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIO_X, &GPIO_InitStructure);//��ʼ��
	SCL_Set();
	SDA_Set();
}
void SDA_OUT()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(GPIO_CLK, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIO_X, &GPIO_InitStructure);//��ʼ��
}
void SDA_IN()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIO_X, &GPIO_InitStructure);//��ʼ��
}


//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	SDA_Set();	  	  
	SCL_Set();
	i2c_Delay();
 	SDA_Clr();//START:when CLK is high,DATA change form high to low 
	i2c_Delay();
	SCL_Clr();//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	SCL_Clr();
	SDA_Clr();//STOP:when CLK is high DATA change form low to high
 	i2c_Delay();
	SCL_Set(); 
	SDA_Set();//����I2C���߽����ź�
	i2c_Delay();							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	SDA_Set();i2c_Delay();	   
	SCL_Set();i2c_Delay();	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_Clr();//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	SCL_Clr();
	SDA_OUT();
	SDA_Clr();
	i2c_Delay();
	SCL_Set();
	i2c_Delay();
	SCL_Clr();
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	SCL_Clr();
	SDA_OUT();
	SDA_Set();
	i2c_Delay();
	SCL_Set();
	i2c_Delay();
	SCL_Clr();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    SCL_Clr();//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
			if((txd&0x80)>>7)	SDA_Set();
			else	SDA_Clr();
      txd<<=1; 	  
		  i2c_Delay();   //��TEA5767��������ʱ���Ǳ����
		  SCL_Set();
		  i2c_Delay(); 
		  SCL_Clr();	
		  i2c_Delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        SCL_Clr(); 
        i2c_Delay();
		SCL_Set();
        receive<<=1;
        if(READ_SDA)receive++;   
		i2c_Delay(); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}
