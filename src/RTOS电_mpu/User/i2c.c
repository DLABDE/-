#include "i2c.h"

static void i2c_Delay(void)
{
	uint8_t i;
	for (i = 0; i < 35; i++);	//循环次数为20~250时都能通讯正常
}

//初始化IIC
void IIC_Init(void)
{			
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(GPIO_CLK, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_SCLK_Pin | GPIO_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIO_X, &GPIO_InitStructure);//初始化
	SCL_Set();
	SDA_Set();
}
void SDA_OUT()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(GPIO_CLK, ENABLE);//使能GPIOB时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIO_X, &GPIO_InitStructure);//初始化
}
void SDA_IN()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_SDA_Pin;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIO_X, &GPIO_InitStructure);//初始化
}


//产生IIC起始信号
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	SDA_Set();	  	  
	SCL_Set();
	i2c_Delay();
 	SDA_Clr();//START:when CLK is high,DATA change form high to low 
	i2c_Delay();
	SCL_Clr();//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	SCL_Clr();
	SDA_Clr();//STOP:when CLK is high DATA change form low to high
 	i2c_Delay();
	SCL_Set(); 
	SDA_Set();//发送I2C总线结束信号
	i2c_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
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
	SCL_Clr();//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    SCL_Clr();//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
			if((txd&0x80)>>7)	SDA_Set();
			else	SDA_Clr();
      txd<<=1; 	  
		  i2c_Delay();   //对TEA5767这三个延时都是必须的
		  SCL_Set();
		  i2c_Delay(); 
		  SCL_Clr();	
		  i2c_Delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_NAck();//发送nACK
    else
        IIC_Ack(); //发送ACK   
    return receive;
}
