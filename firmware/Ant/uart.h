/*----------------------------------------------------------------
                     8051串口中断驱动程序
--------------------------------------------------------------*/
#include <AT89X52.h>
#include <stdio.h>
#define XTAL 11059200
#define baudrate 9600

#define OLEN 8	      //串行发送缓冲区大小
unsigned char ostart;  //发送缓冲区起始索引
unsigned char oend;      //发送缓冲区结束索引
char idata outbuf[OLEN]; //发送缓冲区存储数组

#define ILEN 8	        //串行接收缓冲区大小
unsigned char istart;   //接收缓冲区起始索引
unsigned char iend;     //接收缓冲区结束索引
char idata inbuf[ILEN]; //接收缓冲存储数组

bit bdata sendfull;    //发送缓冲区满标志
bit bdata sendactive;  //发送有效标志
/*串行中断服务程序*/
static void com_isr(void) interrupt 4 using 1        //串口中断
{ 
   //-------------接收数据中断--------------
  char c;
  if(RI)			     //接收中断
  { 				     
    c=SBUF;	    //读字符
    RI=0;	    //清接收中断请求标志
    if(istart+ILEN!=iend)
    { 
      inbuf[iend++&(ILEN-1)]=c;   //缓冲区接收数据
    }
  }

  //-------------发送数据中断--------------
  if(TI)
  { 
    TI=0;	 //清发送中断标志
    if(ostart!=oend)
    {  
       SBUF=outbuf[ostart++&(OLEN-1)];//向发送缓冲区传送字符
       sendfull=0;		      //设置缓冲区满标志位
     }
     else
     { 
        sendactive=0; 	          //设置发送无效
     }
   }
}   

//PUTBUF: 写字符到SBUF或发送缓冲区
void putbuf(char c)
{ 
    if(!sendfull)	 //如果缓冲区不满就发送
    {
      if(!sendactive)   
      { 
          sendactive=1; //直接发送一个字符
          SBUF=c;       //写到SBUF启动缓冲区
      }
      else 
      { 
        ES=0;	      //暂时串行口关闭中断
        outbuf[oend++&(OLEN-1)]=c;  //向发送缓冲区传送字符
        if(((oend^ostart)&(OLEN-1))==0)  
         { sendfull=1;}    //设置缓冲区满标志
         ES=1;	      //打开串行口中断
      }
    }
}

//putchar:中断控制putchar函数
//替换标准库函数putchar程序
//printf函数使用putchar输出一个字符
char putchar (char c)
{ 
  if (c=='\n')	         //增加新的行
    {
      while(sendfull);   //等待发送缓冲区空
      putbuf(0x0D);      //对新行在LF前发送CR
    }
  while(sendfull); 
  putbuf(c);
  return(c);
}

//_getkey:中断控制_getkey函数
//替换标准库函数_getkey程序
//getchar和gets函数使用_getkey
char _getkey(void)
{ 
  char c;
  while(iend==istart)      //判断接收缓冲区起始索引是否等于接收缓冲区结束索引
  {;}
  ES=0;
  c=inbuf[istart++&(ILEN-1)];
  ES=1;
  return(c);
}

/* 初始化串行口和UART波特率函数*/
void com_initialize(void)
{ 
  istart=0;
  iend=0;
  ostart=0;
  oend=0;
  sendactive=0;
  sendfull=0;

  TMOD |=0x20;   //设置定时器T/C1工作在方式2,定时1工作于自动重载模式
  SCON=0x50;     //设置串行口工作方式1：SCON格式 |M0|M1|M2|REN|TB8|RB8|TI|RI
  TH1=0xfd;      //波特率9600
  TL1=0xfd;
  TR1=1;         //启动定时器
  ES=1;	        //开串行口中断
}    

void uart_Init()
{ 
  com_initialize();
  EA=1;	        //CPU开总中断
}

  
