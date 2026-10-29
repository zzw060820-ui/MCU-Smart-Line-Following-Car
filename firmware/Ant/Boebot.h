/*----------------------------------------------------------------------------------------
                      delay functions us&ms
----------------------------------------------------------------------------------------*/
 /*内嵌汇编的方法:一般格式 
   #pragma ASM
     nop  ；
    #pragma ENDASM，*/
void delay_nus(unsigned int i)  //延时:i>=12 ,i的最小延时单12 us
{ 
  i=i/10;
  while(--i);
}   

void delay_nms(unsigned int n)  //延时n ms
{ 
  n=n+1;
  while(--n)  
  delay_nus(900);         //延时 1ms,同时进行补偿
}   
