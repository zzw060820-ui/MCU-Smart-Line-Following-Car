#include <stdio.h>
unsigned char TestReadP0(void);
#define P0 TestReadP0()
extern unsigned char P1_0, P1_1;
void uart_Init(void);
