#ifndef __SHELL_PORT_H
#define __SHELL_PORT_H
#include "shell.h"


/* ��shell����Ϊ�ⲿ�������ڴ����жϻص������л�Ҫʹ�� */
extern Shell shell;

/* �����Լ���д�ĳ�ʼ������ */
void User_Shell_Init(void);
/* letter_shell���Ժ��� */
int letter_shell_demo(int i, char ch, char *str); 

#endif /* _SHELL_PORT_H */
