
#include "UART.h"
/*!
 *  @brief      ���ڷ���һ���ֽ�
 *  @param      UART_HandleTypeDef *huart   ����ָ�루��&huart1��
 *  @param      ch          ��Ҫ���͵��ֽ�
 *  Sample usage:       uart_putchar(&huart1,'5');ͨ��UART1�����ַ�5
 */
void uart_putchar (UART_HandleTypeDef *huart,char ch)
{
	    uint8_t *addr=(uint8_t *)&ch;
			HAL_UART_Transmit(huart,addr,1,0);
	    while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TC)!=1);//�ȴ��������
}
/*!
 *  @brief      ���ڽ���һ���ֽ�
 *  @param      UART_HandleTypeDef *huart   ����ָ�루��&huart1��
 *  @param      ch          �������ݴ�ŵĵ�ַ
 *  Sample usage:       uart_getchar(&huart2,&c);;ͨ��UART12����һ���ַ������c�ַ�������
 */
void uart_getchar(UART_HandleTypeDef *huart,char *ch)
{
	  if (HAL_UART_Receive(huart, (uint8_t *)ch, 1, 5)== HAL_ERROR)
  {
    Error_Handler();
  }
}
/*!
 *  @brief      ��ѯ����1���ַ�
 *  @param      UART_HandleTypeDef *huart   ����ָ�루��&huart1��
 *  @param      ch          ���յ�ַ
 *  @return     1Ϊ���ճɹ���0Ϊ����ʧ��
 *  Sample usage:       char ch ;
                        if( uart_querychar (&huart2,&ch))     //��ѯ����1���ַ������浽 ch��
                        {
                            printf("�ɹ����յ�һ���ֽ�");
                        }
 */
uint8_t uart_querychar (UART_HandleTypeDef *huart, char *ch)
{
	char str=0;
	uart_getchar(huart,&str);
    if(str)           //��ѯ�Ƿ���ܵ�����
    {      
			 *ch=str;
        return  1;                                             //���� 1 ��ʾ���ճɹ�
    }
		*ch=0;
    return 0;                                   //����0��ʾ����ʧ��
}
/*!
 *  @brief      ��ѯ����buff
 *  @param      UART_HandleTypeDef *huart   ����ָ�루��&huart1��
 *  @param      str         ���յ�ַ
 *  @param      max_len     �����ճ���
 *  @return     ���յ����ֽ���Ŀ
 *  Sample usage:       char buff[100];
                        uint32 num;
                        num = uart_pendbuff (UART3,&buff,100);
                        if( num != 0 )
                        {
                            printf("�ɹ����յ�%d���ֽ�:%s",num,buff);
                        }
 */
uint8_t uart_querybuff (UART_HandleTypeDef *huart, char *buff, uint32_t max_len)
{
    uint32_t i = 0;
	     while(uart_querychar(huart,buff+i))
			 {
				 i++; 		
				 if(i>=max_len)	
				 return i;	
			 }
			 return i;
}

	/**
 *  @brief      �����ַ���
 *  @param      huart       ģ��ţ�UART0~UART5��
 *  @param      str         �ַ�����ַ
 *  @since      v5.0
 *  Sample usage:       uart_putstr (&huart1,"1234567"); //ʵ�ʷ�����7���ֽ�
  */
void uart_putstr (UART_HandleTypeDef *huart,char str[])
{
    while(*str)
    {
        uart_putchar(huart,*str++);
    }
}
/*!
 *  @brief      ����ָ��len���ֽڳ������� ������ NULL Ҳ�ᷢ�ͣ�
 *  @param      UARTn_e       ģ��ţ�UART0~UART5��
 *  @param      buff        �����ַ
 *  @param      len         ��������ĳ���
 *  @since      v5.0
 *  Sample usage:       uart_putbuff (&huart1,"1234567", 3); //ʵ�ʷ�����3���ֽ�'1','2','3'
 */
void uart_putbuff (UART_HandleTypeDef *huart, uint8_t *buff, uint32_t len)
{
    while(len--)
    {
        uart_putchar(huart, (char)*buff);
        buff++;
    }
}
/*!
 *  @brief      ɽ��๦�ܵ���������λ��������ʾ������ʾ����
 *  @param      wareaddr    ����������ʼ��ַ
 *  @param      waresize    ��������ռ�ÿռ�Ĵ�С
 *  @since      v5.0
*  Sample usage:
���ȶ��岨�����飬��uint16_t var[2];
Ȼ��������Ա��ֵ����var[0]=1;
�����ú���   vcan_sendware((uint8_t *)var, sizeof(var));
 */
void vcan_sendware(void *wareaddr, uint32_t waresize)
{
#define CMD_WARE     3
    uint8_t cmdf[2] = {CMD_WARE, ~CMD_WARE};    //���ڵ��� ʹ�õ�ǰ����
    uint8_t cmdr[2] = {~CMD_WARE, CMD_WARE};    //���ڵ��� ʹ�õĺ�����
    HAL_UART_Transmit(&husart_debug,cmdf,sizeof(cmdf),100);
		HAL_UART_Transmit(&husart_debug,(uint8_t *)wareaddr,waresize,100);
		HAL_UART_Transmit(&husart_debug,cmdr,sizeof(cmdr),100);
//    uart_putbuff(&husart_debug, cmdf, sizeof(cmdf));    //�ȷ���ǰ����
//    uart_putbuff(&husart_debug, (uint8_t *)wareaddr, waresize);    //��������
//    uart_putbuff(&husart_debug, cmdr, sizeof(cmdr));    //���ͺ�����

}
/**
  * ��������: �ض���c�⺯��printf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&husart_debug, (uint8_t *)&ch, 1, 0);
	while(__HAL_UART_GET_FLAG(&husart_debug,UART_FLAG_TC)!=1);//�ȴ��������
  return ch;
}

/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    �����ı䴮�ں�&huart1���ɸı�printf���õĴ���
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&husart_debug,&ch, 1, 5);
  return ch;
}
/**
  * ��������: ���ڽ�����ɻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
		printf("�����жϱ������\n");	
}
