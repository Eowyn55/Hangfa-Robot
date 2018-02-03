#include "drv_button.h"

GPIO_TypeDef *Button_GPIO_PORT[BUTTONn] = { Button2_GPIO_PORT, Button3_GPIO_PORT };
const INT16U    Button_GPIO_PIN[BUTTONn] = { Button2_PIN, Button3_PIN };
const INT32U    Button_GPIO_CLK[BUTTONn] = { Button2_GPIO_CLK, Button3_GPIO_CLK };
const INT8U     Button_EXTI_PortSource[BUTTONn] = { Button2_EXTI_PortSource, Button3_EXTI_PortSource };
const INT8U     Button_EXTI_PinSource[BUTTONn] = { Button2_EXTI_PinSource, Button3_EXTI_PinSource };
const INT32U    Button_EXTI_Line[BUTTONn] = { Button2_EXTI_Line, Button3_EXTI_Line };
const INT8S     Button_EXTI_IRQn[BUTTONn] = { Button2_EXTI_IRQn, Button3_EXTI_IRQn };

//============================================================================
// ���ƣ�drv_button_line_init
// ���ܣ�һλ�������߳�ʼ��
// ������Button��Ҫ��ʼ���İ�������
//      EnableInt���Ƿ�ʹ���жϣ�1Ϊ�жϣ�0Ϊ��ѯ
// ���أ���
// ˵����button_init����
//============================================================================
void drv_button_line_init( ButtonType Button, INT8U EnableInt )
{
    EXTI_InitTypeDef   EXTI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;

    // ʹ�ܿ��߶˿�ʱ��
    RCC_AHB1PeriphClockCmd(Button_GPIO_CLK[Button], ENABLE);

    // ���ÿ���Ϊ��������ģʽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = Button_GPIO_PIN[Button];
    GPIO_Init(Button_GPIO_PORT[Button], &GPIO_InitStructure);

    if( EnableInt > 0 )
    {
        // �����ж��źŵ�����
        SYSCFG_EXTILineConfig(Button_EXTI_PortSource[Button], Button_EXTI_PinSource[Button]);

        // �����ж��ߣ��жϻ��¼�ģʽ
        EXTI_InitStructure.EXTI_Line = Button_EXTI_Line[Button];
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;
        EXTI_Init(&EXTI_InitStructure);

        // �����ж����ȼ�
        NVIC_InitStructure.NVIC_IRQChannel = Button_EXTI_IRQn[Button];
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
}

//============================================================================
// ���ƣ�drv_button_init
// ���ܣ��������߳�ʼ��
// ��������
// ���أ���
// ˵����ϵͳ��ʼ��ʱ����
//============================================================================
void drv_button_init( void )
{
    INT8U i;

    // ʹ�� SYSCFG ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    for( i = 0; i < BUTTONn; i++ )
    {
        drv_button_line_init((ButtonType )i, 0);
    }
}

//============================================================================
// ���ƣ�drv_button_get_status
// ���ܣ���ȡ����״̬
// ������Button��Ҫ��ȡ״̬�İ���
// ���أ���������״̬��1Ϊ���£�0Ϊ�ͷ�
// ˵������������ʱΪ�͵�ƽ
//============================================================================
INT8U drv_button_get_status( ButtonType Button )
{
    if( ((Button_GPIO_PORT[Button]->IDR) & Button_GPIO_PIN[Button]) == 0 )
    {
        return ( 1 );
    }
    return ( 0 );
}

////============================================================================
//// ���ƣ�button2_3_irq
//// ���ܣ�����2��3�жϷ������
//// ��������
//// ���أ���
//// ˵������
////============================================================================
//void button2_3_irq(void)
//{
//    if( EXTI_GetITStatus(Button_EXTI_Line[Button1]) == SET )
//    {
//        debug_printf("button1 interrupt.\r\n");
//        // ���ж��߹���λ
//        EXTI_ClearITPendingBit(Button_EXTI_Line[Button1]);
//    }
//    if( EXTI_GetITStatus(Button_EXTI_Line[Button2]) == SET )
//    {
//        debug_printf("button2 interrupt.\r\n");
//        // ���ж��߹���λ
//        EXTI_ClearITPendingBit(Button_EXTI_Line[Button2]);
//    }
//}










