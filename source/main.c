/**
 */

#include "stm32f10x.h"

#define PORT_GLED GPIOC
#define PIN_GLED 13
#define PORT_IRLED GPIOA
#define PIN_IRLED 11
#define PIN_IRLED_CTL 12

#define BTN0_PORT0 GPIOA
#define BTN0_PIN0 1
#define BTN0_PORT1 GPIOC
#define BTN0_PIN1 15

#define BTN1_PORT0 GPIOA
#define BTN1_PIN0 6
#define BTN1_PORT1 GPIOA
#define BTN1_PIN1 4

#define BTN2_PORT0 GPIOB
#define BTN2_PIN0 11
#define BTN2_PORT1 GPIOB
#define BTN2_PIN1 1

/// Port Mode
typedef enum
{
    GPIO_MODE_INPUT     = 0x00, ///< GPIO is input
    GPIO_MODE_OUT10MHZ  = 0x01, ///< Max output Speed 10MHz
    GPIO_MODE_OUT2MHZ   = 0x02, ///< Max output Speed  2MHz
    GPIO_MODE_OUT50MHZ  = 0x03  ///< Max output Speed 50MHz
} GPIO_MODE;

/// Port Conf
typedef enum
{
    GPIO_OUT_PUSH_PULL  = 0x00, ///< general purpose output push-pull
    GPIO_OUT_OPENDRAIN  = 0x01, ///< general purpose output open-drain
    GPIO_AF_PUSHPULL    = 0x02, ///< alternate function push-pull
    GPIO_AF_OPENDRAIN   = 0x03, ///< alternate function open-drain
    GPIO_IN_ANALOG      = 0x00, ///< input analog
    GPIO_IN_FLOATING    = 0x01, ///< input floating
    GPIO_IN_PULL        = 0x02, ///< alternate function push-pull
} GPIO_CONF;

#define CONF_CLR(pin) (0xF << ((pin) << 2))
#define CONF_SET(pin, conf) ((((conf) << 2) | GPIO_MODE_OUT50MHZ) << ((pin) << 2))
#define CONF_SET_IN(pin, conf) ((((conf) << 2) | GPIO_MODE_INPUT) << ((pin) << 2))

/// @brief Основная функция
int main()
{
    // GPIO init
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPCEN;

    PORT_GLED->CRH &= ~(CONF_CLR(PIN_GLED - 8));
    PORT_GLED->CRH |= CONF_SET(PIN_GLED - 8, GPIO_OUT_OPENDRAIN);

    // https://hubstub.ru/stm32/81-stm32-shim.html

    // Тактирование  GPIOA , TIM1, альтернативных функций порта
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN;

    //PA11 push-pull
    PORT_IRLED->CRH &= ~(CONF_CLR(PIN_IRLED - 8));
    PORT_IRLED->CRH |= CONF_SET(PIN_IRLED - 8, GPIO_AF_OPENDRAIN);
    
    PORT_IRLED->CRH &= ~(CONF_CLR(PIN_IRLED_CTL - 8));
    PORT_IRLED->CRH |= CONF_SET(PIN_IRLED_CTL - 8, GPIO_OUT_PUSH_PULL);
    PORT_IRLED->BSRR = (1UL << PIN_IRLED_CTL);

#if (0)
	//делитель
	TIM1->PSC = 7999;
	//значение перезагрузки
    TIM1->ARR = 1000;
	//коэф. заполнения
	TIM1->CCR4 = 1;
#else
	//делитель
	TIM1->PSC = 49/* * 16*/;
	//значение перезагрузки
    TIM1->ARR = 4;
	//коэф. заполнения
	TIM1->CCR4 = 1;
#endif
	//настроим на выход канал 4, активный уровень низкий 
	TIM1->CCER |= TIM_CCER_CC4E | TIM_CCER_CC4P;
	//разрешим использовать выводы таймера как выходы
	TIM1->BDTR |= TIM_BDTR_MOE;
	//PWM mode 1, прямой ШИМ 4 канал
    TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	//считаем вверх, выравнивание по фронту, Fast PWM
	TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
    
    TIM1->DIER |= TIM_DIER_UIE;
    
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    
    __enable_irq();

	//включаем счётчик
	TIM1->CR1 |= TIM_CR1_CEN;
    
    // Инициализация кнопок
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    
    BTN0_PORT0->CRL &= ~(CONF_CLR(BTN0_PIN0));
    BTN0_PORT0->CRL |= CONF_SET(BTN0_PIN0, GPIO_OUT_OPENDRAIN);
    BTN0_PORT0->BRR = (1UL << BTN0_PIN0);

    BTN1_PORT0->CRL &= ~(CONF_CLR(BTN1_PIN0));
    BTN1_PORT0->CRL |= CONF_SET(BTN1_PIN0, GPIO_OUT_OPENDRAIN);
    BTN1_PORT0->BRR = (1UL << BTN1_PIN0);

    BTN2_PORT0->CRH &= ~(CONF_CLR(BTN2_PIN0 - 8));
    BTN2_PORT0->CRH |= CONF_SET(BTN2_PIN0 - 8, GPIO_OUT_OPENDRAIN);
    BTN2_PORT0->BRR = (1UL << BTN2_PIN0);

    BTN0_PORT1->CRH &= ~(CONF_CLR(BTN0_PIN1 - 8));
    BTN0_PORT1->CRH |= CONF_SET_IN(BTN0_PIN1 - 8, GPIO_IN_PULL);
    BTN0_PORT1->ODR |= 1UL << BTN0_PIN1;

    BTN1_PORT1->CRL &= ~(CONF_CLR(BTN1_PIN1));
    BTN1_PORT1->CRL |= CONF_SET_IN(BTN1_PIN1, GPIO_IN_PULL);
    BTN1_PORT1->ODR |= 1UL << BTN1_PIN1;

    BTN2_PORT1->CRL &= ~(CONF_CLR(BTN2_PIN1));
    BTN2_PORT1->CRL |= CONF_SET_IN(BTN2_PIN1, GPIO_IN_PULL);
    BTN2_PORT1->ODR |= 1UL << BTN2_PIN1;

    // Бесконечно
    for (;;)__WFI();
}

#define CMD_SHUTTER   0x2D
#define CMD_2S        0x37
#define CMD_STARTSTOP 0x48
#define ADDR          0x1E3A
#define ONE_PULSE     25 // один импульс таймера, если принимаем, что частота ШИМ - 40 кГц
#define TM_ZERO       (600   / ONE_PULSE)
#define TM_MUTE       (600   / ONE_PULSE)
#define TM_ONE        (1200  / ONE_PULSE)
#define TM_START      (2400  / ONE_PULSE)
#define TM_REPEAT     (11000 / ONE_PULSE)

uint32_t cmd_first  = (ADDR << 8) | CMD_SHUTTER;
uint32_t cmd_repeat = (ADDR << 8) | CMD_SHUTTER;
uint32_t delay      = TM_START + TM_MUTE;
uint32_t repeat     = 1;

void TIM1_UP_IRQHandler(void)
{
    TIM1->SR &= ~TIM_SR_UIF;

    if (delay == TM_MUTE)
    {
        PORT_IRLED->BRR = (1UL << PIN_IRLED_CTL);

        if (!cmd_repeat && cmd_first)
        {
            cmd_repeat = cmd_first;
            cmd_first = 0;
            delay = TM_REPEAT;
            repeat = 1;
        }
    }

    if (--delay == 0)
    {
        if (!cmd_first)
        {
            if (repeat)
            {
                delay = TM_START + TM_MUTE;
                PORT_IRLED->BSRR = (1UL << PIN_IRLED_CTL);
                repeat = 0;
                return;
            }
            else if (!cmd_repeat)
            {
                delay = TM_REPEAT;
                // Ожидаем ввода команды
                {
                    if ((BTN0_PORT1->IDR & (uint16_t)(1 << BTN0_PIN1)) == 0)
                    {
                        __NOP();
                    }
                    if ((BTN1_PORT1->IDR & (uint16_t)(1 << BTN1_PIN1)) == 0)
                    {
                        __NOP();
                    }
                    if ((BTN2_PORT1->IDR & (uint16_t)(1 << BTN2_PIN1)) == 0)
                    {
                        __NOP();
                    }
                }
                
                return;
            }
        }

        delay = (cmd_repeat & 1) ? TM_ONE + TM_MUTE : TM_ZERO + TM_MUTE;
        PORT_IRLED->BSRR = (1UL << PIN_IRLED_CTL);

        cmd_repeat >>= 1;
    }

    // PORT_GLED->BSRR = (1UL << PIN_GLED);
    // PORT_GLED->BRR  = (1UL << PIN_GLED);
}
