/**
 */

#include "stm32f10x.h"

// Коды команд и тайминги протокола
#define CMD_SHUTTER   0x2D
#define CMD_2S        0x37
#define CMD_STARTSTOP 0x48
#define ADDR          0x1E3A
#define ONE_PULSE     25 // один импульс таймера, если принимаем, что частота ШИМ - 40 кГц
#define TM_MUTE       (600   / ONE_PULSE)
#define TM_ZERO       (600   / ONE_PULSE)
#define TM_ONE        (1200  / ONE_PULSE)
#define TM_START      (2400  / ONE_PULSE)
#define TM_REPEAT     (45000 / ONE_PULSE)
#define REPEAT_LIM    (3)

// Назначение пинов
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

static inline void CONF_CLR(GPIO_TypeDef *port, uint32_t pin)
{
    *((pin < 8) ? &port->CRL : &port->CRH) &= ~(0xF << ((pin % 8) << 2));
}

static inline void CONF_SET_OUT(GPIO_TypeDef *port, uint32_t pin, GPIO_CONF conf)
{
    CONF_CLR(port, pin);
    *((pin < 8) ? &port->CRL : &port->CRH) |= ((((conf) << 2) | GPIO_MODE_OUT50MHZ) << ((pin % 8) << 2));
}

static inline void CONF_SET_IN(GPIO_TypeDef *port, uint32_t pin, GPIO_CONF conf)
{
    CONF_CLR(port, pin);
    *((pin < 8) ? &port->CRL : &port->CRH) |= ((((conf) << 2) | GPIO_MODE_INPUT) << ((pin % 8) << 2));
}

static inline void IRLED_ON(void)
{
    PORT_IRLED->BSRR = (1UL << PIN_IRLED_CTL);
    PORT_GLED->BRR  = (1UL << PIN_GLED);
}

static inline void IRLED_OFF(void)
{
    PORT_IRLED->BRR = (1UL << PIN_IRLED_CTL);
    PORT_GLED->BSRR  = (1UL << PIN_GLED);
}

/// @brief Основная функция
int main()
{
    // GPIO init
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPCEN;

    CONF_SET_OUT(PORT_GLED, PIN_GLED, GPIO_OUT_OPENDRAIN);

    // https://hubstub.ru/stm32/81-stm32-shim.html

    // Тактирование  GPIOA , TIM1, альтернативных функций порта
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN | RCC_APB2ENR_AFIOEN;

    //PA11 push-pull
    CONF_SET_OUT(PORT_IRLED, PIN_IRLED, GPIO_AF_PUSHPULL);

    CONF_SET_OUT(PORT_IRLED, PIN_IRLED_CTL, GPIO_OUT_PUSH_PULL);

    IRLED_OFF();

#if (0)
    TIM1->PSC = 49 * 16; //делитель
#else
    TIM1->PSC = 49; //делитель
#endif
    TIM1->ARR = 3; //значение перезагрузки
    TIM1->CCR4 = 1; //коэф. заполнения
    TIM1->CCER |= TIM_CCER_CC4E | TIM_CCER_CC4P; //настроим на выход канал 4, активный уровень низкий
    TIM1->BDTR |= TIM_BDTR_MOE; //разрешим использовать выводы таймера как выходы
    TIM1->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; //PWM mode 1, прямой ШИМ 4 канал
    TIM1->CR1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS); //считаем вверх, выравнивание по фронту, Fast PWM
    TIM1->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    __enable_irq();
    TIM1->CR1 |= TIM_CR1_CEN; //включаем счётчик

    // Инициализация кнопок
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;

    CONF_SET_OUT(BTN0_PORT0, BTN0_PIN0, GPIO_OUT_OPENDRAIN);
    BTN0_PORT0->BRR = (1UL << BTN0_PIN0);

    CONF_SET_OUT(BTN1_PORT0, BTN1_PIN0, GPIO_OUT_OPENDRAIN);
    BTN1_PORT0->BRR = (1UL << BTN1_PIN0);

    CONF_SET_OUT(BTN2_PORT0, BTN2_PIN0, GPIO_OUT_OPENDRAIN);
    BTN2_PORT0->BRR = (1UL << BTN2_PIN0);

    CONF_SET_IN(BTN0_PORT1, BTN0_PIN1, GPIO_IN_PULL);
    BTN0_PORT1->ODR |= 1UL << BTN0_PIN1;

    CONF_SET_IN(BTN1_PORT1, BTN1_PIN1, GPIO_IN_PULL);
    BTN1_PORT1->ODR |= 1UL << BTN1_PIN1;

    CONF_SET_IN(BTN2_PORT1, BTN2_PIN1, GPIO_IN_PULL);
    BTN2_PORT1->ODR |= 1UL << BTN2_PIN1;

    // Бесконечно
    for (;;)
    {
        //__WFI();
    }
}

uint32_t cmd_curr;
uint32_t cmd_ops;
uint32_t delay_data;
uint32_t repeat_cnt;

static inline void start(typeof(repeat_cnt) repeat)
{
    cmd_ops = (ADDR << 7) | cmd_curr;
    repeat_cnt = repeat;
    delay_data = TM_START + TM_MUTE;
    IRLED_ON();
}

static inline void keybrd(void)
{
    static uint32_t prev = 0;
    static uint32_t push = 0;
    uint32_t curr = 0;

    if ((BTN0_PORT1->IDR & (1 << BTN0_PIN1)) == 0)
    {
        curr = CMD_SHUTTER;
    }
    if ((BTN1_PORT1->IDR & (1 << BTN1_PIN1)) == 0)
    {
        curr = CMD_2S;
    }
    if ((BTN2_PORT1->IDR & (1 << BTN2_PIN1)) == 0)
    {
        curr = CMD_STARTSTOP;
    }

    if (prev == curr)
    {
        static uint32_t push = 0;
        
        if (curr)
        {
            if (!push)
            {
                push = 1;
                cmd_curr = curr;
                start(REPEAT_LIM - 1);
            }
        }
        else
            push = 0;
    }
    prev = curr;
}

void TIM1_UP_IRQHandler(void)
{
    TIM1->SR &= ~TIM_SR_UIF;

    static uint32_t delay_repeat = TM_REPEAT;

    if (--delay_repeat == 0)
    {
        delay_repeat = TM_REPEAT;

        if (repeat_cnt)
            start(repeat_cnt - 1);
        else
            keybrd();
    }

    if (delay_data == TM_MUTE)
    {
        IRLED_OFF();
    }

    if (!delay_data)
    {
        if (cmd_ops)
        {
            delay_data = ((cmd_ops & 1) ? TM_ONE : TM_ZERO) + TM_MUTE;
            cmd_ops >>= 1;
            IRLED_ON();
        }
    }
    else
        delay_data--;
}
