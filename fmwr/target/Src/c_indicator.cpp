#include <c_indicator.h>
#include "cmsis_os.h"
#include "gpio.h"

#define SEG_A (LL_GPIO_PIN_14)
#define SEG_B (LL_GPIO_PIN_15)
#define SEG_C (LL_GPIO_PIN_13)
#define SEG_D (LL_GPIO_PIN_11)
#define SEG_E (LL_GPIO_PIN_10)
#define SEG_F (LL_GPIO_PIN_8)
#define SEG_G (LL_GPIO_PIN_9)
#define SEG_P (LL_GPIO_PIN_12)

#define ALL_SEGS (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G | SEG_P)

#define DIGIT2 (LL_GPIO_PIN_13)
#define DIGIT1 (LL_GPIO_PIN_14)
#define DIGIT0 (LL_GPIO_PIN_15)

#define ALL_DIGITS (DIGIT0 | DIGIT1 | DIGIT2)

const uint32_t Indicator::char_to_segments[] =
{
    (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F),
    (SEG_B | SEG_C),
    (SEG_A | SEG_B | SEG_G | SEG_D | SEG_E),
    (SEG_A | SEG_B | SEG_C | SEG_D | SEG_G),
    (SEG_B | SEG_C | SEG_F | SEG_G),
    (SEG_A | SEG_C | SEG_D | SEG_G | SEG_F),
    (SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),
    (SEG_A | SEG_B | SEG_C),
    (SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G),
    (SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G)
};

const uint32_t Indicator::digits[size] =
{
    DIGIT0,
    DIGIT1,
    DIGIT2
};

uint32_t Indicator::decode(char ch)
{
    switch (ch)
    {
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            return char_to_segments[ch - '0'];

        case ' ':
        default:
            return 0;
    }
}

void Indicator::set(char *str)
{
    for(size_t i = 0; i < size && str[i]; i++)
    {
        segments[i] = decode(str[i]);
    }
}

void Indicator::runtask()
{
    for(;;)
    {
        for(size_t dg = 0; dg < size; dg++)
        {
            LL_GPIO_SetOutputPin(GPIOC, ALL_DIGITS);
            LL_GPIO_SetOutputPin(GPIOB, ALL_SEGS);

            LL_GPIO_ResetOutputPin(GPIOC, digits[dg]);
            LL_GPIO_ResetOutputPin(GPIOB, segments[dg]);

            osDelay(1);
        }
    }
}