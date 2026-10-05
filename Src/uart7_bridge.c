#include "main.h"

volatile uint8_t uart7_last_rx;
volatile uint32_t uart7_rx_count;

void UART7_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(UART7) &&
        LL_USART_IsEnabledIT_RXNE(UART7))
    {
        uart7_last_rx = LL_USART_ReceiveData8(UART7);
        uart7_rx_count++;

#ifdef BIKE_UART7_ECHO
        while (!LL_USART_IsActiveFlag_TXE(UART7))
        {
        }
        LL_USART_TransmitData8(UART7, uart7_last_rx);
#endif
    }

    if (LL_USART_IsActiveFlag_ORE(UART7))
    {
        LL_USART_ClearFlag_ORE(UART7);
    }
}
