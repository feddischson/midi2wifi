/* ********************************************************************
 *
 *  Copyright (c) 2018 Christian Haettich [feddischson@gmail.com]
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "midi2wifi/logging.h"
#include "midi2wifi/uart_handler.h"

/** @brief Private UART data.
 */
static struct _uart_data_
{
   int               initialized; /**< Flag to remember, if it already is
                                       initialized */
   int               uart_no;     /**< The selected uart no. */
   int               baud;        /**< The selected baud rate */
   xQueueHandle      tx_queue;    /**< See global tx_queue */
   xQueueHandle      rx_queue;    /**< See global rx_queue */
} uart_data = {
   .initialized = 0,
   .uart_no     = 0,
   .baud        = BIT_RATE_9600,
   .tx_queue    = 0,
   .rx_queue    = 0
};

/** @brief UART-Task
 *  @details
 *    Reads bytes from the rx_queue and writes them out via UART.
 *  @param param Ignored.
 */
void uart_task (void *param )
{
   portBASE_TYPE n;
   uint8_t buf = 0;
   while(1)
   {
      n = xQueueReceive(
            uart_data.rx_queue,
            &buf,
            10
            );
      if( n )
      {
         while (true) {
             uint32_t fifo_cnt = READ_PERI_REG(
                   UART_STATUS(uart_data.uart_no)) &
                                       (UART_TXFIFO_CNT << UART_TXFIFO_CNT_S);

             if ((fifo_cnt >> UART_TXFIFO_CNT_S & UART_TXFIFO_CNT) < 126) {
                 break;
             }
         }

         WRITE_PERI_REG(UART_FIFO(uart_data.uart_no) , buf);
      }
   }
}

/** @brief Local rx-handler, writes the received bytes into the tx_queue.
 *  @details
 *    Copied and adapted from uart.h
 */
LOCAL void uart_rx_handler(void * param)
{
   uint8_t uart_no = uart_data.uart_no;
   uint8_t fifo_len = 0;
   uint8_t buf_idx = 0;
   uint8_t fifo_tmp[128] = {0};
   uint8_t tmp;
   uint32_t uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;
   portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

   while (uart_intr_status != 0x0)
   {
      if (UART_FRM_ERR_INT_ST == (uart_intr_status & UART_FRM_ERR_INT_ST))
      {
          WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_FRM_ERR_INT_CLR);
      }
      else if (UART_RXFIFO_FULL_INT_ST ==
                                 (uart_intr_status & UART_RXFIFO_FULL_INT_ST) )
      {
         fifo_len = (READ_PERI_REG(UART_STATUS(uart_no) )
                                          >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
         buf_idx = 0;

         while (buf_idx < fifo_len)
         {
            tmp = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
            xQueueSendFromISR( uart_data.tx_queue,
                               &tmp,
                               &xHigherPriorityTaskWoken );
            buf_idx++;
         }
         WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_FULL_INT_CLR);
      }
      else if (UART_RXFIFO_TOUT_INT_ST ==
                                 (uart_intr_status & UART_RXFIFO_TOUT_INT_ST) )
      {
         fifo_len = (READ_PERI_REG(UART_STATUS(uart_no))
                                          >> UART_RXFIFO_CNT_S)&UART_RXFIFO_CNT;
         buf_idx = 0;

         while (buf_idx < fifo_len)
         {
            tmp = READ_PERI_REG(UART_FIFO(uart_no)) & 0xFF;
            xQueueSendFromISR( uart_data.tx_queue,
                               &tmp,
                               &xHigherPriorityTaskWoken );
            buf_idx++;
         }
         WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_RXFIFO_TOUT_INT_CLR);
      }
      else if (UART_TXFIFO_EMPTY_INT_ST ==
                                 (uart_intr_status & UART_TXFIFO_EMPTY_INT_ST) )
      {
          WRITE_PERI_REG(UART_INT_CLR(uart_no), UART_TXFIFO_EMPTY_INT_CLR);
          CLEAR_PERI_REG_MASK(UART_INT_ENA(uart_no), UART_TXFIFO_EMPTY_INT_ENA);
      }
      else
      {
         //skip
      }
      uart_intr_status = READ_PERI_REG(UART_INT_ST(uart_no)) ;
   }
}

void uart_init( int uart_no,
                int baud,
                xQueueHandle tx_queue,
                xQueueHandle rx_queue )
{
   if( uart_data.initialized )
   {
      return;
   }

   uart_data.baud       = baud;
   uart_data.uart_no    = uart_no;
   uart_data.tx_queue   = tx_queue;
   uart_data.rx_queue   = rx_queue;

   UART_WaitTxFifoEmpty( uart_no );
   UART_ConfigTypeDef  uart_config;


   uart_config.baud_rate           = baud;
   uart_config.data_bits           = UART_WordLength_8b;
   uart_config.parity              = USART_Parity_None;
   uart_config.stop_bits           = USART_StopBits_1;
   uart_config.flow_ctrl           = USART_HardwareFlowControl_None;
   uart_config.UART_RxFlowThresh   = 120;
   uart_config.UART_InverseMask    = UART_None_Inverse;
   UART_ParamConfig( uart_no , &uart_config);

   UART_IntrConfTypeDef uart_intr;
   uart_intr.UART_IntrEnMask = UART_RXFIFO_TOUT_INT_ENA |
                               UART_FRM_ERR_INT_ENA     |
                               UART_RXFIFO_FULL_INT_ENA |
                               UART_TXFIFO_EMPTY_INT_ENA;

   uart_intr.UART_RX_FifoFullIntrThresh  = 10;
   uart_intr.UART_RX_TimeOutIntrThresh   = 2;
   uart_intr.UART_TX_FifoEmptyIntrThresh = 20;

   UART_IntrConfig( uart_no , &uart_intr);
   UART_SetPrintPort( uart_no );
   UART_intr_handler_register( uart_rx_handler, NULL);
   ETS_UART_INTR_ENABLE();

   xTaskCreate( uart_task, (signed char *)"UART", 256, NULL, 2, NULL);
   uart_data.initialized = 1;
}


