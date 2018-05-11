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

#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_common.h"
#include "uart.h"

/** @brief Initializes the UART-handling
 *  @param uart_no Selects the UART: UART0 or UART1.
 *  @param paud The desired baud rate
 *  @param tx_queue See global tx_queue.
 *  @param rx_queue See global rx_queue.
 *  @returns Nothing.
 */
void uart_init( int uart_no,
                int baud,
                xQueueHandle tx_queue,
                xQueueHandle rx_queue );


#endif /* UART_HANDLER_H */

