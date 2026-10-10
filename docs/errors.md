19:00:57 **** Incremental Build of configuration Debug for project APPS_BSPD Firmware ****
make -j16 all 
arm-none-eabi-gcc "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.d" -MT"Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.o" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.o"
arm-none-eabi-gcc "../Core/Src/main.c" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/main.d" -MT"Core/Src/main.o" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "Core/Src/main.o"
arm-none-eabi-gcc "../Core/Src/stm32f4xx_hal_msp.c" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/stm32f4xx_hal_msp.d" -MT"Core/Src/stm32f4xx_hal_msp.o" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "Core/Src/stm32f4xx_hal_msp.o"
arm-none-eabi-gcc "../Core/Src/stm32f4xx_it.c" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"Core/Src/stm32f4xx_it.d" -MT"Core/Src/stm32f4xx_it.o" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "Core/Src/stm32f4xx_it.o"
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:288:32: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  288 | static void UART_EndTxTransfer(UART_HandleTypeDef *huart);
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Core/Src/stm32f4xx_it.c:60:8: error: unknown type name 'UART_HandleTypeDef'
   60 | extern UART_HandleTypeDef huart2;
      |        ^~~~~~~~~~~~~~~~~~
../Core/Src/stm32f4xx_hal_msp.c:239:23: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  239 | void HAL_UART_MspInit(UART_HandleTypeDef* huart)
      |                       ^~~~~~~~~~~~~~~~~~
      |                       DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:289:32: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  289 | static void UART_EndRxTransfer(UART_HandleTypeDef *huart);
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Core/Src/stm32f4xx_it.c: In function 'USART2_IRQHandler':
../Core/Src/stm32f4xx_it.c:219:3: warning: implicit declaration of function 'HAL_UART_IRQHandler'; did you mean 'HAL_DAC_IRQHandler'? [-Wimplicit-function-declaration]
  219 |   HAL_UART_IRQHandler(&huart2);
      |   ^~~~~~~~~~~~~~~~~~~
      |   HAL_DAC_IRQHandler
../Core/Src/stm32f4xx_hal_msp.c:257:25: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  257 | void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
      |                         ^~~~~~~~~~~~~~~~~~
      |                         DAC_HandleTypeDef
make: *** [Core/Src/subdir.mk:37: Core/Src/stm32f4xx_it.o] Error 1
make: *** Waiting for unfinished jobs....
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:300:43: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  300 | static HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart);
      |                                           ^~~~~~~~~~~~~~~~~~
      |                                           DAC_HandleTypeDef
make: *** [Core/Src/subdir.mk:37: Core/Src/stm32f4xx_hal_msp.o] Error 1
../Core/Src/main.c:85:1: error: unknown type name 'UART_HandleTypeDef'; did you mean 'USBD_HandleTypeDef'?
   85 | UART_HandleTypeDef huart2;
      | ^~~~~~~~~~~~~~~~~~
      | USBD_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:301:46: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  301 | static HAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart);
      |                                              ^~~~~~~~~~~~~~~~~~
      |                                              DAC_HandleTypeDef
../Core/Src/main.c: In function 'ConsoleSendString':
../Core/Src/main.c:258:11: warning: implicit declaration of function 'HAL_UART_Transmit'; did you mean 'HAL_PCD_EP_Transmit'? [-Wimplicit-function-declaration]
  258 |     (void)HAL_UART_Transmit(&huart2, (uint8_t*)str, (uint16_t)length, 100U);
      |           ^~~~~~~~~~~~~~~~~
      |           HAL_PCD_EP_Transmit
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:302:42: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  302 | static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart);
      |                                          ^~~~~~~~~~~~~~~~~~
      |                                          DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:303:54: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  303 | static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
      |                                                      ^~~~~~~~~~~~~~~~~~
      |                                                      DAC_HandleTypeDef
../Core/Src/main.c: In function 'main':
../Core/Src/main.c:408:9: warning: implicit declaration of function 'HAL_UART_Receive_IT' [-Wimplicit-function-declaration]
  408 |   (void)HAL_UART_Receive_IT(&huart2, &uartRxByte, 1U);
      |         ^~~~~~~~~~~~~~~~~~~
../Core/Src/main.c: In function 'MX_USART2_UART_Init':
../Core/Src/main.c:558:9: error: request for member 'Instance' in something not a structure or union
  558 |   huart2.Instance = USART2;
      |         ^
../Core/Src/main.c:559:9: error: request for member 'Init' in something not a structure or union
  559 |   huart2.Init.BaudRate = 115200;
      |         ^
../Core/Src/main.c:560:9: error: request for member 'Init' in something not a structure or union
  560 |   huart2.Init.WordLength = UART_WORDLENGTH_8B;
      |         ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:305:28: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  305 | static void UART_SetConfig(UART_HandleTypeDef *huart);
      |                            ^~~~~~~~~~~~~~~~~~
      |                            DAC_HandleTypeDef
../Core/Src/main.c:560:28: error: 'UART_WORDLENGTH_8B' undeclared (first use in this function)
  560 |   huart2.Init.WordLength = UART_WORDLENGTH_8B;
      |                            ^~~~~~~~~~~~~~~~~~
../Core/Src/main.c:560:28: note: each undeclared identifier is reported only once for each function it appears in
../Core/Src/main.c:561:9: error: request for member 'Init' in something not a structure or union
  561 |   huart2.Init.StopBits = UART_STOPBITS_1;
      |         ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:357:33: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  357 | HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart)
      |                                 ^~~~~~~~~~~~~~~~~~
      |                                 DAC_HandleTypeDef
../Core/Src/main.c:561:26: error: 'UART_STOPBITS_1' undeclared (first use in this function)
  561 |   huart2.Init.StopBits = UART_STOPBITS_1;
      |                          ^~~~~~~~~~~~~~~
../Core/Src/main.c:562:9: error: request for member 'Init' in something not a structure or union
  562 |   huart2.Init.Parity = UART_PARITY_NONE;
      |         ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:435:39: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  435 | HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart)
      |                                       ^~~~~~~~~~~~~~~~~~
      |                                       DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:509:32: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  509 | HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength)
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Core/Src/main.c:562:24: error: 'UART_PARITY_NONE' undeclared (first use in this function)
  562 |   huart2.Init.Parity = UART_PARITY_NONE;
      |                        ^~~~~~~~~~~~~~~~
../Core/Src/main.c:563:9: error: request for member 'Init' in something not a structure or union
  563 |   huart2.Init.Mode = UART_MODE_TX_RX;
      |         ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:591:43: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  591 | HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod)
      |                                           ^~~~~~~~~~~~~~~~~~
      |                                           DAC_HandleTypeDef
../Core/Src/main.c:563:22: error: 'UART_MODE_TX_RX' undeclared (first use in this function)
  563 |   huart2.Init.Mode = UART_MODE_TX_RX;
      |                      ^~~~~~~~~~~~~~~
../Core/Src/main.c:564:9: error: request for member 'Init' in something not a structure or union
  564 |   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      |         ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:669:35: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  669 | HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart)
      |                                   ^~~~~~~~~~~~~~~~~~
      |                                   DAC_HandleTypeDef
../Core/Src/main.c:564:27: error: 'UART_HWCONTROL_NONE' undeclared (first use in this function)
  564 |   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
      |                           ^~~~~~~~~~~~~~~~~~~
../Core/Src/main.c:565:9: error: request for member 'Init' in something not a structure or union
  565 |   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
      |         ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:715:30: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  715 | __weak void HAL_UART_MspInit(UART_HandleTypeDef *huart)
      |                              ^~~~~~~~~~~~~~~~~~
      |                              DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:730:32: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
  730 | __weak void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Core/Src/main.c:565:30: error: 'UART_OVERSAMPLING_16' undeclared (first use in this function)
  565 |   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
      |                              ^~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1135:37: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1135 | HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout)
      |                                     ^~~~~~~~~~~~~~~~~~
      |                                     DAC_HandleTypeDef
../Core/Src/main.c:566:7: warning: implicit declaration of function 'HAL_UART_Init'; did you mean 'HAL_DAC_Init'? [-Wimplicit-function-declaration]
  566 |   if (HAL_UART_Init(&huart2) != HAL_OK)
      |       ^~~~~~~~~~~~~
      |       HAL_DAC_Init
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1221:36: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1221 | HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout)
      |                                    ^~~~~~~~~~~~~~~~~~
      |                                    DAC_HandleTypeDef
../Core/Src/main.c: At top level:
../Core/Src/main.c:752:30: error: unknown type name 'UART_HandleTypeDef'; did you mean 'USBD_HandleTypeDef'?
  752 | void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
      |                              ^~~~~~~~~~~~~~~~~~
      |                              USBD_HandleTypeDef
make: *** [Core/Src/subdir.mk:37: Core/Src/main.o] Error 1
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1308:40: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1308 | HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
      |                                        ^~~~~~~~~~~~~~~~~~
      |                                        DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1347:39: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1347 | HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
      |                                       ^~~~~~~~~~~~~~~~~~
      |                                       DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1379:41: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1379 | HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size)
      |                                         ^~~~~~~~~~~~~~~~~~
      |                                         DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1449:40: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1449 | HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
      |                                        ^~~~~~~~~~~~~~~~~~
      |                                        DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1476:37: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1476 | HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart)
      |                                     ^~~~~~~~~~~~~~~~~~
      |                                     DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1507:38: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1507 | HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart)
      |                                      ^~~~~~~~~~~~~~~~~~
      |                                      DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1541:36: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1541 | HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart)
      |                                    ^~~~~~~~~~~~~~~~~~
      |                                    DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1596:44: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1596 | HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint16_t *RxLen,
      |                                            ^~~~~~~~~~~~~~~~~~
      |                                            DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1721:47: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1721 | HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
      |                                               ^~~~~~~~~~~~~~~~~~
      |                                               DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1781:48: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1781 | HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
      |                                                ^~~~~~~~~~~~~~~~~~
      |                                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1846:1: error: unknown type name 'HAL_UART_RxEventTypeTypeDef'
 1846 | HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *huart)
      | ^~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1846:55: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1846 | HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *huart)
      |                                                       ^~~~~~~~~~~~~~~~~~
      |                                                       DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1864:34: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1864 | HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart)
      |                                  ^~~~~~~~~~~~~~~~~~
      |                                  DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:1953:42: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 1953 | HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart)
      |                                          ^~~~~~~~~~~~~~~~~~
      |                                          DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2004:41: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2004 | HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart)
      |                                         ^~~~~~~~~~~~~~~~~~
      |                                         DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2065:37: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2065 | HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart)
      |                                     ^~~~~~~~~~~~~~~~~~
      |                                     DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2200:45: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2200 | HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart)
      |                                             ^~~~~~~~~~~~~~~~~~
      |                                             DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2277:44: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2277 | HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart)
      |                                            ^~~~~~~~~~~~~~~~~~
      |                                            DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2355:26: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2355 | void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)
      |                          ^~~~~~~~~~~~~~~~~~
      |                          DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2619:37: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2619 | __weak void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
      |                                     ^~~~~~~~~~~~~~~~~~
      |                                     DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2634:41: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2634 | __weak void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
      |                                         ^~~~~~~~~~~~~~~~~~
      |                                         DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2649:37: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2649 | __weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
      |                                     ^~~~~~~~~~~~~~~~~~
      |                                     DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2664:41: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2664 | __weak void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
      |                                         ^~~~~~~~~~~~~~~~~~
      |                                         DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2679:36: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2679 | __weak void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
      |                                    ^~~~~~~~~~~~~~~~~~
      |                                    DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2693:40: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2693 | __weak void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
      |                                        ^~~~~~~~~~~~~~~~~~
      |                                        DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2708:48: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2708 | __weak void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart)
      |                                                ^~~~~~~~~~~~~~~~~~
      |                                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2723:47: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2723 | __weak void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
      |                                               ^~~~~~~~~~~~~~~~~~
      |                                               DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2740:40: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2740 | __weak void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
      |                                        ^~~~~~~~~~~~~~~~~~
      |                                        DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2780:37: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2780 | HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart)
      |                                     ^~~~~~~~~~~~~~~~~~
      |                                     DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2807:52: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2807 | HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart)
      |                                                    ^~~~~~~~~~~~~~~~~~
      |                                                    DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2835:51: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2835 | HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart)
      |                                                   ^~~~~~~~~~~~~~~~~~
      |                                                   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2863:52: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2863 | HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart)
      |                                                    ^~~~~~~~~~~~~~~~~~
      |                                                    DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2898:49: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 2898 | HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart)
      |                                                 ^~~~~~~~~~~~~~~~~~
      |                                                 DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2955:1: error: unknown type name 'HAL_UART_StateTypeDef'; did you mean 'HAL_DAC_StateTypeDef'?
 2955 | HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart)
      | ^~~~~~~~~~~~~~~~~~~~~
      | HAL_DAC_StateTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2955:47: error: unknown type name 'UART_HandleTypeDef'
 2955 | HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart)
      |                                               ^~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'HAL_UART_GetState':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2958:16: error: request for member 'gState' in something not a structure or union
 2958 |   temp1 = huart->gState;
      |                ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2959:16: error: request for member 'RxState' in something not a structure or union
 2959 |   temp2 = huart->RxState;
      |                ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2961:11: error: 'HAL_UART_StateTypeDef' undeclared (first use in this function); did you mean 'HAL_DAC_StateTypeDef'?
 2961 |   return (HAL_UART_StateTypeDef)(temp1 | temp2);
      |           ^~~~~~~~~~~~~~~~~~~~~
      |           HAL_DAC_StateTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2961:11: note: each undeclared identifier is reported only once for each function it appears in
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: At top level:
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2970:34: error: unknown type name 'UART_HandleTypeDef'
 2970 | uint32_t HAL_UART_GetError(const UART_HandleTypeDef *huart)
      |                                  ^~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'HAL_UART_GetError':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2972:15: error: request for member 'ErrorCode' in something not a structure or union
 2972 |   return huart->ErrorCode;
      |               ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMATransmitCplt':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3017:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3017 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3017:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3017 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3017:52: error: expected expression before ')' token
 3017 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3021:10: error: request for member 'TxXferCount' in something not a structure or union
 3021 |     huart->TxXferCount = 0x00U;
      |          ^~
In file included from ../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h:29,
                 from ../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h:27,
                 from ../Core/Inc/stm32f4xx_hal_conf.h:275,
                 from ../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h:29,
                 from ../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:258:
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3025:27: error: request for member 'Instance' in something not a structure or union
 3025 |     ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:242:41: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  242 |       val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
      |                                         ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3025:27: error: request for member 'Instance' in something not a structure or union
 3025 |     ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:243:47: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  243 |     } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
      |                                               ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3028:25: error: request for member 'Instance' in something not a structure or union
 3028 |     ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
      |                         ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:233:41: note: in definition of macro 'ATOMIC_SET_BIT'
  233 |       val = __LDREXW((__IO uint32_t *)&(REG)) | (BIT);       \
      |                                         ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3028:25: error: request for member 'Instance' in something not a structure or union
 3028 |     ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_TCIE);
      |                         ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:234:47: note: in definition of macro 'ATOMIC_SET_BIT'
  234 |     } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
      |                                               ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3039:5: warning: implicit declaration of function 'HAL_UART_TxCpltCallback'; did you mean 'HAL_UART_WakeupCallback'? [-Wimplicit-function-declaration]
 3039 |     HAL_UART_TxCpltCallback(huart);
      |     ^~~~~~~~~~~~~~~~~~~~~~~
      |     HAL_UART_WakeupCallback
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMATxHalfCplt':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3052:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3052 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3052:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3052 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3052:52: error: expected expression before ')' token
 3052 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3059:3: warning: implicit declaration of function 'HAL_UART_TxHalfCpltCallback'; did you mean 'HAL_ADC_ConvHalfCpltCallback'? [-Wimplicit-function-declaration]
 3059 |   HAL_UART_TxHalfCpltCallback(huart);
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~
      |   HAL_ADC_ConvHalfCpltCallback
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMAReceiveCplt':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3071:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3071 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3071:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3071 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3071:52: error: expected expression before ')' token
 3071 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3076:10: error: request for member 'RxXferCount' in something not a structure or union
 3076 |     huart->RxXferCount = 0U;
      |          ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3079:27: error: request for member 'Instance' in something not a structure or union
 3079 |     ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:242:41: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  242 |       val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
      |                                         ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3079:27: error: request for member 'Instance' in something not a structure or union
 3079 |     ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:243:47: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  243 |     } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
      |                                               ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3080:27: error: request for member 'Instance' in something not a structure or union
 3080 |     ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:242:41: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  242 |       val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
      |                                         ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3080:27: error: request for member 'Instance' in something not a structure or union
 3080 |     ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:243:47: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  243 |     } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
      |                                               ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3084:27: error: request for member 'Instance' in something not a structure or union
 3084 |     ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:242:41: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  242 |       val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
      |                                         ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3084:27: error: request for member 'Instance' in something not a structure or union
 3084 |     ATOMIC_CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);
      |                           ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:243:47: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  243 |     } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
      |                                               ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3087:10: error: request for member 'RxState' in something not a structure or union
 3087 |     huart->RxState = HAL_UART_STATE_READY;
      |          ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3087:22: error: 'HAL_UART_STATE_READY' undeclared (first use in this function); did you mean 'HAL_DAC_STATE_READY'?
 3087 |     huart->RxState = HAL_UART_STATE_READY;
      |                      ^~~~~~~~~~~~~~~~~~~~
      |                      HAL_DAC_STATE_READY
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3090:14: error: request for member 'ReceptionType' in something not a structure or union
 3090 |     if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      |              ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3090:33: error: 'HAL_UART_RECEPTION_TOIDLE' undeclared (first use in this function)
 3090 |     if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      |                                 ^~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3092:29: error: request for member 'Instance' in something not a structure or union
 3092 |       ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
      |                             ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:242:41: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  242 |       val = __LDREXW((__IO uint32_t *)&(REG)) & ~(BIT);      \
      |                                         ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3092:29: error: request for member 'Instance' in something not a structure or union
 3092 |       ATOMIC_CLEAR_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
      |                             ^~
../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h:243:47: note: in definition of macro 'ATOMIC_CLEAR_BIT'
  243 |     } while ((__STREXW(val,(__IO uint32_t *)&(REG))) != 0U); \
      |                                               ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3098:8: error: request for member 'RxEventType' in something not a structure or union
 3098 |   huart->RxEventType = HAL_UART_RXEVENT_TC;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3098:24: error: 'HAL_UART_RXEVENT_TC' undeclared (first use in this function)
 3098 |   huart->RxEventType = HAL_UART_RXEVENT_TC;
      |                        ^~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3102:12: error: request for member 'ReceptionType' in something not a structure or union
 3102 |   if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      |            ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3109:5: warning: implicit declaration of function 'HAL_UARTEx_RxEventCallback'; did you mean 'HAL_UART_WakeupCallback'? [-Wimplicit-function-declaration]
 3109 |     HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
      |     ^~~~~~~~~~~~~~~~~~~~~~~~~~
      |     HAL_UART_WakeupCallback
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3109:44: error: request for member 'RxXferSize' in something not a structure or union
 3109 |     HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize);
      |                                            ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3120:5: warning: implicit declaration of function 'HAL_UART_RxCpltCallback'; did you mean 'HAL_UART_WakeupCallback'? [-Wimplicit-function-declaration]
 3120 |     HAL_UART_RxCpltCallback(huart);
      |     ^~~~~~~~~~~~~~~~~~~~~~~
      |     HAL_UART_WakeupCallback
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMARxHalfCplt':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3133:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3133 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3133:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3133 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3133:52: error: expected expression before ')' token
 3133 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3137:8: error: request for member 'RxEventType' in something not a structure or union
 3137 |   huart->RxEventType = HAL_UART_RXEVENT_HT;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3137:24: error: 'HAL_UART_RXEVENT_HT' undeclared (first use in this function)
 3137 |   huart->RxEventType = HAL_UART_RXEVENT_HT;
      |                        ^~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3141:12: error: request for member 'ReceptionType' in something not a structure or union
 3141 |   if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      |            ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3141:31: error: 'HAL_UART_RECEPTION_TOIDLE' undeclared (first use in this function)
 3141 |   if (huart->ReceptionType == HAL_UART_RECEPTION_TOIDLE)
      |                               ^~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3148:44: error: request for member 'RxXferSize' in something not a structure or union
 3148 |     HAL_UARTEx_RxEventCallback(huart, huart->RxXferSize / 2U);
      |                                            ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3159:5: warning: implicit declaration of function 'HAL_UART_RxHalfCpltCallback'; did you mean 'HAL_ADC_ConvHalfCpltCallback'? [-Wimplicit-function-declaration]
 3159 |     HAL_UART_RxHalfCpltCallback(huart);
      |     ^~~~~~~~~~~~~~~~~~~~~~~~~~~
      |     HAL_ADC_ConvHalfCpltCallback
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMAError':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3173:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3173 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3173:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3173 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3173:52: error: expected expression before ')' token
 3173 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3176:36: error: request for member 'Instance' in something not a structure or union
 3176 |   dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAT);
      |                                    ^~
../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h:63:45: note: in definition of macro 'HAL_IS_BIT_SET'
   63 | #define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
      |                                             ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3177:13: error: request for member 'gState' in something not a structure or union
 3177 |   if ((huart->gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
      |             ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3177:25: error: 'HAL_UART_STATE_BUSY_TX' undeclared (first use in this function); did you mean 'HAL_DAC_STATE_BUSY'?
 3177 |   if ((huart->gState == HAL_UART_STATE_BUSY_TX) && dmarequest)
      |                         ^~~~~~~~~~~~~~~~~~~~~~
      |                         HAL_DAC_STATE_BUSY
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3179:10: error: request for member 'TxXferCount' in something not a structure or union
 3179 |     huart->TxXferCount = 0x00U;
      |          ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3180:5: warning: implicit declaration of function 'UART_EndTxTransfer' [-Wimplicit-function-declaration]
 3180 |     UART_EndTxTransfer(huart);
      |     ^~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3184:36: error: request for member 'Instance' in something not a structure or union
 3184 |   dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
      |                                    ^~
../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h:63:45: note: in definition of macro 'HAL_IS_BIT_SET'
   63 | #define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) == (BIT))
      |                                             ^~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3185:13: error: request for member 'RxState' in something not a structure or union
 3185 |   if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
      |             ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3185:26: error: 'HAL_UART_STATE_BUSY_RX' undeclared (first use in this function); did you mean 'HAL_ADC_STATE_BUSY_REG'?
 3185 |   if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
      |                          ^~~~~~~~~~~~~~~~~~~~~~
      |                          HAL_ADC_STATE_BUSY_REG
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3187:10: error: request for member 'RxXferCount' in something not a structure or union
 3187 |     huart->RxXferCount = 0x00U;
      |          ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3188:5: warning: implicit declaration of function 'UART_EndRxTransfer' [-Wimplicit-function-declaration]
 3188 |     UART_EndRxTransfer(huart);
      |     ^~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3191:8: error: request for member 'ErrorCode' in something not a structure or union
 3191 |   huart->ErrorCode |= HAL_UART_ERROR_DMA;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3191:23: error: 'HAL_UART_ERROR_DMA' undeclared (first use in this function); did you mean 'HAL_ADC_ERROR_DMA'?
 3191 |   huart->ErrorCode |= HAL_UART_ERROR_DMA;
      |                       ^~~~~~~~~~~~~~~~~~
      |                       HAL_ADC_ERROR_DMA
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3197:3: warning: implicit declaration of function 'HAL_UART_ErrorCallback'; did you mean 'HAL_ADC_ErrorCallback'? [-Wimplicit-function-declaration]
 3197 |   HAL_UART_ErrorCallback(huart);
      |   ^~~~~~~~~~~~~~~~~~~~~~
      |   HAL_ADC_ErrorCallback
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: At top level:
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3212:54: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3212 | static HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status,
      |                                                      ^~~~~~~~~~~~~~~~~~
      |                                                      DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3263:41: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3263 | HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
      |                                         ^~~~~~~~~~~~~~~~~~
      |                                         DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3298:42: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3298 | HAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
      |                                          ^~~~~~~~~~~~~~~~~~
      |                                          DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3356:32: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3356 | static void UART_EndTxTransfer(UART_HandleTypeDef *huart)
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3370:32: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3370 | static void UART_EndRxTransfer(UART_HandleTypeDef *huart)
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMAAbortOnError':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3396:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3396 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3396:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3396 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3396:52: error: expected expression before ')' token
 3396 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3397:8: error: request for member 'RxXferCount' in something not a structure or union
 3397 |   huart->RxXferCount = 0x00U;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMATxAbortCallback':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3419:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3419 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3419:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3419 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3419:52: error: expected expression before ')' token
 3419 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3421:8: error: request for member 'hdmatx' in something not a structure or union
 3421 |   huart->hdmatx->XferAbortCallback = NULL;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3424:12: error: request for member 'hdmarx' in something not a structure or union
 3424 |   if (huart->hdmarx != NULL)
      |            ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3426:14: error: request for member 'hdmarx' in something not a structure or union
 3426 |     if (huart->hdmarx->XferAbortCallback != NULL)
      |              ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3433:8: error: request for member 'TxXferCount' in something not a structure or union
 3433 |   huart->TxXferCount = 0x00U;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3434:8: error: request for member 'RxXferCount' in something not a structure or union
 3434 |   huart->RxXferCount = 0x00U;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3437:8: error: request for member 'ErrorCode' in something not a structure or union
 3437 |   huart->ErrorCode = HAL_UART_ERROR_NONE;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3437:22: error: 'HAL_UART_ERROR_NONE' undeclared (first use in this function); did you mean 'HAL_DAC_ERROR_NONE'?
 3437 |   huart->ErrorCode = HAL_UART_ERROR_NONE;
      |                      ^~~~~~~~~~~~~~~~~~~
      |                      HAL_DAC_ERROR_NONE
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3440:8: error: request for member 'gState' in something not a structure or union
 3440 |   huart->gState  = HAL_UART_STATE_READY;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3440:20: error: 'HAL_UART_STATE_READY' undeclared (first use in this function); did you mean 'HAL_DAC_STATE_READY'?
 3440 |   huart->gState  = HAL_UART_STATE_READY;
      |                    ^~~~~~~~~~~~~~~~~~~~
      |                    HAL_DAC_STATE_READY
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3441:8: error: request for member 'RxState' in something not a structure or union
 3441 |   huart->RxState = HAL_UART_STATE_READY;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3442:8: error: request for member 'ReceptionType' in something not a structure or union
 3442 |   huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3442:26: error: 'HAL_UART_RECEPTION_STANDARD' undeclared (first use in this function)
 3442 |   huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
      |                          ^~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3450:3: warning: implicit declaration of function 'HAL_UART_AbortCpltCallback'; did you mean 'HAL_ADC_ConvCpltCallback'? [-Wimplicit-function-declaration]
 3450 |   HAL_UART_AbortCpltCallback(huart);
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~
      |   HAL_ADC_ConvCpltCallback
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMARxAbortCallback':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3465:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3465 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3465:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3465 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3465:52: error: expected expression before ')' token
 3465 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3467:8: error: request for member 'hdmarx' in something not a structure or union
 3467 |   huart->hdmarx->XferAbortCallback = NULL;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3470:12: error: request for member 'hdmatx' in something not a structure or union
 3470 |   if (huart->hdmatx != NULL)
      |            ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3472:14: error: request for member 'hdmatx' in something not a structure or union
 3472 |     if (huart->hdmatx->XferAbortCallback != NULL)
      |              ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3479:8: error: request for member 'TxXferCount' in something not a structure or union
 3479 |   huart->TxXferCount = 0x00U;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3480:8: error: request for member 'RxXferCount' in something not a structure or union
 3480 |   huart->RxXferCount = 0x00U;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3483:8: error: request for member 'ErrorCode' in something not a structure or union
 3483 |   huart->ErrorCode = HAL_UART_ERROR_NONE;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3483:22: error: 'HAL_UART_ERROR_NONE' undeclared (first use in this function); did you mean 'HAL_DAC_ERROR_NONE'?
 3483 |   huart->ErrorCode = HAL_UART_ERROR_NONE;
      |                      ^~~~~~~~~~~~~~~~~~~
      |                      HAL_DAC_ERROR_NONE
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3486:8: error: request for member 'gState' in something not a structure or union
 3486 |   huart->gState  = HAL_UART_STATE_READY;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3486:20: error: 'HAL_UART_STATE_READY' undeclared (first use in this function); did you mean 'HAL_DAC_STATE_READY'?
 3486 |   huart->gState  = HAL_UART_STATE_READY;
      |                    ^~~~~~~~~~~~~~~~~~~~
      |                    HAL_DAC_STATE_READY
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3487:8: error: request for member 'RxState' in something not a structure or union
 3487 |   huart->RxState = HAL_UART_STATE_READY;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3488:8: error: request for member 'ReceptionType' in something not a structure or union
 3488 |   huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3488:26: error: 'HAL_UART_RECEPTION_STANDARD' undeclared (first use in this function)
 3488 |   huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
      |                          ^~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMATxOnlyAbortCallback':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3511:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3511 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3511:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3511 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3511:52: error: expected expression before ')' token
 3511 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3513:8: error: request for member 'TxXferCount' in something not a structure or union
 3513 |   huart->TxXferCount = 0x00U;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3516:8: error: request for member 'gState' in something not a structure or union
 3516 |   huart->gState = HAL_UART_STATE_READY;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3516:19: error: 'HAL_UART_STATE_READY' undeclared (first use in this function); did you mean 'HAL_DAC_STATE_READY'?
 3516 |   huart->gState = HAL_UART_STATE_READY;
      |                   ^~~~~~~~~~~~~~~~~~~~
      |                   HAL_DAC_STATE_READY
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3524:3: warning: implicit declaration of function 'HAL_UART_AbortTransmitCpltCallback' [-Wimplicit-function-declaration]
 3524 |   HAL_UART_AbortTransmitCpltCallback(huart);
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'UART_DMARxOnlyAbortCallback':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3539:3: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3539 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |   ^~~~~~~~~~~~~~~~~~
      |   DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3539:32: error: 'UART_HandleTypeDef' undeclared (first use in this function); did you mean 'DAC_HandleTypeDef'?
 3539 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                ^~~~~~~~~~~~~~~~~~
      |                                DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3539:52: error: expected expression before ')' token
 3539 |   UART_HandleTypeDef *huart = (UART_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
      |                                                    ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3541:8: error: request for member 'RxXferCount' in something not a structure or union
 3541 |   huart->RxXferCount = 0x00U;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3544:8: error: request for member 'RxState' in something not a structure or union
 3544 |   huart->RxState = HAL_UART_STATE_READY;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3544:20: error: 'HAL_UART_STATE_READY' undeclared (first use in this function); did you mean 'HAL_DAC_STATE_READY'?
 3544 |   huart->RxState = HAL_UART_STATE_READY;
      |                    ^~~~~~~~~~~~~~~~~~~~
      |                    HAL_DAC_STATE_READY
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3545:8: error: request for member 'ReceptionType' in something not a structure or union
 3545 |   huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
      |        ^~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3545:26: error: 'HAL_UART_RECEPTION_STANDARD' undeclared (first use in this function)
 3545 |   huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
      |                          ^~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3553:3: warning: implicit declaration of function 'HAL_UART_AbortReceiveCpltCallback' [-Wimplicit-function-declaration]
 3553 |   HAL_UART_AbortReceiveCpltCallback(huart);
      |   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: At top level:
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3563:43: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3563 | static HAL_StatusTypeDef UART_Transmit_IT(UART_HandleTypeDef *huart)
      |                                           ^~~~~~~~~~~~~~~~~~
      |                                           DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3603:46: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3603 | static HAL_StatusTypeDef UART_EndTransmit_IT(UART_HandleTypeDef *huart)
      |                                              ^~~~~~~~~~~~~~~~~~
      |                                              DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3628:42: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3628 | static HAL_StatusTypeDef UART_Receive_IT(UART_HandleTypeDef *huart)
      |                                          ^~~~~~~~~~~~~~~~~~
      |                                          DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3731:28: error: unknown type name 'UART_HandleTypeDef'; did you mean 'DAC_HandleTypeDef'?
 3731 | static void UART_SetConfig(UART_HandleTypeDef *huart)
      |                            ^~~~~~~~~~~~~~~~~~
      |                            DAC_HandleTypeDef
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'HAL_UART_GetState':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2962:1: warning: control reaches end of non-void function [-Wreturn-type]
 2962 | }
      | ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: In function 'HAL_UART_GetError':
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:2973:1: warning: control reaches end of non-void function [-Wreturn-type]
 2973 | }
      | ^
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c: At top level:
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3537:13: warning: 'UART_DMARxOnlyAbortCallback' defined but not used [-Wunused-function]
 3537 | static void UART_DMARxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3509:13: warning: 'UART_DMATxOnlyAbortCallback' defined but not used [-Wunused-function]
 3509 | static void UART_DMATxOnlyAbortCallback(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3463:13: warning: 'UART_DMARxAbortCallback' defined but not used [-Wunused-function]
 3463 | static void UART_DMARxAbortCallback(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3417:13: warning: 'UART_DMATxAbortCallback' defined but not used [-Wunused-function]
 3417 | static void UART_DMATxAbortCallback(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3394:13: warning: 'UART_DMAAbortOnError' defined but not used [-Wunused-function]
 3394 | static void UART_DMAAbortOnError(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3170:13: warning: 'UART_DMAError' defined but not used [-Wunused-function]
 3170 | static void UART_DMAError(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3131:13: warning: 'UART_DMARxHalfCplt' defined but not used [-Wunused-function]
 3131 | static void UART_DMARxHalfCplt(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3069:13: warning: 'UART_DMAReceiveCplt' defined but not used [-Wunused-function]
 3069 | static void UART_DMAReceiveCplt(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3050:13: warning: 'UART_DMATxHalfCplt' defined but not used [-Wunused-function]
 3050 | static void UART_DMATxHalfCplt(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~
../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c:3015:13: warning: 'UART_DMATransmitCplt' defined but not used [-Wunused-function]
 3015 | static void UART_DMATransmitCplt(DMA_HandleTypeDef *hdma)
      |             ^~~~~~~~~~~~~~~~~~~~
make: *** [Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk:82: Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.o] Error 1
"make -j16 all" terminated with exit code 2. Build might be incomplete.

19:01:00 Build Failed. 189 errors, 27 warnings. (took 2s.555ms)

