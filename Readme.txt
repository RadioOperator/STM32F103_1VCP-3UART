
Monitoring other UARTs Rx/Tx lines, use 1 terminal window only.


Hardware: STM32F103 Bluepill

All 3 UART Rx, go to 1 USB-VCP.

USB-VCP Tx, goes to Uart-1 PA9.


====================================
Uart-1:  Rx-PA10   Tx-PA9
Uart-2:  Rx-PA3    -
Uart-3:  Rx-PB11   -

short PB7-PB8, print in Hex mode, or
short PB8-PB9, print in ASCII mode

Uart speed = USB-VCP speed.
====================================



USB-VCP Rx Terminal Sample

ASCII mode:
Uart-1: 1111 1111 1111 1111
Uart-2: 2222 2222 2222 2222
Uart-3: 3333 3333 3333 3333

HEX mode:
Uart-1: 31 31 31 31 20 31 31 31 31 20 31 31 31 31 20 31 31 31 31 
Uart-2: 32 32 32 32 20 32 32 32 32 20 32 32 32 32 20 32 32 32 32 
Uart-3: 33 33 33 33 20 33 33 33 33 20 33 33 33 33 20 33 33 33 33 
