# Lecture1-UART and LCD Practice


* Press User Button to switch two modes
* Mode 1
    1. LED Blink every 200ms.
* Mode 2
    1. Turn off LED and wait for input.
    2. PC Uses UART to transmit number to STM32.
    3. STM32 return "START" through UART and display on LCD.
    4. Blink LED and countdown number return number to UART and LCD.
    5. After finish return "END" and go back (1) state.
