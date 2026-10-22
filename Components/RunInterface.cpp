/*
 *  RunInterface.cpp
 *
 *  Created on: Apr 3, 2023
 *      Author: Chris (cjchanx)
 */

#include "main_system.hpp"
#include "UARTDriver.hpp"

extern "C" {
    void run_interface()
    {
        run_main();
    }

    void cpp_USART1_IRQHandler()
    {
        Driver::usart1.HandleIRQ_UART();
    }
    void cpp_UART7_IRQHandler(){
    	Driver::uart7.HandleIRQ_UART();
    }
}



