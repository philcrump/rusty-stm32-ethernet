
// Don't complain about variable naming convention
#![allow(non_snake_case)]

use defmt::*;
use embassy_stm32::usart::{Config, BufferedUart};
use embassy_stm32::peripherals::{USART2,PD6,PD5};
use embassy_time::{Timer};

use heapless::String;

use {defmt_rtt as _, panic_probe as _};


//USART2
//PD6-RX / PD5-TX


