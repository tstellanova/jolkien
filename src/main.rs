#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;

use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;
use stm32h7xx_hal::hal::digital::v2::InputPin;

use stm32h7xx_hal::{pac, prelude::*};


#[cfg(debug_assertions)]
use cortex_m_log::{print, println};

use cortex_m_log::{d_print, d_println};

//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};
#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;

#[entry]
fn main() -> ! {
  let cp = cortex_m::Peripherals::take().unwrap();
  let dp = pac::Peripherals::take().unwrap();
#[cfg(debug_assertions)]
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
//  let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

  // Constrain and Freeze power
  d_print!(log, "Setup PWR...");
  let pwr = dp.PWR.constrain();
  let vos = pwr.freeze();
  d_println!(log,"done!");

  // Constrain and Freeze clock
  d_print!(log, "Setup RCC...");
  let rcc = dp.RCC.constrain();
  //use the existing sysclk
  let mut ccdr = rcc.freeze(vos, &dp.SYSCFG);
  d_println!(log,"done!");


  d_print!(log, "Setup GPIO...");

  //access the required GPIO registers
  let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);
  let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);

  // Configure LED2 (PE1)
  let mut led2 = gpioe.pe1.into_push_pull_output();

  // Configure LED1 (PB0)
  let mut led1 = gpiob.pb0.into_push_pull_output();
  // Configure LED3 (PB14)
  let mut led3 = gpiob.pb14.into_push_pull_output();

  //set initial states of user LEDs
  led1.set_high().unwrap();
  led2.set_low().unwrap();
  led3.set_high().unwrap();


  //setup user pushbutton on PC13
  let gpioc = dp.GPIOC.split(&mut ccdr.ahb4);
  let user_butt = gpioc.pc13.into_pull_down_input();
  d_println!(log, "done!");

  // Get the delay provider.
  let mut delay = cp.SYST.delay(ccdr.clocks);
  loop {
    //note that when running in debug mode, these delays are plain wrong
    delay.delay_ms(100_u32);
    if !user_butt.is_high().unwrap_or(false) {
      led1.toggle().unwrap();
      led2.toggle().unwrap();
      led3.toggle().unwrap();
    }
    else {
      d_print!(log, ".");
    }
  }
}
