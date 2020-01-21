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

use cmsis_rtos2;

#[allow(non_upper_case_globals)]
#[no_mangle]
pub static SystemCoreClock: u32 = 480000000;

#[cfg(debug_assertions)]
use cortex_m_log::{print, println};

use cortex_m_log::{d_print, d_println};
// FOR itm mode:
//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};
#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;


#[no_mangle]
pub extern "C" fn start_default_task( _arg: *mut cty::c_void ) {
  loop {
    cmsis_rtos2::rtos_os_delay(1); //1 Hz heartbeat
  }
}

fn setup_rtos() {
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();

  d_println!(log, "Setup RTOS...");

  let rc = cmsis_rtos2::rtos_kernel_initialize();
  d_println!(log, "kernel_initialize rc: {}", rc);

  let default_task_attributes  = cmsis_rtos2::osThreadAttr_t {
    name: "defaultTask".as_ptr(),
    attr_bits: 0,
    cb_mem: core::ptr::null_mut(),
    cb_size: 0,
    stack_mem: core::ptr::null_mut(),
    stack_size: 128,
    priority:  cmsis_rtos2::osPriority_t_osPriorityNormal,
    tz_module: 0,
    reserved: 0,
  };

  let default_task_handle = cmsis_rtos2::rtos_os_thread_new(
    Some(start_default_task),
    core::ptr::null_mut(),
    &default_task_attributes);
  if default_task_handle.is_null() {
    d_println!(log, "rtos_os_thread_new failed!");
    return;
  }

  let rc = cmsis_rtos2::rtos_kernel_start();
  d_println!(log, "kernel_start rc: {}", rc);

  d_println!(log,"RTOS done!");
}

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

  setup_rtos();

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
