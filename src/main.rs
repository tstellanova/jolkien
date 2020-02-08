#![deny(warnings)]
//#![deny(unsafe_code)]
#![no_main]
#![no_std]

//extern crate panic_itm;

extern crate panic_semihosting;


use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};

//use stm32h7xx_hal as processor_hal;
use stm32f4xx_hal as processor_hal;

//use stm32h7xx_hal::pac as pac;




#[macro_use]
extern crate cortex_m_rt;

use cortex_m_rt::{entry, ExceptionFrame};

//use processor_hal::hal::digital::v2::OutputPin;
use processor_hal::hal::digital::v2::ToggleableOutputPin;
//use processor_hal::hal::digital::v2::InputPin;


use cmsis_rtos2;

#[allow(non_upper_case_globals)]
#[no_mangle]
//pub static SystemCoreClock: u32 = 48000000; // stm32h743 HSI 48 MHz
pub static SystemCoreClock: u32 = 16_000_000; //stm32f4xx_hal rcc::HSI
  ///25000000; //stm32f401cc board with 25 MHz xtal

#[cfg(debug_assertions)]
use cortex_m_log::{print, println};

use cortex_m_log::{d_print, d_println};
// FOR itm mode:
//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};
#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;

//use processor_hal::pwr::{Pwr, VoltageScale};

//use processor_hal::rcc::Ccdr;
use processor_hal::rcc::Clocks;

//use processor_hal::rcc::CoreClocks;
//use processor_hal::rcc::Rcc;

use processor_hal::gpio::GpioExt;
use processor_hal::rcc::RccExt;

use cmsis_rtos2::{osThreadId_t};
use core::ops::DerefMut;

use processor_hal::{prelude::*, stm32};


//TODO this kind of hardcoding is not ergonomic

type GpioTypeUserLed1 =  processor_hal::gpio::gpioc::PC13<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;

// stm32h743 dev board:
//type GpioTypeUserLed1 =  processor_hal::gpio::gpiob::PB0<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;
//type GpioTypeUserLed2 =  processor_hal::gpio::gpioe::PE1<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;
//type GpioTypeUserLed3 =  processor_hal::gpio::gpiob::PB14<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;
//type GpioTypeUserButt =  processor_hal::gpio::gpioc::PC13<processor_hal::gpio::Input<processor_hal::gpio::PullDown>>;


static APP_CCDR:  Mutex<RefCell< Option< Clocks >>> = Mutex::new(RefCell::new(None));
static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));
//static USER_LED_2:  Mutex<RefCell<Option<GpioTypeUserLed2>>> = Mutex::new(RefCell::new(None));
//static USER_LED_3:  Mutex<RefCell<Option<GpioTypeUserLed3>>> = Mutex::new(RefCell::new(None));
//static USER_BUTT:  Mutex<RefCell<Option<GpioTypeUserButt>>> =  Mutex::new(RefCell::new(None));


// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// we can override this in debug mode for handy debugging
#[exception]
fn DefaultHandler(_irqn: i16) {
  #[cfg(debug_assertions)]
  let mut log = semihosting::InterruptFree::<_>::stdout().unwrap();
  d_println!(log, "IRQn = {}", _irqn);

}


// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
  loop {

  }
}

// Toggle the user leds from their prior state
fn toggle_leds() {
  interrupt::free(|cs| {
    if let Some(ref mut led1) = USER_LED_1.borrow(cs).borrow_mut().deref_mut() {
      led1.toggle().unwrap();
    }
//    if let Some(ref mut led2) = USER_LED_2.borrow(cs).borrow_mut().deref_mut() {
//      led2.toggle().unwrap();
//    }
//    if let Some(ref mut led3) = USER_LED_3.borrow(cs).borrow_mut().deref_mut() {
//      led3.toggle().unwrap();
//    }
  });
}

#[no_mangle]
extern "C" fn start_default_task(_arg: *mut cty::c_void) {
  #[cfg(debug_assertions)]
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_println!(log, "Start default loop...");

  let core_peripherals = cortex_m::Peripherals::take().unwrap();
  let mut delay = interrupt::free(|cs| {
    processor_hal::delay::Delay::new(core_peripherals.SYST, *APP_CCDR.borrow(cs).borrow().as_ref().unwrap())


       // delay(APP_CCDR.borrow(cs).borrow().as_ref().unwrap())
  });

  loop {
    toggle_leds();

//    // look at user button and if it's NOT pressed, blink the user LEDs
//    let user_butt_pressed = interrupt::free(|cs| {
//      USER_BUTT.borrow(cs).borrow().as_ref().unwrap().is_high().unwrap_or(false)
//    });
//
//    if !user_butt_pressed {
//      toggle_leds();
//    }
//    else {
//      d_print!(log, ".");
//    }
    // note: this delay is not accurate in debug mode with semihosting activated
    delay.delay_ms(100_u32);
    //TODO figure out why cmsis_rtos2::rtos_os_delay never fires?
  }
}

// Setup peripherals such as GPIO
fn setup_peripherals()  {
  #[cfg(debug_assertions)]
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_print!(log, "setup_peripherals...");

  let dp = stm32::Peripherals::take().unwrap();

  let gpioc = dp.GPIOC.split();
  let mut user_led1 = gpioc.pc13.into_push_pull_output();

  // Set up the system clock at 16 MHz
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();


  //store shared peripherals
  interrupt::free(|cs| {
    APP_CCDR.borrow(cs).replace(Some(clocks));
    USER_LED_1.borrow(cs).replace(Some(user_led1));
  });

  d_println!(log, "done!");

}


fn setup_rtos() -> osThreadId_t {
  #[cfg(debug_assertions)]
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_println!(log, "Setup RTOS...");

  let _rc = cmsis_rtos2::rtos_kernel_initialize();
  d_println!(log, "kernel_initialize rc: {}", _rc);

  let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
  d_println!(log, "tick_hz : {}", _tick_hz);

  let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
  d_println!(log, "sys_timer_hz : {}", _sys_timer_hz);


//  let default_task_attributes  = cmsis_rtos2::osThreadAttr_t {
//    name: "defaultTask\0".as_ptr(),
//    attr_bits: 0,
//    cb_mem: core::ptr::null_mut(),
//    cb_size: 0,
//    stack_mem: core::ptr::null_mut(),
//    stack_size: 128,
//    priority:  cmsis_rtos2::osPriority_t_osPriorityNormal,
//    tz_module: 0,
//    reserved: 0,
//  };

  // We don't pass context to the default task here, since that involves problematic
  // casting to/from C void pointers; instead, we use global static context.
  let default_thread_id = cmsis_rtos2::rtos_os_thread_new(
    Some(start_default_task),
    core::ptr::null_mut(),
    core::ptr::null(),
//    &default_task_attributes
  );

  if default_thread_id.is_null() {
    d_println!(log, "rtos_os_thread_new failed!");
    return core::ptr::null_mut()
  }
  d_println!(log, "rtos_os_thread_new ok! ");

  let _rc = cmsis_rtos2::rtos_kernel_start();
  d_println!(log, "kernel_start rc: {}", _rc);

  d_println!(log,"RTOS done!");

  default_thread_id
}

#[entry]
fn main() -> ! {

  setup_peripherals();
  let _default_thread_id = setup_rtos();

  loop {
    cmsis_rtos2::rtos_os_thread_yield();
  }

}
