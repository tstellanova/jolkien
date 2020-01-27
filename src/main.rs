#![deny(warnings)]
//#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;



use core::cell::RefCell;
use cortex_m::interrupt::{self, Mutex};

#[macro_use]
extern crate cortex_m_rt;

use cortex_m_rt::{entry, ExceptionFrame};

use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;
use stm32h7xx_hal::hal::digital::v2::InputPin;

use stm32h7xx_hal::{pac, prelude::*};

use cmsis_rtos2;

#[allow(non_upper_case_globals)]
#[no_mangle]
//pub static SystemCoreClock: u32 = 480000000;
pub static SystemCoreClock: u32 = 400000000;

#[cfg(debug_assertions)]
use cortex_m_log::{print, println};

use cortex_m_log::{d_print, d_println};
// FOR itm mode:
//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};
#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;

//use stm32h7xx_hal::pwr::{Pwr, VoltageScale};
use stm32h7xx_hal::rcc::Ccdr;
//use stm32h7xx_hal::rcc::CoreClocks;
//use stm32h7xx_hal::rcc::Rcc;

use stm32h7xx_hal::gpio::GpioExt;
use cmsis_rtos2::{osThreadId_t};
use core::ops::DerefMut;


//TODO this kind of hardcoding is not ergonomic
type GpioTypeUserLed1 =  stm32h7xx_hal::gpio::gpiob::PB0<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;
type GpioTypeUserLed2 =  stm32h7xx_hal::gpio::gpioe::PE1<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;
type GpioTypeUserLed3 =  stm32h7xx_hal::gpio::gpiob::PB14<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>;
type GpioTypeUserButt =  stm32h7xx_hal::gpio::gpioc::PC13<stm32h7xx_hal::gpio::Input<stm32h7xx_hal::gpio::PullDown>>;


static APP_CCDR:  Mutex<RefCell< Option< Ccdr >>> = Mutex::new(RefCell::new(None));
static USER_LED_1:  Mutex<RefCell< Option< GpioTypeUserLed1 >>> = Mutex::new(RefCell::new(None));
static USER_LED_2:  Mutex<RefCell<Option<GpioTypeUserLed2>>> = Mutex::new(RefCell::new(None));
static USER_LED_3:  Mutex<RefCell<Option<GpioTypeUserLed3>>> = Mutex::new(RefCell::new(None));
static USER_BUTT:  Mutex<RefCell<Option<GpioTypeUserButt>>> =  Mutex::new(RefCell::new(None));


// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// we can override this in debug mode for handy debugging
#[exception]
fn DefaultHandler(irqn: i16) {
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_println!(log, "IRQn = {}", irqn);
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
    if let Some(ref mut led2) = USER_LED_2.borrow(cs).borrow_mut().deref_mut() {
      led2.toggle().unwrap();
    }
    if let Some(ref mut led3) = USER_LED_3.borrow(cs).borrow_mut().deref_mut() {
      led3.toggle().unwrap();
    }
  });
}

#[no_mangle]
extern "C" fn start_default_task(_arg: *mut cty::c_void) {
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_println!(log, "Start default loop...");

  let core_peripherals = cortex_m::Peripherals::take().unwrap();
  let mut delay = interrupt::free(|cs| {
    core_peripherals.SYST.delay(APP_CCDR.borrow(cs).borrow().as_ref().unwrap().clocks)
  });

  loop {
    // look at user button and if it's NOT pressed, blink the user LEDs
    let user_butt_pressed = interrupt::free(|cs| {
      USER_BUTT.borrow(cs).borrow().as_ref().unwrap().is_high().unwrap_or(false)
    });

    if !user_butt_pressed {
      toggle_leds();
    }
    else {
      d_print!(log, ".");
    }
    delay.delay_ms(10_u32);
    //TODO try using CMSIS rtos_os_delay instead??
    //cmsis_rtos2::rtos_os_delay(3); //  heartbeat
  }
}

// Setup peripherals such as GPIO
fn setup_peripherals()  {
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_print!(log, "setup_peripherals...");
  //let core_peripherals = cortex_m::Peripherals::take().unwrap();

  let device_peripherals = pac::Peripherals::take().unwrap();

  let pwr = device_peripherals.PWR.constrain();
  let vos = pwr.freeze();

  // Constrain and Freeze clock
  let rcc = device_peripherals.RCC.constrain();
  //use the existing sysclk
  let mut ccdr = rcc.freeze(vos, &device_peripherals.SYSCFG);
  let gpiob = device_peripherals.GPIOB.split(&mut ccdr.ahb4);
  let gpioc = device_peripherals.GPIOC.split(&mut ccdr.ahb4);
  let gpioe = device_peripherals.GPIOE.split(&mut ccdr.ahb4);

  let mut user_led1 = gpiob.pb0.into_push_pull_output();
  let mut user_led2 = gpioe.pe1.into_push_pull_output();
  let mut user_led3 = gpiob.pb14.into_push_pull_output();
  let user_butt = gpioc.pc13.into_pull_down_input();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();
  user_led2.set_low().unwrap();
  user_led3.set_high().unwrap();


  //store shared peripherals
  interrupt::free(|cs| {
    APP_CCDR.borrow(cs).replace(Some(ccdr));
    USER_LED_1.borrow(cs).replace(Some(user_led1));
    USER_LED_2.borrow(cs).replace(Some(user_led2));
    USER_LED_3.borrow(cs).replace(Some(user_led3));
    USER_BUTT.borrow(cs).replace(Some(user_butt));
  });

  d_println!(log, "done!");

}


fn setup_rtos() -> osThreadId_t {
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_println!(log, "Setup RTOS...");

  let rc = cmsis_rtos2::rtos_kernel_initialize();
  d_println!(log, "kernel_initialize rc: {}", rc);

  let tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
  d_println!(log, "tick_hz : {}", tick_hz);

  let sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
  d_println!(log, "sys_timer_hz : {}", sys_timer_hz);


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

  let rc = cmsis_rtos2::rtos_kernel_start();
  d_println!(log, "kernel_start rc: {}", rc);

  d_println!(log,"RTOS done!");

  default_thread_id
}

#[entry]
fn main() -> ! {

  setup_peripherals();
  let _default_thread_id = setup_rtos();

  loop {
    cmsis_rtos2::rtos_os_thread_yield();

    //This is not provided by the ordinary FreeRTOS port:
    //cmsis_rtos2::rtos_os_thread_join(default_thread_id);
  }

}
