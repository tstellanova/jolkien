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
pub static SystemCoreClock: u32 = 16_000_000; //or use stm32f4xx_hal rcc::HSI
//Can use 25_000_000 on an stm32f401 board with 25 MHz xtal
// 48_000_000 for stm32h743 HSI (48 MHz)

#[cfg(debug_assertions)]
use cortex_m_log::{print, println};

use cortex_m_log::{d_print, d_println};
// FOR itm mode:
//use cortex_m_log::{
//  destination::Itm, printer::itm::InterruptSync as InterruptSyncItm,
//};
#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;

#[cfg(debug_assertions)]
use cortex_m_semihosting;


//use processor_hal::pwr::{Pwr, VoltageScale};

//use processor_hal::rcc::Ccdr;
use processor_hal::rcc::Clocks;

//use processor_hal::rcc::CoreClocks;
//use processor_hal::rcc::Rcc;

use processor_hal::gpio::GpioExt;
use processor_hal::rcc::RccExt;

//use cmsis_rtos2::{osThreadId_t};
use core::ops::{DerefMut};

use processor_hal::{prelude::*, stm32};
//use core::ptr::null;

#[cfg(debug_assertions)]
type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;

//TODO this kind of hardcoding is not ergonomic

type GpioTypeUserLed1 =  processor_hal::gpio::gpioc::PC13<processor_hal::gpio::Output<processor_hal::gpio::PushPull>>;

static APP_CLOCKS:  Mutex<RefCell< Option< Clocks >>> = Mutex::new(RefCell::new(None));
static USER_LED_1:  Mutex<RefCell<Option< GpioTypeUserLed1>>> = Mutex::new(RefCell::new(None));

//static GLOBAL_QUEUE_HANDLE: Mutex<RefCell<Option< cmsis_rtos2::osMessageQueueId_t  >>> = Mutex::new(RefCell::new(None));

// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// we can override this in debug mode for handy debugging
#[exception]
fn DefaultHandler(_irqn: i16) {
  d_println!(get_debug_log(), "IRQn = {}", _irqn);

}


// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
  loop {

  }
}


/// Used in debug builds to provide a logging outlet
#[cfg(debug_assertions)]
fn get_debug_log() -> DebugLog {
  semihosting::InterruptFree::<_>::stdout().unwrap()
}

// Toggle the user leds from their prior state
fn toggle_leds() {
  interrupt::free(|cs| {
    if let Some(ref mut led1) = USER_LED_1.borrow(cs).borrow_mut().deref_mut() {
      led1.toggle().unwrap();
    }
  });
}

/// RTOS calls this function to run the default task
#[no_mangle]
extern "C" fn task1_cb(_arg: *mut cty::c_void) {
  //d_println!(get_debug_log(), "task1");

  loop {
    cmsis_rtos2::rtos_os_delay(100);
  }
  
}

#[no_mangle]
extern "C" fn task2_cb(_arg: *mut cty::c_void) {
  //d_println!(get_debug_log(), "task2");

  loop {
    cmsis_rtos2::rtos_os_delay(250);
  }

}

#[no_mangle]
extern "C" fn repeating_timer_task(_arg: *mut cty::c_void) {
  toggle_leds();
  //d_print!(get_debug_log(), ".");
}


fn setup_repeating_timer() {
  let tid = cmsis_rtos2::rtos_os_timer_new(
    Some(repeating_timer_task),
    cmsis_rtos2::osTimerType_t_osTimerPeriodic,
    core::ptr::null_mut(),
    core::ptr::null(),
  );

  if tid.is_null() {
    d_println!(get_debug_log(), "setup_repeating_timer failed...");
  }
  else {
    let rc = cmsis_rtos2::rtos_os_timer_start(tid, 50);
    if 0 != rc {
      d_println!(get_debug_log(), "rtos_os_timer_start failed {:?}", rc);
    }
    else {
      d_println!(get_debug_log(),"valid timer: {:?}", tid);
    }
  }

}

pub fn setup_default_threads() {
//  let default_task_attributes  = cmsis_rtos2::osThreadAttr_t {
//    name: "defaultTask/0".as_ptr(),
//    attr_bits: 0,
//    cb_mem: core::ptr::null_mut(),
//    cb_size: 0,
//    stack_mem: core::ptr::null_mut(),
//    stack_size: 128,
//    priority:  cmsis_rtos2::osPriority_t_osPriorityIdle,
//    tz_module: 0,
//    reserved: 0,
//  };

//  //create a shared msg queue
//  let mq_id = cmsis_rtos2::rtos_os_msg_queue_new(512, 1, null());
//  if mq_id.is_null() {
//    d_println!(get_debug_log(), "rtos_os_msg_queue_new failed");
//    return;
//  }
//  else {
//    d_println!(get_debug_log(), "new queue: {:?}", mq_id);
//  }

//  interrupt::free(|cs| {
//    GLOBAL_QUEUE_HANDLE.borrow(cs).replace(Some(mq_id));
//  });

  // We don't pass context to the default task here, since that involves problematic
  // casting to/from C void pointers; instead, we use global static context.
  let thread1_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task1_cb),
    core::ptr::null_mut(),
    core::ptr::null(),
  );
  if thread1_id.is_null() {
    d_println!(get_debug_log(), "rtos_os_thread_new failed!");
    return;
  }

  let thread2_id = cmsis_rtos2::rtos_os_thread_new(
    Some(task2_cb),
    core::ptr::null_mut(),
    core::ptr::null(),
  );
  if thread2_id.is_null() {
    d_println!(get_debug_log(), "rtos_os_thread_new failed!");
    return;
  }
}

// Setup peripherals such as GPIO
fn setup_peripherals()  {
  d_print!(get_debug_log(), "setup_peripherals...");

  let dp = stm32::Peripherals::take().unwrap();

  let gpioc = dp.GPIOC.split();
  let mut user_led1 = gpioc.pc13.into_push_pull_output();

  // Set up the system clock at 16 MHz
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.freeze();
//  let clocks = rcc.cfgr.sysclk(16.mhz()).freeze();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();


  //store shared peripherals
  interrupt::free(|cs| {
    APP_CLOCKS.borrow(cs).replace(Some(clocks));
    USER_LED_1.borrow(cs).replace(Some(user_led1));
  });

  d_println!(get_debug_log(), "done!");

}


fn setup_rtos() {
//  d_println!(get_debug_log(), "Setup RTOS...");

  let _rc = cmsis_rtos2::rtos_kernel_initialize();
//  d_println!(get_debug_log(), "kernel_initialize rc: {}", _rc);

  let _tick_hz = cmsis_rtos2::rtos_kernel_get_tick_freq_hz();
//  d_println!(get_debug_log(), "tick_hz : {}", _tick_hz);

  let _sys_timer_hz = cmsis_rtos2::rtos_kernel_get_sys_timer_freq_hz();
//  d_println!(get_debug_log(), "sys_timer_hz : {}", _sys_timer_hz);


  setup_repeating_timer();
  setup_default_threads();

  let _rc = cmsis_rtos2::rtos_kernel_start();
  d_println!(get_debug_log(), "kernel_start rc: {}", _rc);

  //d_println!(get_debug_log(),"RTOS done!");

}

#[entry]
fn main() -> ! {

  setup_peripherals();
  setup_rtos();

  loop {
    //cmsis_rtos2::rtos_os_thread_yield();
    //one hz heartbeat
    cmsis_rtos2::rtos_os_delay(1000);
    d_print!(get_debug_log(),".");
  }

}
