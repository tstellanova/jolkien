#![deny(warnings)]
//#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate panic_itm;



#[macro_use]
extern crate cortex_m_rt;

use cortex_m_rt::{entry, ExceptionFrame};

//use cortex_m::peripheral::syst::SystClkSource;
use stm32h7xx_hal::hal::digital::v2::OutputPin;
use stm32h7xx_hal::hal::digital::v2::ToggleableOutputPin;
use stm32h7xx_hal::hal::digital::v2::InputPin;

use stm32h7xx_hal::{pac, prelude::*}; //}, Never};

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
//use core::ptr::null_mut;
//use stm32h7::stm32h743;
//use core::borrow::BorrowMut;
//use stm32h7::stm32h743::{GPIOB, GPIOC, GPIOE};

#[allow(unused)]
struct SharedAppContext {
  ccdr: Ccdr,
//  gpiob: stm32h7xx_hal::gpio::gpiob::Parts,
//  gpioc: stm32h7xx_hal::gpio::gpioc::Parts,
//  gpioe: stm32h7xx_hal::gpio::gpioe::Parts,
  user_led1: stm32h7xx_hal::gpio::gpiob::PB0<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>,
  user_led2: stm32h7xx_hal::gpio::gpioe::PE1<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>,
  user_led3: stm32h7xx_hal::gpio::gpiob::PB14<stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>>,
  user_butt: stm32h7xx_hal::gpio::gpioc::PC13<stm32h7xx_hal::gpio::Input<stm32h7xx_hal::gpio::PullDown>>,

}

//static APP_CTX: Option<SharedAppContext> = None;





//#[exception]
//#[no_mangle]
//fn DefaultHandler(arg: i16) -> ! {
//  let mut err_log =  semihosting::InterruptFree::<_>::stderr().unwrap();
//  d_println!(err_log, "DefaultHandler {}", arg );
//  panic!("DefaultHandler");
//}

#[exception]
fn DefaultHandler(irqn: i16) {
  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
  d_println!(log, "IRQn = {}", irqn);
}

//#[exception]
//fn SVCall() -> ! {
//  let mut err_log =  semihosting::InterruptFree::<_>::stdout().unwrap();
//  d_println!(err_log, " SVCall " );
//  panic!("o noes");
//}


#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {

  loop {

  }
}


#[no_mangle]
extern "C" fn start_default_task(arg: *mut cty::c_void) {

  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();

  d_println!(log, "Start default loop...");

//  let mut app_ctx: &mut SharedAppContext = unsafe { &mut *(arg as *mut SharedAppContext) };
  let app_ctx: &mut SharedAppContext = unsafe { &mut *(arg as *mut SharedAppContext) };

  let core_peripherals = cortex_m::Peripherals::take().unwrap();
  let mut delay = core_peripherals.SYST.delay(app_ctx.ccdr.clocks);
  loop {
    let app_ctx: &mut SharedAppContext = unsafe { &mut *(arg as *mut SharedAppContext) };
    let user_butt_pressed = app_ctx.user_butt.is_high().unwrap_or(false) ;
    if !user_butt_pressed {
      app_ctx.user_led1.toggle().unwrap();
      app_ctx.user_led2.toggle().unwrap();
      app_ctx.user_led3.toggle().unwrap();
    }
    else {
      d_print!(log, ".");
    }
    delay.delay_ms(100_u32);
    //cmsis_rtos2::rtos_os_delay(3); //  heartbeat
  }
}


fn setup_peripherals() -> SharedAppContext {
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
  let user_butt= gpioc.pc13.into_pull_down_input();

  //set initial states of user LEDs
  user_led1.set_high().unwrap();
  user_led2.set_low().unwrap();
  user_led3.set_high().unwrap();

  d_println!(log, "done!");

  SharedAppContext {
    ccdr,
    user_led1,
    user_led2,
    user_led3,
    user_butt,
  }
}


fn setup_rtos(app_ctx: &mut SharedAppContext) -> osThreadId_t {
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


  let ctx_ptr: *mut cty::c_void = app_ctx as *mut _ as *mut cty::c_void;

  let default_task_handle = cmsis_rtos2::rtos_os_thread_new(
    Some(start_default_task),
    ctx_ptr, //core::ptr::null_mut(),
    core::ptr::null(),
//    &default_task_attributes
  );
  if default_task_handle.is_null() {
    d_println!(log, "rtos_os_thread_new failed!");
    return core::ptr::null_mut()
  }
  d_println!(log, "rtos_os_thread_new ok! ");

  let rc = cmsis_rtos2::rtos_kernel_start();
  d_println!(log, "kernel_start rc: {}", rc);

  d_println!(log,"RTOS done!");

  default_task_handle
}

#[entry]
fn main() -> ! {
  //let core_peripherals = cortex_m::Peripherals::take().unwrap();
#[cfg(debug_assertions)]
//  let mut log =  semihosting::InterruptFree::<_>::stdout().unwrap();
//  let mut log = InterruptSyncItm::new(Itm::new(cp.ITM));

  let mut app_ctx = setup_peripherals();
  let _default_thread_id = setup_rtos(&mut app_ctx);

  loop {
    cmsis_rtos2::rtos_os_thread_yield();
    //cmsis_rtos2::rtos_os_thread_join(default_thread_id);
  }

}
