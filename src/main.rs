#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::println;
use stm32f7xx_hal::pac::{self};
use {defmt_rtt as _, panic_probe as _};


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC;
    let tim = dp.TIM3;
    let gpio_a = dp.GPIOA;
    let dma1 = dp.DMA1;
    let pwr = dp.PWR;

    rcc.apb1enr.modify(|_, w| w.pwren().set_bit());

    pwr.cr1.modify(|_, w| w.vos().scale1());

    pwr.cr1.modify(|_, w| w.oden().set_bit());
    while pwr.csr1.read().odrdy().bit_is_clear() {}

    pwr.cr1.modify(|_, w| w.odswen().set_bit());
    while pwr.csr1.read().odswrdy().bit_is_clear() {}

    rcc.cr.modify(|_, w| w.hseon().on());
    while rcc.cr.read().hserdy().is_not_ready() {}
    println!("Hse ready");

    rcc.pllcfgr.modify(|_, w| w.pllsrc().hse());
    rcc.pllcfgr.modify(unsafe {|_, w| w.pllm().bits(4)});
    rcc.pllcfgr.modify(unsafe {|_, w| w.plln().bits(108)});
    rcc.pllcfgr.modify(|_, w| w.pllp().div2());

    rcc.cr.modify(|_, w| w.pllon().on());
    while rcc.cr.read().pllrdy().is_not_ready() {}
    println!("Pll ready");

    dp.FLASH.acr.modify(|_, w| w.latency().ws9());
    while !dp.FLASH.acr.read().latency().is_ws9() {}

    rcc.cfgr.modify(|_, w| w
        .ppre1().div2()
        .ppre2().div2()
    );

    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}
    println!("pll selectected");

    rcc.apb1enr.write(|w| w.tim3en().set_bit());
    rcc.ahb1enr.write(|w| w.gpioaen().set_bit());
    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());

    gpio_a.moder.write(|w| w.moder6().alternate());
    gpio_a.otyper.write(|w| w.ot6().push_pull());
    gpio_a.ospeedr.write(|w| w.ospeedr6().very_high_speed());
    gpio_a.pupdr.write(|w| w.pupdr6().floating());
    gpio_a.afrl.write(|w| w.afrl6().af2()); 

    unsafe {
        tim.arr.write(|w| w.bits(26)); // frequency
        tim.ccr1().write(|w| w.bits(0));// duty cycle
    }

    // clear enable to zero just to be sure
    tim.ccmr1_output().modify(|_, w| w.oc1ce().clear_bit());
    //enable preload
    tim.ccmr1_output().modify(|_, w| w.oc1pe().enabled());
    // set pwm mode 1
    tim.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1());
    // enable output
    tim.ccer.write(|w| w.cc1e().set_bit());
    // enable auto-reload
    tim.cr1.modify(|_, w| w.arpe().set_bit());
    tim.dier.modify(|_, w| w.ude().set_bit());
    tim.dier.modify(|_, w| w.cc1de().set_bit());
    tim.dier.modify(|_, w| w.tde().set_bit());
    // enable update generation - needed at first start
    tim.egr.write(|w| w.ug().set_bit());
    tim.egr.write(|w| w.ug().set_bit());
    // start pwm
    tim.cr1.modify(|_, w| w.cen().set_bit());

    unsafe {
        let ccr1_addr = tim.ccr1() as *const _ as u32;
        let dma_buf = [9, 18, 18, 9, 18, 0, 9, 0, 0];
        // let dma_buf = [6, 9, 9, 12, 0, 0, 0];

        dma1.st[4].m0ar.write(|w| w.m0a().bits(dma_buf.as_ptr() as u32));
        dma1.st[4].ndtr.write(|w| w.ndt().bits(dma_buf.len() as u16));
        dma1.st[4].par.write(|w| w.pa().bits(ccr1_addr));

        dma1.st[4].cr.reset();
        dma1.st[4].cr.modify(|_, w| w
            .chsel().bits(5)
            .mburst().single()
            .pburst().single()
            .pl().high()
            .msize().bits32() // value depends on the type of buffer elements 
            .psize().bits32() // value depends on the type of buffer elements 
            .minc().set_bit()
            .pinc().clear_bit()
            .circ().enabled()
            .dir().memory_to_peripheral()
            .teie().set_bit()
            .htie().set_bit()
            .tcie().set_bit()
            .en().enabled()
        );
    }

    loop {}
}
