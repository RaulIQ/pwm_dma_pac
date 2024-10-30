#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::println;
use stm32f7xx_hal::pac::{self, tim2::dmar};
use {defmt_rtt as _, panic_probe as _};


#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC;
    let tim = dp.TIM3;
    let gpio_a = dp.GPIOA;
    let gpio_c = dp.GPIOC;
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
    rcc.ahb1enr.modify(|_, w| w.gpiocen().set_bit());
    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());

    gpio_a.moder.write(|w| w.moder6().alternate());
    gpio_a.otyper.write(|w| w.ot6().push_pull());
    gpio_a.ospeedr.write(|w| w.ospeedr6().very_high_speed());
    gpio_a.pupdr.write(|w| w.pupdr6().floating());
    gpio_a.afrl.write(|w| w.afrl6().af2()); 

    gpio_c.moder.modify(|_, w| w.moder7().alternate());
    gpio_c.otyper.modify(|_, w| w.ot7().push_pull());
    gpio_c.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());
    gpio_c.pupdr.modify(|_, w| w.pupdr7().floating());
    gpio_c.afrl.modify(|_, w| w.afrl7().af2()); 

    gpio_c.moder.modify(|_, w| w.moder8().alternate());
    gpio_c.otyper.modify(|_, w| w.ot8().push_pull());
    gpio_c.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed());
    gpio_c.pupdr.modify(|_, w| w.pupdr8().floating());
    gpio_c.afrh.modify(|_, w| w.afrh8().af2()); 

    gpio_c.moder.modify(|_, w| w.moder9().alternate());
    gpio_c.otyper.modify(|_, w| w.ot9().push_pull());
    gpio_c.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());
    gpio_c.pupdr.modify(|_, w| w.pupdr9().floating());
    gpio_c.afrh.modify(|_, w| w.afrh9().af2()); 

    unsafe {
        tim.arr.write(|w| w.bits(35)); // frequency
        tim.ccr1().write(|w| w.bits(0));// duty cycle
        tim.ccr2().write(|w| w.bits(0));
        tim.ccr3().write(|w| w.bits(0));
        tim.ccr4().write(|w| w.bits(0));
    }

    tim.ccmr1_output().modify(|_, w| w
        .oc1ce().clear_bit() // clear enable to zero just to be sure
        .oc2ce().clear_bit()
        .oc1pe().enabled() //enable preload
        .oc2pe().enabled()
        .oc1m().pwm_mode1() // set pwm mode 1
        .oc2m().pwm_mode1()
    );

    tim.ccmr2_output().modify(|_, w| w
        .oc3ce().clear_bit() // clear enable to zero just to be sure
        .oc4ce().clear_bit()
        .oc3pe().enabled() //enable preload
        .oc4pe().enabled()
        .oc3m().pwm_mode1() // set pwm mode 1
        .oc4m().pwm_mode1()
    );
    
    // enable output
    tim.ccer.write(|w| w
        .cc1e().set_bit()
        .cc2e().set_bit()
        .cc3e().set_bit()
        .cc4e().set_bit()
    );
    // enable auto-reload
    tim.cr1.modify(|_, w| w.arpe().set_bit());

    tim.dier.modify(|_, w| w.ude().set_bit());
    tim.dier.modify(|_, w| w.cc1de().clear_bit());
    tim.dier.modify(|_, w| w.cc2de().clear_bit());
    tim.dier.modify(|_, w| w.cc3de().clear_bit());
    tim.dier.modify(|_, w| w.cc4de().clear_bit());
    
    tim.dcr.modify(|_, w| w.dba().bits(13));
    tim.dcr.modify(unsafe {
        |_, w| w.dbl().bits(0b11)
    });
    // enable update generation - needed at first start
    tim.egr.write(|w| w.ug().set_bit());
    tim.egr.write(|w| w.ug().set_bit());
    // start pwm
    tim.cr1.modify(|_, w| w.cen().set_bit());

    unsafe {
        let dmar_addr = tim.dmar.as_ptr() as u32;
        let dma_buf: [u16; 16] = [9, 9, 9, 9, 18, 18, 18, 18, 0, 0, 0, 0, 0, 0 ,0, 0];

        dma1.st[2].m0ar.write(|w| w.m0a().bits(dma_buf.as_ptr() as u32));
        dma1.st[2].ndtr.write(|w| w.ndt().bits(dma_buf.len() as u16));
        dma1.st[2].par.write(|w| w.pa().bits(dmar_addr));

        dma1.st[2].cr.reset();
        dma1.st[2].cr.modify(|_, w| w
            .chsel().bits(5)
            .mburst().single()
            .pburst().single()
            .pl().high()
            .msize().bits16() // value depends on the type of buffer elements 
            .psize().bits16() // value depends on the type of buffer elements 
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
