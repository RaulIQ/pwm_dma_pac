#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::println;
use stm32f7xx_hal::pac::{self};
use {defmt_rtt as _, panic_probe as _};

const ONE_THIRD: u16 = 120;
const TWO_THIRD: u16 = 240;

const DSHOT_FRAME_SIZE: usize = 16; // 16 bits per frame
static mut DSHOT_FRAME: [u16; DSHOT_FRAME_SIZE] = [0; DSHOT_FRAME_SIZE];

fn init_outputs(dp: &pac::Peripherals) {
    let rcc = &dp.RCC;

    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
    rcc.ahb1enr.modify(|_, w| w.gpiocen().set_bit());

    let gpio_a = &dp.GPIOA;
    let gpio_c = &dp.GPIOC;

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
}

fn init_tim3(dp: &pac::Peripherals) {
    let rcc = &dp.RCC;
    rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());

    let tim = &dp.TIM3;

    unsafe {
        tim.arr.write(|w| w.bits(359)); // 600KHz
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

}

fn init_dma1(dp: &pac::Peripherals, dmar_addr: u32) {
    let rcc = &dp.RCC;

    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());

    let dma1 = &dp.DMA1;

    unsafe {
        dma1.st[2].fcr.modify(|_, w| w
            .dmdis().set_bit()
            .fth().full()
        );

        dma1.st[2].m0ar.write(|w| w.m0a().bits(DSHOT_FRAME.as_ptr() as u32));
        dma1.st[2].ndtr.write(|w| w.ndt().bits(DSHOT_FRAME_SIZE as u16));
        dma1.st[2].par.write(|w| w.pa().bits(dmar_addr));

        dma1.st[2].cr.reset();
        dma1.st[2].cr.modify(|_, w| w
            .chsel().bits(5)
            .mburst().incr4()
            .pburst().incr4()
            .pl().very_high()
            .msize().bits16()
            .psize().bits16() 
            .minc().set_bit()
            .pinc().clear_bit()
            .circ().enabled()
            .dir().memory_to_peripheral()
            .teie().clear_bit()
            .htie().clear_bit()
            .tcie().clear_bit()
            .en().enabled()
        );
    }

}

fn init_power_and_clock(dp: &pac::Peripherals) {
    let rcc = &dp.RCC;
    let pwr = &dp.PWR;

    rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
    pwr.cr1.modify(|_, w| w.vos().scale1().oden().set_bit());
    while pwr.csr1.read().odrdy().bit_is_clear() {}
    pwr.cr1.modify(|_, w| w.odswen().set_bit());
    while pwr.csr1.read().odswrdy().bit_is_clear() {}

    // Configure HSE as PLL source
    rcc.cr.modify(|_, w| w.hseon().on());
    while rcc.cr.read().hserdy().is_not_ready() {}
    println!("HSE ready");

    rcc.pllcfgr.modify(|_, w| w.pllsrc().hse());
    rcc.pllcfgr.modify(unsafe { |_, w| w.pllm().bits(4) });
    rcc.pllcfgr.modify(unsafe { |_, w| w.plln().bits(216) });
    rcc.pllcfgr.modify(|_, w| w.pllp().div2());
    rcc.cr.modify(|_, w| w.pllon().on());

    while rcc.cr.read().pllrdy().is_not_ready() {}
    println!("PLL ready");

    dp.FLASH.acr.modify(|_, w| w.latency().ws7());
    while !dp.FLASH.acr.read().latency().is_ws7() {}

    rcc.dckcfgr1.modify(|_, w| w.timpre().mul4());
    rcc.cfgr.modify(|_, w| w.ppre1().div4().ppre2().div2());
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}
    println!("PLL selected");
}

// Create DShot data packet
fn create_dshot_packet(throttle: u16, telemetry: bool) -> u16 {
    let mut packet = (throttle & 0x07FF) << 1; // 11-bit throttle
    if telemetry {
        packet |= 1; // Set telemetry bit if needed
    }

    let crc = ((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F) as u16; // 4-bit CRC
    (packet << 4) | crc
}

// Prepare DShot frame as array of pulse widths for each bit
fn prepare_dshot_frame(packet: u16) {
    for i in 0..DSHOT_FRAME_SIZE {
        // Calculate each bit’s pulse width
        let is_one = (packet & (1 << (15 - i))) != 0;
        unsafe {
            DSHOT_FRAME[i] = if is_one {
                TWO_THIRD // Set for ~62.5% high pulse
            } else {
                ONE_THIRD // Set for ~37.5% high pulse
            };
        }
    }
}

#[entry]
fn main() -> ! {
    let dp = &pac::Peripherals::take().unwrap();

    let tim = &dp.TIM3;
    init_power_and_clock(dp);
    init_outputs(dp);
    init_tim3(dp);

    let dmar_addr = tim.dmar.as_ptr() as u32;
    init_dma1(dp, dmar_addr);

    let packet = create_dshot_packet(1046, false);
    prepare_dshot_frame(packet);

    unsafe {
        println!("{}", DSHOT_FRAME);
    }

    loop {}
}
