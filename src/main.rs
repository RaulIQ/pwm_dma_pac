#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::println;
use stm32f7xx_hal::pac::{self, tim2::dmar};
use {defmt_rtt as _, panic_probe as _};

const ONE_THIRD: u16 = 120;
const TWO_THIRD: u16 = 240;

const DSHOT_FRAME_SIZE: usize = 16; // 16 bits per frame
static mut DSHOT_FRAME: [u16; DSHOT_FRAME_SIZE] = [0; DSHOT_FRAME_SIZE];

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

    let packet = create_dshot_packet(1046, false);
    prepare_dshot_frame(packet);

    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC;
    let tim = dp.TIM3;
    let gpio_a = dp.GPIOA;
    let dma1 = dp.DMA1;
    let pwr = dp.PWR;

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
        tim.arr.write(|w| w.bits(359)); // frequency
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
    tim.dier.modify(|_, w| w.cc1de().clear_bit());
    tim.dcr.modify(|_, w| w.dba().bits(13));
    // enable update generation - needed at first start
    tim.egr.write(|w| w.ug().set_bit());
    tim.egr.write(|w| w.ug().set_bit());
    // start pwm
    tim.cr1.modify(|_, w| w.cen().set_bit());


    unsafe {
        let dmar_addr = tim.dmar.as_ptr() as u32;

        // let dma_buf = [9, 18, 18, 9, 0, 18, 0];

        dma1.st[2].fcr.modify(|_, w| w.dmdis().set_bit());

        dma1.st[2].m0ar.write(|w| w.m0a().bits(DSHOT_FRAME.as_ptr() as u32));
        dma1.st[2].ndtr.write(|w| w.ndt().bits(DSHOT_FRAME_SIZE as u16));
        dma1.st[2].par.write(|w| w.pa().bits(dmar_addr));

        dma1.st[2].cr.reset();
        dma1.st[2].cr.modify(|_, w| w
            .chsel().bits(5)
            .mburst().incr4()
            .pburst().single()
            .pl().high()
            .msize().bits16() // value depends on the type of buffer elements 
            .psize().bits16() // value depends on the type with which the peripheral operates 
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
