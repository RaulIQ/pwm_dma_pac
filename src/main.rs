#![no_std]
#![no_main]

use cortex_m::asm::nop;
use cortex_m_rt::entry;
use defmt::println;
use stm32f7xx_hal::pac::{self};
use {defmt_rtt as _, panic_probe as _};

// Constants for DShot protocol
const ONE_THIRD: u16 = 120;
const TWO_THIRD: u16 = 240;
const DSHOT_FRAME_SIZE: usize = 18;
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
    for i in 0..DSHOT_FRAME_SIZE - 2 {
        let is_one = (packet & (1 << (15 - i))) != 0;
        unsafe {
            DSHOT_FRAME[i] = if is_one { TWO_THIRD } else { ONE_THIRD };
        }
    }
}

// Initialize clock and power settings
fn init_clocks(dp: &pac::Peripherals) {
    let rcc = &dp.RCC;
    let pwr = &dp.PWR;

    rcc.cr.modify(|_, w| w.hseon().on());
    while rcc.cr.read().hserdy().is_not_ready() {}
    println!("HSE ready");

    rcc.pllcfgr.modify(|_, w| w.pllsrc().hse());
    rcc.pllcfgr.modify(unsafe { |_, w| w.pllm().bits(4) });
    rcc.pllcfgr.modify(unsafe { |_, w| w.plln().bits(108) });
    rcc.pllcfgr.modify(|_, w| w.pllp().div2());

    rcc.cr.modify(|_, w| w.pllon().on());
    while rcc.cr.read().pllrdy().is_not_ready() {}
    println!("PLL ready");

    dp.FLASH.acr.modify(|_, w| w.latency().ws9());
    while !dp.FLASH.acr.read().latency().is_ws9() {}

    rcc.cfgr.modify(|_, w| w.ppre1().div2().ppre2().div2());
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}
    println!("PLL selected");
}

// Initialize GPIO for PWM output
fn init_gpio(dp: &pac::Peripherals) {
    let gpio_a = &dp.GPIOA;
    let rcc = &dp.RCC;

    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());

    gpio_a.moder.write(|w| w.moder6().alternate());
    gpio_a.otyper.write(|w| w.ot6().push_pull());
    gpio_a.ospeedr.write(|w| w.ospeedr6().very_high_speed());
    gpio_a.pupdr.write(|w| w.pupdr6().floating());
    gpio_a.afrl.write(|w| w.afrl6().af2()); 
}

// Initialize TIM3 for PWM generation
fn init_tim3(dp: &pac::Peripherals) {
    let tim = &dp.TIM3;
    let rcc = &dp.RCC;

    rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());

    unsafe {
        tim.arr.write(|w| w.bits(359)); // Set PWM frequency
        tim.ccr1().write(|w| w.bits(0)); // Duty cycle
    }

    tim.ccmr1_output().modify(|_, w| w.oc1ce().clear_bit().oc1pe().enabled().oc1m().pwm_mode1());
    tim.ccer.write(|w| w.cc1e().set_bit());
    tim.cr1.modify(|_, w| w.arpe().set_bit());
    tim.dier.modify(|_, w| w.ude().set_bit());
    tim.dcr.modify(|_, w| w.dba().bits(13));

    // Update generation and start PWM
    tim.egr.write(|w| w.ug().set_bit());
    tim.cr1.modify(|_, w| w.cen().set_bit());
}

// Initialize DMA for DShot data transmission
fn init_dma(dp: &pac::Peripherals) {
    let dma1 = &dp.DMA1;
    let tim = &dp.TIM3;
    let rcc = &dp.RCC;

    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());

    dma1.lifcr.write(|w| w
        .cfeif2().set_bit()  // Clear FIFO error interrupt flag
        .cdmeif2().set_bit() // Clear direct mode error interrupt flag
        .cteif2().set_bit()  // Clear transfer error interrupt flag
        .chtif2().set_bit()  // Clear half-transfer interrupt flag
        .ctcif2().set_bit()  // Clear transfer complete interrupt flag
    );

    unsafe {
        let dmar_addr = tim.dmar.as_ptr() as u32;

        dma1.st[2].fcr.modify(|_, w| w
            .dmdis().set_bit()
            .fth().full()
        );

        dma1.st[2].fcr.modify(|_, w| w.dmdis().set_bit());
        dma1.st[2].m0ar.write(|w| w.m0a().bits(DSHOT_FRAME.as_ptr() as u32));
        dma1.st[2].ndtr.write(|w| w.ndt().bits(DSHOT_FRAME_SIZE as u16));
        dma1.st[2].par.write(|w| w.pa().bits(dmar_addr));

        dma1.st[2].cr.modify(|_, w| w
            .chsel().bits(5)
            .mburst().single()
            .pburst().single()
            .pl().high()
            .msize().bits16()
            .psize().bits16()
            .minc().set_bit()
            .pinc().clear_bit()
            .circ().clear_bit()
            .dir().memory_to_peripheral()
            .teie().set_bit()
            .htie().set_bit()
            .tcie().set_bit()
            .en().enabled()
        );
    }
}

fn waveform_up(dp: &pac::Peripherals, frame: u16) {
    let dma1= &dp.DMA1;
    
    let packet = create_dshot_packet(frame, false);
    prepare_dshot_frame(packet);

    init_dma(&dp);
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    let tim = &dp.TIM3;
    let dma1 = &dp.DMA1;

    init_clocks(&dp);
    init_gpio(&dp);
    init_tim3(&dp);

    let arr = [1046, 999, 1046, 0];

    loop {
        for i in 0..4 {
            waveform_up(&dp, arr[i]);
            for _ in 0..10_000 {
                nop();
            }
        }

    }
}
