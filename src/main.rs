#![deny(unsafe_code)]
#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use ssd1306::{prelude::*, Builder};
use stm32f1xx_hal::{adc, delay::Delay, pac, prelude::*, spi::{Mode, Phase, Polarity, Spi}, timer::Timer}; // STM32F1 specific functions
use microfft;
use micromath::F32Ext;
use nb::block;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

#[allow(unused_imports)]

#[entry]
fn main() -> ! {
    
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    // let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(56.mhz())
        .pclk1(28.mhz())
        .freeze(&mut flash.acr);

    // hprintln!("adc freq: {}", clocks.adcclk().0).unwrap();

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);    

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);

    // Setup GPIOB
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    
    // configure pa0 as an analog input
    let mut ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);

    // disable jtag and use pb4
    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

    
    // Display

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);


    let mut rst = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    let dc = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        8.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let interface = display_interface_spi::SPIInterfaceNoCS::new(spi, dc);
    let mut disp: GraphicsMode<_> = Builder::new().connect(interface).into();

    disp.reset(&mut rst, &mut delay).unwrap();
    disp.init().unwrap();

    // stepper 1
    let mut purple_pa1_in1 = gpioa.pa1.into_push_pull_output(&mut gpioa.crl);
    let mut blue_pa2_in2 = gpioa.pa2.into_push_pull_output(&mut gpioa.crl);
    let mut green_pa3_in3 = gpioa.pa3.into_push_pull_output(&mut gpioa.crl);
    let mut yellow_pa4_in4 = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    // stepper 2
    let mut yellow_pb7_in1 = gpiob.pb7.into_push_pull_output(&mut gpiob.crl);
    let mut green_pb6_in2 = gpiob.pb6.into_push_pull_output(&mut gpiob.crl);
    let mut blue_pb5_in3 = gpiob.pb5.into_push_pull_output(&mut gpiob.crl);
    let mut purple_pb4_in4 = pb4.into_push_pull_output(&mut gpiob.crl);

    const STEPPER_DELAY: u16 = 2u16;
    
    // FFT samples buffers
    const N: usize = 256;
    let mut samples: [f32; N] = [0.0; N];
    
    loop {
        
        // get samples
        for i in 0..N {
            let data: u16 = adc1.read(&mut ch0).unwrap();
            samples[i as usize] = data as f32;
        }

        // clear display
        for i in 0..64 {
            for j in 0..128 {
                disp.set_pixel(j, i, 0);
            }
        }

        let spectrum = microfft::real::rfft_256(&mut samples);
        
        // define pass band filter
        let fl = (N/9) as usize;
        let fh = (N/7) as usize;

        // max amplitude
        let ampl_max = spectrum[fl..fh].iter()
        .map(|x| x.norm_sqr() as f32)
        .fold(0., |a, c| {
            if c > a {
                return c;
            }
            a
        });

        
        // get bins
        let mut first_detected_bin: usize = 0;
        let mut last_detected_bin: usize = 0;
        for i in fl..fh {
            let ampl_n = spectrum[i as usize].norm_sqr() as f32;
            let ampl = (64.*(ampl_n/(ampl_max))) as u32;

            // detect tone width
            if ampl > 5 {
                if first_detected_bin == 0 {
                    first_detected_bin = i;
                }
                last_detected_bin = i+1;
            }
            
            for j in 0..ampl {
                disp.set_pixel((i-fl as usize) as u32, 64-j, 1);
            }
        }

        let tone_width = last_detected_bin - first_detected_bin;

        // enable steppers when pure tone is detected
        if tone_width < 4 {
            led.set_low();
            for _i in 0..30 {

                // seq1 stepper 1
                yellow_pa4_in4.set_high().unwrap();
                purple_pa1_in1.set_low().unwrap();
                blue_pa2_in2.set_low().unwrap();
                green_pa3_in3.set_low().unwrap();
                
                // seq1 stepper 2
                purple_pb4_in4.set_high().unwrap();
                yellow_pb7_in1.set_low().unwrap();
                green_pb6_in2.set_low().unwrap();
                blue_pb5_in3.set_low().unwrap();
                
                delay.delay_ms(STEPPER_DELAY);

                // seq3 stepper 1
                green_pa3_in3.set_high().unwrap();
                purple_pa1_in1.set_low().unwrap();
                blue_pa2_in2.set_low().unwrap();
                yellow_pa4_in4.set_low().unwrap();

                // seq3 stepper 2
                blue_pb5_in3.set_high().unwrap();
                yellow_pb7_in1.set_low().unwrap();
                green_pb6_in2.set_low().unwrap();
                purple_pb4_in4.set_low().unwrap();
                
                delay.delay_ms(STEPPER_DELAY);

                // seq5 stepper 1
                blue_pa2_in2.set_high().unwrap();
                purple_pa1_in1.set_low().unwrap();
                green_pa3_in3.set_low().unwrap();
                yellow_pa4_in4.set_low().unwrap();

                // seq5 stepper 2
                green_pb6_in2.set_high().unwrap();
                yellow_pb7_in1.set_low().unwrap();
                blue_pb5_in3.set_low().unwrap();
                purple_pb4_in4.set_low().unwrap();
                
                delay.delay_ms(STEPPER_DELAY);

                // seq7 stepper 1
                purple_pa1_in1.set_high().unwrap();
                blue_pa2_in2.set_low().unwrap();
                green_pa3_in3.set_low().unwrap();
                yellow_pa4_in4.set_low().unwrap();

                // seq7 stepper 2
                yellow_pb7_in1.set_high().unwrap();
                green_pb6_in2.set_low().unwrap();
                blue_pb5_in3.set_low().unwrap();
                purple_pb4_in4.set_low().unwrap();
                
                delay.delay_ms(STEPPER_DELAY);

            }
        } else {
            led.set_high();
        }
        
        disp.flush().unwrap();
    }
}