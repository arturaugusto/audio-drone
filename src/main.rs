#![deny(unsafe_code)]
#![no_std]
#![no_main]

use embedded_hal::digital::v2::OutputPin;
use panic_halt as _;
use ssd1306::{prelude::*, Builder};
use stm32f1xx_hal::{adc, delay::Delay, pac, prelude::*, pwm_input::*, spi::{Mode, Phase, Polarity, Spi}, timer::Timer}; // STM32F1 specific functions
use microfft;
use micromath::F32Ext;
use nb::block;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

#[allow(unused_imports)]



pub struct Goertzel {
    s_prev: f32,
    s_prev2: f32,
    totalpower: f32,
    n: i32,
    mean: f32,
    mean_prev: f32,

    freq: f32,
    samplef: f32
}

impl Goertzel {
    pub fn new(freq: f32, samplef: f32) -> Self {
        Self {
            s_prev: 0.,
            s_prev2: 0.,
            totalpower: 0.,
            n: 0,
            mean: 0.,
            mean_prev: 0.,

            freq: freq,
            samplef: samplef
        }
    }
    
    pub fn filter (&mut self, sample: f32) -> f32 {
        // real time mean
        // https://dsp.stackexchange.com/questions/811/determining-the-mean-and-standard-deviation-in-real-time
        self.mean_prev = self.mean;
        self.n = self.n + 1;
        self.mean = self.mean + (sample-self.mean)/self.n as f32;
        let x: f32 = sample - self.mean;

        let normalizedfreq: f32 = self.freq/self.samplef;
        let coeff: f32 = 2.*(2.*3.1416*normalizedfreq).cos();
        let s: f32 = x + coeff * self.s_prev - self.s_prev2;
        self.s_prev2 = self.s_prev;
        self.s_prev = s;

        let power: f32 = self.s_prev2*self.s_prev2+self.s_prev*self.s_prev-coeff*self.s_prev*self.s_prev2;
        self.totalpower = self.totalpower + x*x;
        if self.totalpower < 0. {
            self.totalpower = 1.
        }
        return power / self.totalpower / self.n as f32
    }
}



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
    let mut dbg = dp.DBGMCU;


    // disable jtag and use pb4
    let (_pa15, _pb3, pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);


    // TIM3
    // let pa8_pwm = gpioa.pa8.into_alternate_push_pull(&mut gpioa.crh);
    // let pa9_pwm = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);

    let pa8_pwm_in = gpioa.pa8;
    let pa9_pwm_in = gpioa.pa9;

    // let pwm = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).pwm((pa8_pwm, pa9_pwm), &mut afio.mapr, 1.khz());
    let pwm_input = Timer::tim1(dp.TIM1, &clocks, &mut rcc.apb2).pwm_input((pa8_pwm_in, pa9_pwm_in), &mut afio.mapr, &mut dbg, Configuration::Frequency(1.khz()));

    // let max = pwm.get_max_duty();

    // let mut pwm_channels = pwm.split();

    // // Enable the individual channels
    // pwm_channels.0.enable();
    // //pwm_channels.1.enable();

    // // full
    // pwm_channels.0.set_duty(max/2);


    
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

    
    // FFT samples buffers
    const N: usize = 100;
    

    const STEPPER_DELAY: u16 = 3u16;
    const DETECTION_TRESH: f32 = 0.2;
    
    // let mut samples: [f32; N] = [0.0; N];
    let mut samples: [u16; N] = [0u16; N];

    

    let sample_freq: f32 = 1.0/((4 * STEPPER_DELAY) as f32);
    
    loop {
        let mut detection_count: u16 = 0u16;

        let mut g_fw_power: f32 = 0.;
        let mut g_left_power: f32 = 0.;
        let mut g_right_power: f32 = 0.;
        let mut g_rev_power: f32 = 0.;

        let mut stepper_1_on_fw = false;
        let mut stepper_1_on_rev = false;
        let mut stepper_2_on_fw = false;
        let mut stepper_2_on_rev = false;

        // get samples
        for i in 0..N {
            let data: u16 = adc1.read(&mut ch0).unwrap();
            samples[i as usize] = data;
            delay.delay_us(100u16);
        }
        
        // ~ 1076 Hz
        let mut g_fw = Goertzel::new(1200., 10000.);
        for x in samples {
            g_fw_power = g_fw.filter(x as f32);
        }
        if g_fw_power > DETECTION_TRESH {
            detection_count += 1u16;
            stepper_1_on_fw = true;
            stepper_1_on_rev = false;
            stepper_2_on_fw = true;
            stepper_2_on_rev = false;
        }

        // ~ 1202 Hz
        let mut g_left = Goertzel::new(1400., 10000.);
        for x in samples {
            g_left_power = g_left.filter(x as f32);
        }
        if g_left_power > DETECTION_TRESH {
            detection_count += 1u16;
            stepper_1_on_fw = true;
            stepper_1_on_rev = false;
            stepper_2_on_fw = false;
            stepper_2_on_rev = true;
        }

        // ~ 1280 Hz
        let mut g_right = Goertzel::new(1600., 10000.);
        for x in samples {
            g_right_power = g_right.filter(x as f32);
        }
        if g_right_power > DETECTION_TRESH {
            detection_count += 1u16;
            stepper_1_on_fw = false;
            stepper_1_on_rev = true;
            stepper_2_on_fw = true;
            stepper_2_on_rev = false;
        }

        // 1454 Hz
        let mut g_rev = Goertzel::new(1800., 10000.);
        for x in samples {
            g_rev_power = g_rev.filter(x as f32);
        }
        if g_rev_power > DETECTION_TRESH {
            detection_count += 1u16;
            stepper_1_on_fw = false;
            stepper_1_on_rev = true;
            stepper_2_on_fw = false;
            stepper_2_on_rev = true;
        }
        
        if g_fw_power > DETECTION_TRESH || g_left_power > DETECTION_TRESH || g_right_power > DETECTION_TRESH || g_rev_power > DETECTION_TRESH {
        // if g_left_power > DETECTION_TRESH {
            led.set_low();
        } else {
            led.set_high();
        }

        if detection_count == 1u16 {
            // hprintln!("adc freq: {}", first_detected_bin).unwrap();
            
            // led.set_low();
            for _i in 0..30 {

                // fw seq1 stepper 1
                if stepper_1_on_fw {yellow_pa4_in4.set_high().unwrap();purple_pa1_in1.set_low().unwrap();blue_pa2_in2.set_low().unwrap();green_pa3_in3.set_low().unwrap();}
                
                // // fw seq1 stepper 2
                if stepper_2_on_fw {purple_pb4_in4.set_high().unwrap();yellow_pb7_in1.set_low().unwrap();green_pb6_in2.set_low().unwrap();blue_pb5_in3.set_low().unwrap();}

                // // rev seq7 stepper 1
                if stepper_1_on_rev {purple_pa1_in1.set_high().unwrap();blue_pa2_in2.set_low().unwrap();green_pa3_in3.set_low().unwrap();yellow_pa4_in4.set_low().unwrap();}

                // // rev seq7 stepper 2
                if stepper_2_on_rev {yellow_pb7_in1.set_high().unwrap();green_pb6_in2.set_low().unwrap();blue_pb5_in3.set_low().unwrap();purple_pb4_in4.set_low().unwrap();}
                
                delay.delay_ms(STEPPER_DELAY);

                // // fw seq3 stepper 1
                if stepper_1_on_fw {green_pa3_in3.set_high().unwrap();purple_pa1_in1.set_low().unwrap();blue_pa2_in2.set_low().unwrap();yellow_pa4_in4.set_low().unwrap();}

                // // fw seq3 stepper 2
                if stepper_2_on_fw {blue_pb5_in3.set_high().unwrap();yellow_pb7_in1.set_low().unwrap();green_pb6_in2.set_low().unwrap();purple_pb4_in4.set_low().unwrap();}
                
                // // rev seq5 stepper 1
                if stepper_1_on_rev {blue_pa2_in2.set_high().unwrap();purple_pa1_in1.set_low().unwrap();green_pa3_in3.set_low().unwrap();yellow_pa4_in4.set_low().unwrap();}

                // // rev seq5 stepper 2
                if stepper_2_on_rev {green_pb6_in2.set_high().unwrap();yellow_pb7_in1.set_low().unwrap();blue_pb5_in3.set_low().unwrap();purple_pb4_in4.set_low().unwrap();}

                delay.delay_ms(STEPPER_DELAY);

                // // fw seq5 stepper 1
                if stepper_1_on_fw {blue_pa2_in2.set_high().unwrap();purple_pa1_in1.set_low().unwrap();green_pa3_in3.set_low().unwrap();yellow_pa4_in4.set_low().unwrap();}

                // // fw seq5 stepper 2
                if stepper_2_on_fw {green_pb6_in2.set_high().unwrap();yellow_pb7_in1.set_low().unwrap();blue_pb5_in3.set_low().unwrap();purple_pb4_in4.set_low().unwrap();}

                // // rev seq3 stepper 1
                if stepper_1_on_rev {green_pa3_in3.set_high().unwrap();purple_pa1_in1.set_low().unwrap();blue_pa2_in2.set_low().unwrap();yellow_pa4_in4.set_low().unwrap();}

                // // rev seq3 stepper 2
                if stepper_2_on_rev {blue_pb5_in3.set_high().unwrap();yellow_pb7_in1.set_low().unwrap();green_pb6_in2.set_low().unwrap();purple_pb4_in4.set_low().unwrap();}
                
                delay.delay_ms(STEPPER_DELAY);

                // // fw seq7 stepper 1
                if stepper_1_on_fw {purple_pa1_in1.set_high().unwrap();blue_pa2_in2.set_low().unwrap();green_pa3_in3.set_low().unwrap();yellow_pa4_in4.set_low().unwrap();}

                // // fw seq7 stepper 2
                if stepper_2_on_fw {yellow_pb7_in1.set_high().unwrap();green_pb6_in2.set_low().unwrap();blue_pb5_in3.set_low().unwrap();purple_pb4_in4.set_low().unwrap();}
                
                // // rev seq1 stepper 1
                if stepper_1_on_rev {yellow_pa4_in4.set_high().unwrap();purple_pa1_in1.set_low().unwrap();blue_pa2_in2.set_low().unwrap();green_pa3_in3.set_low().unwrap();}
                
                // // rev seq1 stepper 2
                if stepper_2_on_rev {purple_pb4_in4.set_high().unwrap();yellow_pb7_in1.set_low().unwrap();green_pb6_in2.set_low().unwrap();blue_pb5_in3.set_low().unwrap();}

                delay.delay_ms(STEPPER_DELAY);

            }
        } else {
            //led.set_high();
            delay.delay_ms(4 * STEPPER_DELAY);
        }
        
        // disp.flush().unwrap();
    }
}