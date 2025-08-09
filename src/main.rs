#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]
#![allow(warnings)]

use embassy_executor::Spawner;
use embassy_net::Config;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::Duration;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::ascii::FONT_6X13_BOLD;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::Point;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::mono_font::ascii::FONT_5X7;
use embedded_graphics::prelude::RgbColor;
use embedded_graphics::text::Alignment;
use embedded_graphics::text::Text;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Pin;
use esp_hal::init;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::interrupt::Priority;
use esp_hal::main;
// use esp_hal::peripherals;
use esp_hal::rng::Rng;
use esp_hal::time::Rate;
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_embassy::InterruptExecutor;
use esp_hub75::Color;
use esp_hub75::Hub75;
use esp_hub75::Hub75Pins16;
use esp_hub75::framebuffer::compute_frame_count;
use esp_hub75::framebuffer::compute_rows;
use esp_hub75::framebuffer::plain::DmaFrameBuffer;
use esp_println::logger::init_logger_from_env;
use log::*;
// use alloc::string::ToString;
// use alloc::string::ToString;
extern crate alloc;
use alloc::string::ToString;
use log::info;
use static_cell::make_static;

esp_bootloader_esp_idf::esp_app_desc!();

const ROWS: usize = 32;
const COLS: usize = 64;
const BITS: u8 = 2;
const NROWS: usize = compute_rows(ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type FBType = DmaFrameBuffer<ROWS, COLS, NROWS, BITS, FRAME_COUNT>;
type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut FBType>;

pub struct Hub75Peripherals<'d> {
    pub lcd_cam: esp_hal::peripherals::LCD_CAM<'d>,
    pub dma_channel: esp_hal::peripherals::DMA_CH0<'d>,
    pub pins: Hub75Pins16<'d>,
}

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_alloc::heap_allocator!(size: 72 * 1024);

      let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;

    let WIFI_SSID = option_env!("WIFI_SSID")
        .unwrap_or("default_ssid")
        .to_string();
    let WIFI_PWD = option_env!("WIFI_PWD")
        .unwrap_or("default_password")
        .to_string();
    info!("WIFI_SSID: {}, WIFI_PWD: {}", WIFI_SSID, WIFI_PWD);
    // Init Embassy
    let system_timer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(system_timer.alarm0);
    let mut rng = Rng::new(peripherals.RNG);
    let init = &*mk_static!(
        esp_wifi::EspWifiController<'static>,
        esp_wifi::init(timg0.timer0, rng).unwrap()
    );
    let wifi = peripherals.WIFI;

    let (mut controller, interfaces) = esp_wifi::wifi::new(&init, wifi).unwrap();

    // let (wifi_sta_interface, controller) = esp_wifi::wifi::new(init,
    // wifi).unwrap();

    let sta_config = Config::dhcpv4(Default::default());

    let seed = rng.random();

    let (sta_stack, sta_runner) = embassy_net::new(
        interfaces.sta,
        sta_config,
        mk_static!(
            embassy_net::StackResources<3>,
            embassy_net::StackResources::<3>::new()
        ),
        seed.into(),
    );

    spawner.must_spawn(net_task(sta_runner));

    let config = esp_wifi::wifi::Configuration::Client(esp_wifi::wifi::ClientConfiguration {
        ssid: WIFI_SSID,
        password: WIFI_PWD,
        ..Default::default()
    });

    controller.set_configuration(&config).unwrap();
    controller.start().unwrap();
    controller.connect().unwrap();

    loop {
        debug!(
            "link_up={:?}, config_up={}, ip={:?}",
            sta_stack.is_link_up(),
            sta_stack.is_config_up(),
            sta_stack.config_v4().clone().map(|ip| ip.address),
        );

        if sta_stack.is_link_up() && sta_stack.is_config_up() {
            info!("WiFi connected!");
            break;
        };
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }


    let pins = Hub75Pins16 {
        red1: peripherals.GPIO42.degrade(),
        grn1: peripherals.GPIO41.degrade(),
        blu1: peripherals.GPIO40.degrade(),
        red2: peripherals.GPIO38.degrade(),
        grn2: peripherals.GPIO39.degrade(),
        blu2: peripherals.GPIO37.degrade(),

        addr0: peripherals.GPIO45.degrade(), // A  (⚠ GPIO45 is input-only on S3)
        addr1: peripherals.GPIO36.degrade(), // B
        addr2: peripherals.GPIO48.degrade(), // C
        addr3: peripherals.GPIO35.degrade(), // D
        addr4: peripherals.GPIO0.degrade(),  // E (1/8 scan: leave panel E unconnected)

        blank: peripherals.GPIO14.degrade(), // OE
        clock: peripherals.GPIO2.degrade(),  // CLK  ← make sure this matches your wiring
        latch: peripherals.GPIO47.degrade(), // LAT
    };

    let hub75_per = Hub75Peripherals {
        dma_channel: peripherals.DMA_CH0,
        lcd_cam: peripherals.LCD_CAM,
        pins,
    };

    let fb0 = mk_static!(FBType, FBType::new());
    let fb1 = mk_static!(FBType, FBType::new());

    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();

    let cpu1_fnctn = {
        move || {
            use esp_hal_embassy::Executor;
            let hp_executor = mk_static!(
                InterruptExecutor<2>,
                InterruptExecutor::new(software_interrupt)
            );
            let high_pri_spawner = hp_executor.start(Priority::Priority3);

            // hub75 runs as high priority task
            high_pri_spawner
                .spawn(hub75_task(hub75_per, &RX, &TX, fb1))
                .ok();

            let lp_executor = mk_static!(Executor, Executor::new());
            // display task runs as low priority task
            lp_executor.run(|spawner| {
                spawner.spawn(display_task(&TX, &RX, fb0)).ok();
            });
        }
    };

    use esp_hal::system::CpuControl;
    use esp_hal::system::Stack;
    use esp_hal_embassy::Executor;
    let cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    const DISPLAY_STACK_SIZE: usize = 8192;
    let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());
    let mut _cpu_control = cpu_control;

    #[allow(static_mut_refs)]
    let _guard = _cpu_control
        .start_app_core(app_core_stack, cpu1_fnctn)
        .unwrap();

    spawner.must_spawn(wifi_spam_task(sta_stack));

    loop {
        embassy_time::Timer::after(Duration::from_millis(100)).await;
    }
}

use embassy_net::udp::UdpMetadata;
use embassy_net::{Ipv4Address, udp::UdpSocket};
use smoltcp::storage::PacketMetadata;

#[embassy_executor::task]
pub async fn wifi_spam_task(stack: embassy_net::Stack<'static>) {
    let rx_buf = mk_static!([u8; 512], [0u8; 512]);
    let tx_buf = mk_static!([u8; 512], [0u8; 512]);
    let rx_meta = mk_static!([PacketMetadata<UdpMetadata>; 4], [PacketMetadata::EMPTY; 4]);
    let tx_meta = mk_static!([PacketMetadata<UdpMetadata>; 4], [PacketMetadata::EMPTY; 4]);
    info!("Starting UDP socket task");
    let mut socket = UdpSocket::new(stack, rx_meta, rx_buf, tx_meta, tx_buf);
    socket.bind(12345).unwrap();

    let packet = [0x42; 64];
    let dest = (Ipv4Address::new(8, 8, 8, 8), 12345);

    loop {
        info!("SPAM");
        let _ = socket.send_to(&packet, dest).await;
        embassy_time::Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task(pool_size = 3)]
pub async fn net_task(
    mut runner: embassy_net::Runner<'static, esp_wifi::wifi::WifiDevice<'static>>,
) {
    runner.run().await
}


#[embassy_executor::task]
async fn hub75_task(
    peripherals: Hub75Peripherals<'static>,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut FBType,
) {
    info!("hub75_task: starting!");

    // let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());
    let mut hub75 = Hub75::new_async(
        peripherals.lcd_cam,
        peripherals.pins,
        peripherals.dma_channel,
        tx_descriptors,
        Rate::from_mhz(20),
    )
    .expect("failed to create Hub75!");

    let mut fb = fb;

    loop {
        if rx.signaled() {
            let new_fb = rx.wait().await;
            tx.signal(fb);
            fb = new_fb;
        }

        let mut xfer = hub75
            .render(fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to start render!");
        xfer.wait_for_done()
            .await
            .expect("rendering wait_for_done failed!");
        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75;
        result.expect("transfer failed");

// embassy_time::Timer::after(Duration::from_millis(250)).await;
    }
}

#[embassy_executor::task]
async fn display_task(
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    mut fb: &'static mut FBType,
) {
    info!("Starting display");
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X13_BOLD)
        .text_color(Color::YELLOW)
        .background_color(Color::BLACK)
        .build();
    let point = Point::new(32, 15);

    loop {
        // fb.erase();
        Text::with_alignment("Hello, World!", point, text_style, Alignment::Center)
            .draw(fb)
            .expect("failed to draw text");
        tx.signal(fb);
        fb = rx.wait().await;
    }
}
