#![no_std]
#![no_main]

// Don't complain about variable naming convention
#![allow(non_snake_case)]

// Picoserve
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
//use embassy_net::tcp::TcpSocket;
//use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{Stack, StackResources, DhcpConfig};
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::peripherals::ETH;
//use embassy_stm32::rng::Rng;
use embassy_stm32::time::Hertz;
use embassy_stm32::adc::{Adc, SampleTime::Cycles480};
use embassy_stm32::gpio::{Input, Level, Output, Speed, AnyPin, Pull};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::Peripheral;
use embassy_stm32::{bind_interrupts, eth, Config}; // rng, peripherals
use embassy_time::{Delay, Timer, Instant};
//use embedded_io_async::Write;
//use rand_core::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// Picoserve
use static_cell::make_static;
//use cortex_m::register::control::Control;
use embassy_time::Duration;
//use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use picoserve::{
    //response::DebugValue,
    routing::{ get }, //parse_path_segment
};
//use picoserve::extract::State;

// Time
use embassy_stm32::rtc::{Rtc, RtcConfig};
use chrono::{NaiveDate, NaiveDateTime, DateTime};
//use heapless::String;

// NTP
use embassy_net::udp::{UdpSocket, PacketMetadata};
use embassy_net::{IpEndpoint, IpAddress}; // Stack, StackResources

macro_rules! impl_bytes {
    ($t:ident) => {
        impl $t {
            /// Bytes consumed by this type.
            pub const SIZE: usize = core::mem::size_of::<Self>();

            /// Convert to byte array.
            #[allow(unused)]
            pub fn to_bytes(&self) -> [u8; Self::SIZE] {
                unsafe { core::mem::transmute(*self) }
            }

            /// Create from byte array.
            #[allow(unused)]
            pub fn from_bytes(bytes: &[u8; Self::SIZE]) -> &Self {
                let alignment = core::mem::align_of::<Self>();
                unsafe { core::mem::transmute(bytes) }
            }

            /// Create from mutable byte array.
            #[allow(unused)]
            pub fn from_bytes_mut(bytes: &mut [u8; Self::SIZE]) -> &mut Self {
                let alignment = core::mem::align_of::<Self>();

                unsafe { core::mem::transmute(bytes) }
            }
        }
    };
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(C)]
pub struct NTPv4 {
    pub version: u8,
    pub stratum: u8,
    pub poll: u8,
    pub precision: i8,

    pub rootDelay_s: u16,
    pub rootDelay_f: u16,

    pub rootDispersion_s: u16,
    pub rootDispersion_f: u16,

    pub refId: u32,

    pub refTm_s: u32,
    pub refTm_f: u32,

    pub origTm_s: u32,
    pub origTm_f: u32,

    pub rxTm_s: u32,
    pub rxTm_f: u32,

    pub txTm_s: u32,
    pub txTm_f: u32,
}
impl_bytes!(NTPv4);

const SHORTNTPFS_TO_MS: f32 = 1000.0 / 65536.0;

const NTPFS_TO_MS: f32 =   1000.0 / 4294967296.0;
const NTPFS_TO_US: f32 =   1000000.0 / 4294967296.0;
const NTP_NS_TO_FS: f32 =  4294967296.0 / 1000000000.0 ;
const DIFF_SEC_1970_2036: u32 = 2085978496;
const NTP_TIMESTAMP_DELTA: u32 = 2208988800;

//GPIO
use core::sync::atomic::{ AtomicBool, AtomicU16, AtomicU32, Ordering };

static button_pressed: AtomicBool = AtomicBool::new(false);
static temperature_mcu: AtomicU16 = AtomicU16::new(0);
static timestamp_u32: AtomicU32 = AtomicU32::new(0);


bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
//    HASH_RNG => rng::InterruptHandler<peripherals::RNG>; // No HW crypto in F767
});

type Device = Ethernet<'static, ETH, GenericSMI>;

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<Device>) {
    stack.run().await;
}

#[embassy_executor::task]
async fn blink_heartbeat(pin: embassy_stm32::PeripheralRef<'static, AnyPin>) {
    let mut led = Output::new(pin, Level::High, Speed::Low);
    loop {
        led.set_high();
        Timer::after_millis(250).await;
        led.set_low();
        Timer::after_millis(250).await;
    }
}

#[embassy_executor::task]
async fn blink_network(pin: embassy_stm32::PeripheralRef<'static, AnyPin>, stack: &'static Stack<Device>) {
    let mut led = Output::new(pin, Level::High, Speed::Low);
    loop {
        if stack.is_link_up()
        {
            led.set_high();
            if !stack.is_config_up()
            {
                Timer::after_millis(150).await;
                led.set_low();
            }
        }
        else
        {
            led.set_low();
        }
        Timer::after_millis(200).await;
    }
}

#[embassy_executor::task]
async fn rtc_timestamp(rtc: Rtc) {
    let mut now: NaiveDateTime;
    let mut new_timestamp: u32;

    loop {
        now = rtc.now().unwrap().into();
        new_timestamp = (now.and_utc().timestamp()).try_into().unwrap();

        timestamp_u32.store(new_timestamp, Ordering::Relaxed);
        Timer::after_millis(500).await;
    }
}

type AppRouter = impl picoserve::routing::PathRouter;

const WEB_TASK_POOL_SIZE: usize = 8;
const NTP_TASK_POOL_SIZE: usize = 1;
const NET_TASK_POOL_SIZE: usize = WEB_TASK_POOL_SIZE + NTP_TASK_POOL_SIZE + 1; // all above + 1

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
async fn web_task(
    id: usize,
    stack: &'static embassy_net::Stack<Device>,
    app: &'static picoserve::Router<AppRouter>,
    config: &'static picoserve::Config<Duration>,
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::listen_and_serve(
        id,
        app,
        config,
        stack,
        port,
        &mut tcp_rx_buffer,
        &mut tcp_tx_buffer,
        &mut http_buffer
    )
    .await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Bypass, // Nucleo feeds main MCU from Debug MCU
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL216,
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 216 / 2 = 216Mhz
            divq: None,
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ls = LsConfig {
            rtc: RtcClockSource::LSE,
            lsi: false,
            lse: Some(LseConfig {
                frequency: Hertz(32_768),
                mode: LseMode::Oscillator(LseDrive::MediumHigh),
            })
        };
    }
    let p = embassy_stm32::init(config);

    info!("Hello World!");

    // Set up watchdog
    let mut watchdog = embassy_stm32::wdg::IndependentWatchdog::new(p.IWDG, 30*1000*1000); // 30 seconds, to allow bootup
    // Start it
    watchdog.unleash();
    watchdog.pet();

    // Heartbeat LED Blinker
    spawner.spawn(blink_heartbeat(AnyPin::from(p.PB0).into_ref())).unwrap();

    // RTC
    let mut rtc = Rtc::new(p.RTC, RtcConfig::default());

    // Set Initial Time
    let now = NaiveDate::from_ymd_opt(2024, 3, 1)
        .unwrap()
        .and_hms_opt(0, 0, 0)
        .unwrap();
    rtc.set_datetime(now.into()).expect("datetime not set");

    // Button Input
    let button_input = Input::new(p.PC13, Pull::Down);
    let button = ExtiInput::new(button_input, p.EXTI13);

    // MCU Temperature Sensor
    let mut adc = Adc::new(p.ADC1, &mut Delay);
    adc.set_sample_time(Cycles480);
    let mut mcu_temp = adc.enable_temperature();

    // Generate random seed.
    //let mut rng = Rng::new(p.RNG, Irqs);
    let seed = [0; 8];
    //rng.fill_bytes(&mut seed);
    let seed = u64::from_le_bytes(seed);

    let mac_addr = [0x00, 0x00, 0xDE, 0xAD, 0xBE, 0xEF];

    static PACKETS: StaticCell<PacketQueue<16, 16>> = StaticCell::new();
    let device = Ethernet::new(
        PACKETS.init(PacketQueue::<16, 16>::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA2,
        p.PC1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        p.PB13,
        p.PG11,
        GenericSMI::new(0),
        mac_addr,
    );

    let mut dhcp_config = DhcpConfig::default();
    dhcp_config.hostname = Some(heapless::String::try_from("stm32f7-rusty").unwrap());
    let net_config = embassy_net::Config::dhcpv4(dhcp_config);

    //let net_config = embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
    //    address: Ipv4Cidr::new(Ipv4Address::new(10, 42, 0, 61), 24),
    //    dns_servers: Vec::new(),
    //    gateway: Some(Ipv4Address::new(10, 42, 0, 1)),
    //});

    // Init network stack
    static STACK: StaticCell<Stack<Device>> = StaticCell::new();
    static RESOURCES: StaticCell<StackResources<NET_TASK_POOL_SIZE>> = StaticCell::new();
    let stack = &*STACK.init(Stack::new(
        device,
        net_config,
        RESOURCES.init(StackResources::<NET_TASK_POOL_SIZE>::new()),
        seed,
    ));

    // Launch network task
    spawner.spawn(net_task(stack)).unwrap();

    spawner.spawn(blink_network(AnyPin::from(p.PB7).into_ref(), stack)).unwrap();

    info!("Network initialised..");

    // Ensure DHCP configuration is up before setting up server
    stack.wait_config_up().await;

    info!("Network is up!");

    fn make_app() -> picoserve::Router<AppRouter> {
        picoserve::Router::new()
            .route(
                "/",
                get(|| picoserve::response::File::with_content_type_and_headers(
                    "text/html; charset=utf-8",
                    include_bytes!("htdist/index.html.gz"),
                    &[("Content-Encoding", "gzip")],
                )),
            )
            .route(
                "/index.css",
                get(|| picoserve::response::File::with_content_type_and_headers(
                    "text/css; charset=utf-8",
                    include_bytes!("htdist/index.css.gz"),
                    &[("Content-Encoding", "gzip")],
                )),
            )
            .route(
                "/index.js",
                get(|| picoserve::response::File::with_content_type_and_headers(
                    "application/javascript; charset=utf-8",
                    include_bytes!("htdist/index.js.gz"),
                    &[("Content-Encoding", "gzip")],
                )),
            )
            .route(
                "/mithril-2.2.2.min.js",
                get(|| picoserve::response::File::with_content_type_and_headers(
                    "application/javascript; charset=utf-8",
                    include_bytes!("htdist/mithril-2.2.2.min.js.gz"),
                    &[("Content-Encoding", "gzip")],
                )),
            )
            .route(
                "/api/state",
                get(
                    || async move {
                        picoserve::response::Json(
                            ( 
                                ( "uptime_s", Instant::now().as_secs() ),
                                ( "button_state", button_pressed.load(Ordering::Relaxed) ),
                                ( "temp_c", temperature_mcu.load(Ordering::Relaxed) ),
                                ( "device_timestamp", timestamp_u32.load(Ordering::Relaxed) ),
                            )
                        )
                    }
                ),
            )
    }

    let app = make_static!(make_app());

    let config = make_static!(picoserve::Config::new(picoserve::Timeouts {
        start_read_request: Some(Duration::from_secs(5)),
        read_request: Some(Duration::from_secs(1)),
        write: Some(Duration::from_secs(1)),
    })
    .keep_connection_alive());

    info!("HTTP server initialised..");

    for id in 0..WEB_TASK_POOL_SIZE {
        spawner.must_spawn(web_task(
            id,
            stack,
            app,
            config
        ));
    }

    info!("HTTP server up!");

    // SNTP
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; 1024];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; 1024];
    let mut ntp_rx = [0; 48];

    let ntphost = IpEndpoint::new(IpAddress::v4(185, 83, 169, 27), 123);
    let mut ntpsocket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    let _ = ntpsocket.bind(123); // Bind is required before sending on that port.
    
    let current_datetime: NaiveDateTime = rtc.now().unwrap().into();
    let mut new_timestamp_s: u32 = (current_datetime.and_utc().timestamp()).try_into().unwrap(); // u64->u32
    let mut new_timestamp_ns: u32 = current_datetime.and_utc().timestamp_subsec_nanos();
    new_timestamp_s -= DIFF_SEC_1970_2036;
    new_timestamp_ns = (new_timestamp_ns as f32 * NTP_NS_TO_FS) as u32;

    let ntp_tx = NTPv4 {
        version: 0x1b,
        // To be filled in by server
        stratum: 0,
        poll: 0,
        precision: 0,
        rootDelay_s: 0,
        rootDelay_f: 0,
        rootDispersion_s: 0,
        rootDispersion_f: 0,
        refId: 0,
        refTm_s: 0,
        refTm_f: 0,
        origTm_s: 0,
        origTm_f: 0,
        rxTm_s: 0,
        rxTm_f: 0,
        // Initial Timestamp
        txTm_s: new_timestamp_s.to_be(),
        txTm_f: new_timestamp_ns.to_be(),
    };

    info!("Sending packet.");
    let _ = ntpsocket.send_to(&ntp_tx.to_bytes(), ntphost).await;

    info!("Sent packet, awaiting response....");
    let (_bytes_count, _socketAddress) = ntpsocket.recv_from(&mut ntp_rx).await.unwrap();

    info!("Received packet.");

    let ntp_rx2 = NTPv4::from_bytes(&ntp_rx);

    let srv_tx_timestamp_s:u32 = u32::from_be(ntp_rx2.txTm_s) - NTP_TIMESTAMP_DELTA;
    let srv_tx_timestamp_f:u32 = u32::from_be(ntp_rx2.txTm_f) / NTP_NS_TO_FS as u32;

    let new_datetime = DateTime::from_timestamp(srv_tx_timestamp_s as i64, srv_tx_timestamp_f).unwrap();
    rtc.set_datetime(new_datetime.naive_utc().into()).expect("datetime not set");

    // *After* NTP is done with RTC, as rust is 'moving' the ownership
    spawner.spawn(rtc_timestamp(rtc)).unwrap();

    info!("Started RTC Thread");

    // STM32H7 Cal pointers
    //let ts_cal1_ptr = 0x1FF1E820 as *const u16;
    //let ts_cal2_ptr = 0x1FF1E840 as *const u16;

    // STM32F7 Cal pointers
    let ts_cal1_ptr = 0x1FF0F44C as *const u16;
    let ts_cal2_ptr = 0x1FF0F44E as *const u16;

    let ts_cal1: u16 = unsafe { *ts_cal1_ptr };
    let ts_cal2: u16 = unsafe { *ts_cal2_ptr };

    loop {
        watchdog.pet();

        let new_temperature_mcu = ((110.0-30.0) / f32::from(ts_cal2 - ts_cal1)) * f32::from(adc.read(&mut mcu_temp) - ts_cal1) + 30.0;
        temperature_mcu.store(new_temperature_mcu as u16, Ordering::Relaxed);

        info!("{:?}", new_temperature_mcu);

        if button.is_low()
        {
            let _ = button_pressed.compare_exchange(true, false, Ordering::Relaxed, Ordering::Relaxed);
        }
        else {
            let _ = button_pressed.compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed);
        }

        Timer::after_millis(100).await;

        // Check if button got pressed
        //button.wait_for_rising_edge().await;
        //button_pressed.store(true, Ordering::Relaxed);

        //button.wait_for_falling_edge().await;
        //button_pressed.store(false, Ordering::Relaxed);
    }
}
