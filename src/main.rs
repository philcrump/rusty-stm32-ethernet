#![no_std]
#![no_main]

// Picoserve
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
//use embassy_net::tcp::TcpSocket;
//use embassy_net::tcp::client::{TcpClient, TcpClientState};
use embassy_net::{Stack, StackResources, DhcpConfig, IpEndpoint, IpAddress};
use embassy_stm32::eth::generic_smi::GenericSMI;
use embassy_stm32::eth::{Ethernet, PacketQueue};
use embassy_stm32::peripherals::ETH;
//use embassy_stm32::rng::Rng;
use embassy_stm32::time::Hertz;
use embassy_stm32::adc::{Adc, SampleTime::Cycles480};
use embassy_stm32::gpio::{Input, Level, Output, Speed, AnyPin, Pull};
//use embassy_stm32::exti::ExtiInput;
use embassy_stm32::Peripheral;
use embassy_stm32::{bind_interrupts, eth, Config}; // rng, peripherals
use embassy_time::{Delay, Timer, Instant};
//use embedded_io_async::Write;
//use rand_core::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};


// Ethernet
bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler;
//    HASH_RNG => rng::InterruptHandler<peripherals::RNG>; // No HW crypto in F767
});
type Device = Ethernet<'static, ETH, GenericSMI>;


// Picoserve
use static_cell::make_static;
//use cortex_m::register::control::Control;
use embassy_time::Duration;
//use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use picoserve::{
    //response::DebugValue,
    routing::{ get }, //parse_path_segment
};

#[derive(serde::Deserialize)]
struct PostFormBoolState {
    state: heapless::String<16>,
}


// Time
use embassy_stm32::rtc::{Rtc, RtcConfig};
use chrono::{NaiveDate, NaiveDateTime};
//use heapless::String;


// NTP
mod sntp;


// SNMP
use embassy_net::udp::{UdpSocket, PacketMetadata};
use nom::{IResult, Err, Needed};
use nom::error::{Error, ErrorKind};
use nom::combinator::{flat_map, map, map_res};
use nom::number::streaming::{be_f64, be_i16, be_i24, be_u16, be_u24, be_u32, be_u8};
use nom::sequence::{pair, terminated, tuple};
use nom::multi::{length_data, fill};

// GPIO
use core::sync::atomic::{ AtomicU16, Ordering };

struct AppMonitor {
    temperature_mcu: AtomicU16
}
static APP_VALUES: AppMonitor = AppMonitor {
    temperature_mcu: AtomicU16::new(0)
};


// State Sync
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use picoserve::extract::State;

#[derive(Clone, Copy)]
struct RtcSharedControl(&'static Mutex<CriticalSectionRawMutex, Rtc>);
#[derive(Clone, Copy)]
struct InputSharedControl(&'static Mutex<CriticalSectionRawMutex, Input<'static, AnyPin>>);
#[derive(Clone, Copy)]
struct OutputSharedControl(&'static Mutex<CriticalSectionRawMutex, Output<'static, AnyPin>>);

#[derive(Clone, Copy)]
struct AppControl {
    rtc_control: RtcSharedControl,
    testbutton_control: InputSharedControl,
    redled_control: OutputSharedControl,
}
// This is what is shared on picoserve
struct PicoserveAppControl {
    shared_control: AppControl,
}
impl picoserve::extract::FromRef<PicoserveAppControl> for AppControl {
    fn from_ref(state: &PicoserveAppControl) -> Self {
        state.shared_control
    }
}


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
async fn adc1_task(adc_instance: embassy_stm32::peripherals::ADC1) {

    // STM32H7 ADC Cal pointers
    //let ts_cal1_ptr = 0x1FF1E820 as *const u16;
    //let ts_cal2_ptr = 0x1FF1E840 as *const u16;

    // STM32F7 ADC Cal pointers
    let ts_cal1_ptr = 0x1FF0F44C as *const u16;
    let ts_cal2_ptr = 0x1FF0F44E as *const u16;

    let adc_ts_cal1 = unsafe { *ts_cal1_ptr };
    let adc_ts_cal2 = unsafe { *ts_cal2_ptr };

    // MCU Temperature Sensor
    let mut adc = Adc::new(adc_instance, &mut Delay);
    adc.set_sample_time(Cycles480);
    let mut mcu_temp = adc.enable_temperature();

    let mut new_temperature_mcu: f32;

    loop {
        new_temperature_mcu = ((110.0-30.0) / f32::from(adc_ts_cal2 - adc_ts_cal1)) * f32::from(adc.read(&mut mcu_temp) - adc_ts_cal1) + 30.0;

        APP_VALUES.temperature_mcu.store(new_temperature_mcu as u16, Ordering::Relaxed); 

        //info!("MCU Temp: {:?}", new_temperature_mcu);

        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn sntp_task(stack: &'static Stack<Device>, app_control: AppControl) {
    // SNTP - eshail.batc.org.uk server
    let ntphost = IpEndpoint::new(IpAddress::v4(185, 83, 169, 27), 123);
    loop {
        let current_datetime = app_control.rtc_control.0.lock().await.now().unwrap();

        let new_datetime = sntp::sntp_request(stack, ntphost, current_datetime).await;
        if new_datetime.is_some()
        {
            let _ = app_control.rtc_control.0.lock().await.set_datetime(new_datetime.unwrap());
            info!("Time Synchronised.");
        }
        else {
            warn!("Failed to synchronise time.");
        }

        Timer::after_secs(3600).await;
    }
}

fn print_field(field: &TlvTag)
{
    if field.tlvtype == 0x02
    {
        if field.tlvdata.len() == 1
        {
            info!("Integer: {} (d)", field.tlvdata);
        }
        else if field.tlvdata.len() == 2
        {
            info!("Integer: {} (d)", u16::from_be_bytes(field.tlvdata.try_into().unwrap()));
        }
        else if field.tlvdata.len() == 4
        {
            info!("Integer: {} (d)", u32::from_be_bytes(field.tlvdata.try_into().unwrap()));
        }
    }
    else if field.tlvtype == 0x04
    {
        if let Ok(s) = core::str::from_utf8(&field.tlvdata) {
            info!("String: {}", s);
        }
        else {
            info!("Buffer: {}", field.tlvdata);
        }
    }
    else if field.tlvtype == 0x06
    {
        info!("OID: {:02x} (x)", field.tlvdata);
    }
    else if field.tlvtype == 0x30
    {
        info!("ASN Sequence: {:02x} (x)", field.tlvdata);
    }
    else if field.tlvtype == 0xa0
    {
        info!("GetRequest_PDU_TAG: {:02x} (x)", field.tlvdata);
    }
    else
    {
        info!("<unknown> type: {:02x} (x), data: {:02x} (x)", field.tlvtype, field.tlvdata);
    }
}

#[macro_export]
macro_rules! TestOrBreak {
    ( $x:expr ) => {
        if ($x) == false {
            break;
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SnmpRequest<'a> {
  pub version: u8,
  pub tlvdata: &'a [u8],
}

#[embassy_executor::task]
async fn snmp_task(stack: &'static Stack<Device>, app_control: AppControl) {
    let mut rx_buffer = [0; 512];
    let mut tx_buffer = [0; 512];
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut buf = [0; 512];
    loop {
        let mut socket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
        socket.bind(161).unwrap();

        loop {
            let (n, ep) = socket.recv_from(&mut buf).await.unwrap();


            if let Ok(s) = core::str::from_utf8(&buf[..n]) {
                info!("rxd from {}: {}", ep, s);
            }
            else {
                info!("rxd from {} (Undecodable)", ep);
            }

            info!("buf length: {} octets", n);

            let (mut nextbuf, mut r) = tlv(&buf).unwrap();

            info!("Top level ASN Sequence:");
            print_field(&r);

            // Sequence: Table-2 in https://www.ranecommercial.com/legacy/pdf/ranenotes/SNMP_Simple_Network_Management_Protocol.pdf


            /* SNMP Version Integer */
            (nextbuf, r) = tlv(r.tlvdata).unwrap();
            print_field(&r);

            // Sanity Checks
            TestOrBreak!(r.tlvtype == 0x02);
            TestOrBreak!(r.tlvdata.len() == 1);

            let snmp_version = r.tlvdata[0];


            /* SNMP Community String */
            (nextbuf, r) = tlv(nextbuf).unwrap();
            print_field(&r);

            // Sanity Checks
            TestOrBreak!(r.tlvtype == 0x04);
            TestOrBreak!(r.tlvdata.len() > 1);

            if let Ok(snmp_community) = core::str::from_utf8(&r.tlvdata) {
                info!("Community: {}", snmp_community);
            }
            else {
                break;
            }


            /* SNMP PDU */
            (nextbuf, r) = tlv(nextbuf).unwrap();
            print_field(&r);

            // Sanity Checks
            TestOrBreak!(r.tlvtype == 0xa0 || r.tlvtype == 0xa3); // 0xa0 = GetRequest, 0xa3 = SetRequest
            TestOrBreak!(r.tlvdata.len() >= 4);

                /* Request ID */
                (nextbuf, r) = tlv(r.tlvdata).unwrap();
                print_field(&r);

                // Sanity Checks
                TestOrBreak!(r.tlvtype == 0x02);
                TestOrBreak!(r.tlvdata.len() == 4);

                let request_id = u32::from_be_bytes(r.tlvdata.try_into().unwrap());


                /* Error */
                (nextbuf, r) = tlv(nextbuf).unwrap();
                print_field(&r);

                // Sanity Checks
                TestOrBreak!(r.tlvtype == 0x02);
                TestOrBreak!(r.tlvdata.len() == 1);

                let request_error = r.tlvdata[0];


                /* Error Index */
                (nextbuf, r) = tlv(nextbuf).unwrap();
                print_field(&r);

                // Sanity Checks
                TestOrBreak!(r.tlvtype == 0x02);
                TestOrBreak!(r.tlvdata.len() == 1);

                let request_error_index = r.tlvdata[0];


                /* Varbind List Sequence */
                (nextbuf, r) = tlv(nextbuf).unwrap();
                print_field(&r);

                // Sanity Checks
                TestOrBreak!(r.tlvtype == 0x30);
                TestOrBreak!(r.tlvdata.len() >= 4);


                    /* OID */
                    (nextbuf, r) = tlv(r.tlvdata).unwrap();
                    print_field(&r);

                    // Sanity Checks
                    TestOrBreak!(r.tlvtype == 0x30);
                    TestOrBreak!(r.tlvdata.len() >= 2);

                        /* OID */
                        (nextbuf, r) = tlv(r.tlvdata).unwrap();
                        print_field(&r);

                        // Sanity Checks
                        TestOrBreak!(r.tlvtype == 0x06);
                        TestOrBreak!(r.tlvdata.len() >= 1);

                        // OID Encoding: https://learn.microsoft.com/en-gb/windows/win32/seccertenroll/about-object-identifier?redirectedfrom=MSDN

                        info!("Decode complete!");

                        /* Value (NULL TAG) */
                        //(nextbuf, r) = tlvnull(nextbuf).unwrap();
                        //print_field(&r);

                        // Sanity Checks
                        //TestOrBreak!(r.tlvtype != 0x05);
                        //TestOrBreak!(r.tlvdata.len() < 2);

            
            socket.send_to(&buf[..n], ep).await.unwrap();
        }
    }
}

// Testing: snmpget -v2c -c public 192.168.110.102 iso.3.6.1


// SNMP parsing: https://stackoverflow.com/a/23007993
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TagType {
  Integer,
  String,
  OID,
  List,
}

fn tag_type(input: &[u8]) -> IResult<&[u8], TagType> {
  map_res(be_u8, |tag_type| {
    Ok::<TagType, _>(match tag_type {
      0x02 => TagType::Integer,
      0x04 => TagType::String,
      0x06 => TagType::OID,
      0x30 => TagType::List,
      _ => return Err(Err::Error(Error::new(input, ErrorKind::Alt))),
    })
  })(input)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TlvTag<'a> {
  pub tlvtype: u8,
  pub tlvdata: &'a [u8],
}


pub fn tlv(input: &[u8]) -> IResult<&[u8], TlvTag> {
  map(
      tuple((
        be_u8,
        length_data(be_u8),
      )),
      move |(tag_type, data)| TlvTag {
        tlvtype: tag_type,
        tlvdata: data,
      },
    )(input)
}

/*
pub fn tlvnull(input: &[u8]) -> IResult<&[u8], TlvTag> {
  map(
      tuple((
        be_u8,
        be_u8,
      )),
      move |(tag_type, data)| TlvTag {
        tlvtype: tag_type,
        tlvdata: [data],
      },
    )(input)
}
*/

/*
fn req_parser(input: &[u8]) -> IResult<&[u8], [TLV_Tag; 2]> {
  let mut buf = [TLV_Tag { }, TLV_Tag { }];
  let (rest, ()) = fill(tlv, &mut buf)(input)?;
  Ok((rest, buf))
}
*/

type AppRouter = impl picoserve::routing::PathRouter<PicoserveAppControl>;

const WEB_TASK_POOL_SIZE: usize = 8;
const NTP_TASK_POOL_SIZE: usize = 1;
const SNMP_TASK_POOL_SIZE: usize = 1;
const NET_TASK_POOL_SIZE: usize = WEB_TASK_POOL_SIZE + NTP_TASK_POOL_SIZE + SNMP_TASK_POOL_SIZE + 1; // all above + 1

#[embassy_executor::task(pool_size = WEB_TASK_POOL_SIZE)]
async fn web_task(
    id: usize,
    stack: &'static embassy_net::Stack<Device>,
    app: &'static picoserve::Router<AppRouter, PicoserveAppControl>,
    config: &'static picoserve::Config<Duration>,
    state: PicoserveAppControl
) -> ! {
    let port = 80;
    let mut tcp_rx_buffer = [0; 1024];
    let mut tcp_tx_buffer = [0; 1024];
    let mut http_buffer = [0; 2048];

    picoserve::listen_and_serve_with_state(
        id,
        app,
        config,
        stack,
        port,
        &mut tcp_rx_buffer,
        &mut tcp_tx_buffer,
        &mut http_buffer,
        &state,
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
            divp: Some(PllPDiv::DIV2), // 8mhz / 4 * 216 / 2 = 216MHz, F767ZI top speed.
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

    // Set up watchdog
    let mut watchdog = embassy_stm32::wdg::IndependentWatchdog::new(p.IWDG, 30*1000*1000); // 30 seconds, to allow bootup
    // Start watchdog
    watchdog.unleash();
    watchdog.pet();

    info!("Hello World!");

    // Heartbeat LED Blinker
    spawner.spawn(blink_heartbeat(AnyPin::from(p.PB0).into_ref())).unwrap();

    spawner.spawn(adc1_task(p.ADC1)).unwrap();
    info!("MCU Temperature task started.");

    // RTC
    let mut rtc: Rtc = Rtc::new(p.RTC, RtcConfig::default());

    // Set Initial Time
    let now = NaiveDate::from_ymd_opt(2024, 3, 1)
        .unwrap()
        .and_hms_opt(0, 0, 0)
        .unwrap();
    rtc.set_datetime(now.into()).expect("datetime not set");
    
    let app_control = AppControl {
        rtc_control: RtcSharedControl(make_static!(Mutex::new(rtc))),
        testbutton_control: InputSharedControl(make_static!(Mutex::new(Input::new(AnyPin::from(p.PC13), Pull::Down)))),
        redled_control: OutputSharedControl(make_static!(Mutex::new(Output::new(AnyPin::from(p.PB14), Level::Low, Speed::Low)))),
    };

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

    fn make_app() -> picoserve::Router<AppRouter, PicoserveAppControl> {
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
                    |State(shared_control): State<AppControl>| async move {
                        let now: NaiveDateTime;
                        let new_timestamp: u32;

                        now = shared_control.rtc_control.0.lock().await.now().unwrap().into();
                        new_timestamp = (now.and_utc().timestamp()).try_into().unwrap();

                        let testbutton_state: bool = shared_control.testbutton_control.0.lock().await.is_high();
                        let redled_state: bool = shared_control.redled_control.0.lock().await.is_set_high();

                        picoserve::response::Json(
                            ( 
                                ( "uptime_s", Instant::now().as_secs() ),
                                ( "button_state", testbutton_state ),
                                ( "redled_state", redled_state ),
                                ( "temp_c", APP_VALUES.temperature_mcu.load(Ordering::Relaxed) ),
                                ( "device_timestamp", new_timestamp ),
                            )
                        )
                    }
                ).post(
                    |State(shared_control): State<AppControl>, picoserve::extract::Form(PostFormBoolState { state })| async move {
                        if state == "true"
                        {
                            shared_control.redled_control.0.lock().await.set_high();
                        }
                        else if state == "false"
                        {
                            shared_control.redled_control.0.lock().await.set_low();
                        }
                    }
                )
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

    spawner.spawn(sntp_task(stack, app_control)).unwrap();
    info!("SNTP Task started.");

    spawner.spawn(snmp_task(stack, app_control)).unwrap();
    info!("SNMP Task started.");

    for id in 0..WEB_TASK_POOL_SIZE {
        spawner.must_spawn(web_task(
            id,
            stack,
            app,
            config,
            PicoserveAppControl { shared_control: app_control },
        ));
    }

    info!("HTTP server up!");

    loop {
        watchdog.pet();

        Timer::after_millis(100).await;
    }
}
