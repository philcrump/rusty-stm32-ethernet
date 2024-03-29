
// Don't complain about variable naming convention
#![allow(non_snake_case)]

//use defmt::*;
use embassy_net::Stack;
use embassy_net::udp::{UdpSocket, PacketMetadata};
use embassy_net::{IpEndpoint, IpAddress}; // Stack, StackResources
use chrono::{DateTime, NaiveDateTime};

use embassy_stm32::eth::Ethernet;
use embassy_stm32::peripherals::ETH;
use embassy_stm32::eth::generic_smi::GenericSMI;
type Device = Ethernet<'static, ETH, GenericSMI>;

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
struct NTPv4 {
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

pub async fn sntp_request(stack: &Stack<Device>, init_datetime: embassy_stm32::rtc::DateTime) -> embassy_stm32::rtc::DateTime {
    let mut rx_meta = [PacketMetadata::EMPTY; 16];
    let mut rx_buffer = [0; 1024];
    let mut tx_meta = [PacketMetadata::EMPTY; 16];
    let mut tx_buffer = [0; 1024];
    let mut ntp_rx = [0; 48];

    let ntphost = IpEndpoint::new(IpAddress::v4(185, 83, 169, 27), 123);
    let mut ntpsocket = UdpSocket::new(stack, &mut rx_meta, &mut rx_buffer, &mut tx_meta, &mut tx_buffer);
    let _ = ntpsocket.bind(123); // Bind is required before sending on that port.
    
    let now_datetime: NaiveDateTime = init_datetime.into();
    let mut new_timestamp_s: u32 = (now_datetime.and_utc().timestamp()).try_into().unwrap(); // u64->u32
    let mut new_timestamp_ns: u32 = now_datetime.and_utc().timestamp_subsec_nanos();
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

    //info!("Sending packet.");
    let _ = ntpsocket.send_to(&ntp_tx.to_bytes(), ntphost).await;

    //info!("Sent packet, awaiting response....");
    let (_bytes_count, _socketAddress) = ntpsocket.recv_from(&mut ntp_rx).await.unwrap();

    //info!("Received packet.");

    let ntp_rx2 = NTPv4::from_bytes(&ntp_rx);

    let srv_tx_timestamp_s:u32 = u32::from_be(ntp_rx2.txTm_s) - NTP_TIMESTAMP_DELTA;
    let srv_tx_timestamp_f:u32 = u32::from_be(ntp_rx2.txTm_f) / NTP_NS_TO_FS as u32;

    return DateTime::from_timestamp(srv_tx_timestamp_s as i64, srv_tx_timestamp_f).unwrap().naive_utc().try_into().unwrap();
}