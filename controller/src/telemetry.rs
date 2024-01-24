use embassy_futures::select::{select, self};
use embassy_sync::{signal::Signal, blocking_mutex::raw::NoopRawMutex};
use embassy_time::Timer;
use embedded_hal::digital::OutputPin;

use hal::{Rtc, gpio::{Gpio3, Output, PushPull}};
use log::info;
use protocol::{Message, MessageSubscriber, TelemetryMessage};


#[embassy_executor::task]
pub async fn connection_state(signal: &'static Signal<NoopRawMutex,u64>, rtc: &'static Rtc<'_>, mut status_pin: Gpio3<Output<PushPull>>) {
    info!("Connection state started...");
    let mut last_timestamp = 0_u64;
    loop {
        let update_message = select(signal.wait(),Timer::after_millis(1000)).await;
        match update_message {
            select::Either::First(last_message_timestamp) => {
                last_timestamp = last_message_timestamp;
            },
            select::Either::Second(_) => {},
        }
        let l = rtc.get_time_ms() - last_timestamp; // time since previous
        if l > 2000 {
            status_pin.set_high().unwrap();
        } else {
            status_pin.set_low().unwrap();
        }
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::task]
pub async fn telemetry_receiver(mut subscriber: MessageSubscriber, rtc: &'static Rtc<'_>, signal: &'static Signal<NoopRawMutex,u64>) {
    info!("Starting receiver...");
    loop {
        let msg = subscriber.next_message_pure().await;
        match msg {
            Message::Control(_) => {},
            Message::Telemetry(telemetry) => {
                match telemetry {
                    TelemetryMessage::Heartbeat(_) => {
                        let now = rtc.get_time_ms();
                        signal.signal(now);
                    },
                    TelemetryMessage::LogMessage(msg) => {
                        info!("Received message: {}",msg);
                    }
                    _ => {}
                }
            },
        }
    }
}
