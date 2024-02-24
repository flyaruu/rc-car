use embassy_executor::{task, Spawner};
use embassy_futures::select::{select, Either};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::{PubSubChannel, Publisher, Subscriber}, signal::Signal};
use embassy_time::Timer;
use embedded_hal_async::digital::Wait;
use hal::Rtc;
use log::info;
use protocol::MessagePublisher;
use static_cell::make_static;

use crate::types::TachPin;

#[task]
pub async fn tach(spawner: Spawner, publisher: MessagePublisher, tach_pin: TachPin, rtc: &'static Rtc<'static>) {
    let odo_signal = make_static!(Signal::new());
    let rpm_signal = make_static!(Signal::new());
    // let rpm_channel: PubSubChannel<NoopRawMutex,u64,10,4,4> = PubSubChannel::new();
    // let rpm_channel = make_static!(rpm_channel);
    spawner.spawn(revolution(tach_pin, rtc,odo_signal, rpm_signal)).unwrap();
    spawner.spawn(tach_publisher(publisher, odo_signal, rpm_signal, rtc)).unwrap();
}

const MAX_WAIT: u64 = 50;
#[task]
pub async fn revolution(mut tach_pin: TachPin, rtc: &'static Rtc<'static>, odo_signal: &'static Signal<NoopRawMutex,u64>, rpm_signal: &'static Signal<NoopRawMutex,u64>)->! {
    let mut odo = 0_u64;
    let mut last_pulse = rtc.get_time_us();
    loop {
        let event = tach_pin.wait_for_rising_edge().await;
        // match event {
                let now = rtc.get_time_us();
                let elapsed = now - last_pulse;
                let rpm = 60_000_000 / elapsed;
                // info!("Pulse delay: {} rpm: {}",elapsed, rpm);
                odo_signal.signal(odo);
                info!("\nSignaling rpm: {rpm} {elapsed}");
                rpm_signal.signal(rpm);
                // rpm_publisher.publish(rpm).await;
                last_pulse = now;
            // },
            // Either::Second(_) => {
            //     // info!("Timeout for rpm");
            //     let now = rtc.get_time_us();
            //     let elapsed = now - last_pulse;
            //     let rpm = 60_000_000 / elapsed; // Don't like the c/p
            //     // odo_signal.signal(odo);
            //     // rpm_signal.signal(rpm);
            //     rpm_publisher.publish(rpm).await;

            // },
        // }
        // tach_pin.wait_for_rising_edge().await.unwrap();

    }
}

#[task]
pub async fn tach_publisher(publisher: MessagePublisher, odo_signal: &'static Signal<NoopRawMutex,u64>, rpm_signal: &'static Signal<NoopRawMutex,u64>, rtc: &'static Rtc<'static>)->! {
    info!("Tech publisher started");
    let mut last_rpm = 0_u64;
    let mut last_signal = rtc.get_time_us();
    loop {

        if odo_signal.signaled() {
            let odo = odo_signal.wait().await;
            publisher.publish(protocol::Message::Telemetry(protocol::TelemetryMessage::MotorOdo(odo))).await;
        }
        if rpm_signal.signaled() {
            let rpm = rpm_signal.wait().await;
            info!("Signaled rpm: {}",rpm);
            rpm_signal.reset();
            publisher.publish(protocol::Message::Telemetry(protocol::TelemetryMessage::MotorRpm(rpm))).await;
            last_signal = rtc.get_time_us();
        }
        if rtc.get_time_us() - last_signal > 1000000 {
            publisher.publish(protocol::Message::Telemetry(protocol::TelemetryMessage::MotorRpm(0))).await;
            last_signal = rtc.get_time_us();
        }

        // if rpm_subscriber.available() > 0 {
        //     // info!("available");
        //     last_rpm = rpm_subscriber.next_message_pure().await;
        //     // info!("Received Signaled rpm: {}",rpm);
        //     publisher.publish(protocol::Message::Telemetry(protocol::TelemetryMessage::MotorRpm(last_rpm))).await;

        // // } else {
        //     // info!("empty");
        // }
        //
        Timer::after_millis(250).await;
    }
}