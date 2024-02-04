use embassy_executor::{task, Spawner};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, signal::Signal};
use embassy_time::Timer;
use embedded_hal_async::digital::Wait;
use hal::Rtc;
use protocol::MessagePublisher;
use static_cell::make_static;

use crate::types::TachPin;

#[task]
pub async fn tach(spawner: Spawner, publisher: MessagePublisher, tach_pin: TachPin, rtc: &'static Rtc<'static>) {
    let odo_signal = make_static!(Signal::new());
    let rpm_signal = make_static!(Signal::new());
    spawner.spawn(revolution(tach_pin, rtc,odo_signal, rpm_signal)).unwrap();
    spawner.spawn(tach_publisher(publisher, odo_signal, rpm_signal)).unwrap();
}

#[task]
pub async fn revolution(mut tach_pin: TachPin, rtc: &'static Rtc<'static>, odo_signal: &'static Signal<NoopRawMutex,u64>, rpm_signal: &'static Signal<NoopRawMutex,u64>)->! {
    let mut odo = 0_u64;
    let mut last_pulse = rtc.get_time_us();
    loop {
        tach_pin.wait_for_rising_edge().await.unwrap();
        odo+=1;
        let now = rtc.get_time_us();
        let elapsed = now - last_pulse;
        let rpm = 60_000_000 / elapsed;
        odo_signal.signal(odo);
        rpm_signal.signal(rpm);
        last_pulse = now;

    }
}

#[task]
pub async fn tach_publisher(publisher: MessagePublisher, odo_signal: &'static Signal<NoopRawMutex,u64>, rpm_signal: &'static Signal<NoopRawMutex,u64>)->! {
    loop {

        if odo_signal.signaled() {
            let odo = odo_signal.wait().await;
            publisher.publish(protocol::Message::Telemetry(protocol::TelemetryMessage::MotorOdo(odo))).await;
        }
        if rpm_signal.signaled() {
            let rpm = rpm_signal.wait().await;
            publisher.publish(protocol::Message::Telemetry(protocol::TelemetryMessage::MotorRpm(rpm))).await;
        }

        //
        Timer::after_millis(500).await;
    }
}