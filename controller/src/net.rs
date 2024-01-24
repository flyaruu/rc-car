use esp_wifi::esp_now::{EspNowReceiver, EspNowSender, BROADCAST_ADDRESS};
use log::{error, info};
use protocol::{Message, MessagePublisher, MessageSubscriber};

#[embassy_executor::task]
pub async fn sender( mut esp_sender: EspNowSender<'static>, mut subscriber: MessageSubscriber) {
    info!("Starting sender...");
    loop {
        let message = subscriber.next_message_pure().await;
        match message {
            Message::Control(_) => esp_sender.send_async(&BROADCAST_ADDRESS, &message.to_bytes().unwrap()).await.unwrap(),
            Message::Telemetry(_) => {},
        }
    }
}


#[embassy_executor::task]
pub async fn receiver(mut esp_receiver: EspNowReceiver<'static>, publisher: MessagePublisher)->! {
    loop {
        let msg = esp_receiver.receive_async().await;
        let _sender = msg.info.src_address;
        let msg = Message::from_slice(&msg.data);
        match msg {
            Ok(msg) => {
                publisher.publish(msg).await;
            },
            Err(e) => {
                error!("Problem: {:?}",e);
            },
        }
    }
}