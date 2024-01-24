
use esp_wifi::esp_now::{EspNowReceiver, EspNowSender, BROADCAST_ADDRESS};
use log::{error, info};
use protocol::{Message, MessagePublisher, MessageSubscriber};

#[embassy_executor::task]
pub async fn sender(mut esp_sender: EspNowSender<'static>, mut subscriber: MessageSubscriber)->! {
    loop {
        let message = subscriber.next_message_pure().await;
        // Only send telemetry
        match message {
            Message::Control(_) => {},
            Message::Telemetry(_) => {
                info!("Sending message from car: {:?}",message);
                match message.to_bytes() {
                    Ok(msg) => {
                        match esp_sender.send_async(&BROADCAST_ADDRESS, &msg).await {
                            Ok(_) => {},
                            Err(e) => {
                                error!("Error sending telemetry: {:?}",e);
                            },  
                        }
                    },
                    Err(e) => error!("Error serializing telemetry: {:?}",e),
                }
        
            },
        }
    }
}

#[embassy_executor::task]
pub async fn receiver(mut esp_receiver: EspNowReceiver<'static>, publisher: MessagePublisher)->! {
    info!("Starting receiver...");
    loop {
        let msg = esp_receiver.receive_async().await;

        let _sender = msg.info.src_address;
        let msg = Message::from_slice(&msg.data);
        info!("ESPNOW Reeived: {:?}",msg);
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