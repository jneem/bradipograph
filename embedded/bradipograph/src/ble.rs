use core::cell::RefCell;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
};
use esp32c3_hal::peripherals::BT;
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, EspWifiInitialization};

use bradipo_protocol::{CalibrationStatus, Cmd, State};
use heapless::spsc::{Consumer, Producer};

use crate::GLOBAL;

// TODO: define these in a shared crate
// const UUID: &str = "68a79627-2609-4569-8d7d-3b29fde28877";
// const MANUAL_CONTROL_UUID: &str = "68a79628-2609-4569-8d7d-3b29fde28877";

pub type CmdProducer = Producer<'static, Cmd, 64>;
pub type CmdConsumer = Consumer<'static, Cmd, 64>;

#[embassy_executor::task]
pub async fn ble_task(init: EspWifiInitialization, mut bt_peripheral: BT, cmd_tx: CmdProducer) {
    let connector = BleConnector::new(&init, &mut bt_peripheral);
    let mut ble = Ble::new(connector, esp_wifi::current_millis);
    let cmd_tx = RefCell::new(cmd_tx);

    loop {
        ble.init().await.unwrap();
        ble.cmd_set_le_advertising_parameters().await.unwrap();
        ble.cmd_set_le_advertising_data(
            create_advertising_data(&[
                AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                AdStructure::CompleteLocalName("Bradipograph"),
            ])
            .unwrap(),
        )
        .await
        .unwrap();
        ble.cmd_set_le_advertise_enable(true).await.unwrap();

        let mut manual_control =
            |_offset: usize, data: &[u8]| match postcard::from_bytes::<Cmd>(data) {
                Ok(cmd) => {
                    println!("got command {cmd:?}");
                    if let Err(cmd) = cmd_tx.borrow_mut().enqueue(cmd) {
                        println!("dropped command {cmd:?}");
                        Err(bleps::att::AttErrorCode::InsufficientResources)
                    } else {
                        Ok(())
                    }
                }
                Err(e) => {
                    println!("error: failed to deserialize command {data:?}: {e}");
                    Err(bleps::att::AttErrorCode::UnlikelyError)
                }
            };

        let mut calibration_status = |offset: usize, data: &mut [u8]| {
            let msg = match (GLOBAL.config(), GLOBAL.position()) {
                (Some(config), Some(pos)) => {
                    let state = State {
                        geom: config.into(),
                        position: pos,
                        pen_down: GLOBAL.pen_down(),
                    };
                    CalibrationStatus::Calibrated(state)
                }
                _ => CalibrationStatus::Uncalibrated,
            };

            let buf = heapless::Vec::<u8, 64>::new();
            let buf = postcard::to_extend(&msg, buf).unwrap();
            let len = buf.len().saturating_sub(offset).min(data.len());
            data[..len].copy_from_slice(&buf[offset..(offset + len)]);
            len
        };

        let mut buffer_status = |_offset: usize, data: &mut [u8]| {
            let cmd_tx = cmd_tx.borrow();
            data[0] = (cmd_tx.capacity() - cmd_tx.len()) as u8;
            1
        };

        // TODO: make a builder pattern that allows us to make these without requiring literal uuids
        bleps::gatt!([service {
            uuid: "68a79627-2609-4569-8d7d-3b29fde28877",
            characteristics: [
                characteristic {
                    uuid: "68a79628-2609-4569-8d7d-3b29fde28877",
                    write: manual_control,
                },
                characteristic {
                    uuid: "68a79629-2609-4569-8d7d-3b29fde28877",
                    read: calibration_status,
                },
                characteristic {
                    uuid: "68a7962a-2609-4569-8d7d-3b29fde28877",
                    read: buffer_status,
                },
            ],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);
        let mut notifier = core::future::pending;
        srv.run(&mut notifier).await.unwrap();
        println!("finished serving, restarting");
    }
}
