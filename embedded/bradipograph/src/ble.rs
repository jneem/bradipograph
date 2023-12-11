use core::cell::RefCell;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
};
use embassy_sync::blocking_mutex::{self, raw::CriticalSectionRawMutex};
use esp32c3_hal::peripherals::BT;
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, EspWifiInitialization};

use bradipous_protocol::{CalibrationStatus, Cmd};

use crate::{Channel, Sender};

// TODO: define these in a shared crate
// const UUID: &str = "68a79627-2609-4569-8d7d-3b29fde28877";
// const MANUAL_CONTROL_UUID: &str = "68a79628-2609-4569-8d7d-3b29fde28877";

//pub type CmdReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, Cmd, 16>;
pub type CmdSender = Sender<Cmd, 64>;
pub type CmdChannel = Channel<Cmd, 64>;

// We need to access part of the main state from within one of the gatt callbacks.
// We coordinate this with a global mutex.
pub struct Status {
    pub calibration: Option<bradipous_protocol::Calibration>,
    pub position: Option<bradipous_protocol::Position>,
}

pub static STATUS: blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<Status>> =
    blocking_mutex::Mutex::new(RefCell::new(Status {
        calibration: None,
        position: None,
    }));

#[embassy_executor::task]
pub async fn ble_task(init: EspWifiInitialization, mut bt_peripheral: BT, cmds: CmdSender) {
    let connector = BleConnector::new(&init, &mut bt_peripheral);
    let mut ble = Ble::new(connector, esp_wifi::current_millis);

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
                    if let Err(cmd) = cmds.try_send(cmd) {
                        println!("dropped command {cmd:?}");
                    }
                }
                Err(e) => println!("error: failed to deserialize command {data:?}: {e}"),
            };

        let mut calibration_status = |offset: usize, data: &mut [u8]| {
            let msg = STATUS.lock(|status| match &status.borrow().calibration {
                Some(calib) => CalibrationStatus::Calibrated(calib.clone()),
                None => CalibrationStatus::Uncalibrated,
            });

            let buf = heapless::Vec::<u8, 64>::new();
            let buf = postcard::to_extend(&msg, buf).unwrap();
            let len = buf.len().saturating_sub(offset).min(data.len());
            data[..len].copy_from_slice(&buf[offset..(offset + len)]);
            len
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
            ],
        },]);

        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes);
        let mut notifier = core::future::pending;
        srv.run(&mut notifier).await.unwrap();
    }
}
