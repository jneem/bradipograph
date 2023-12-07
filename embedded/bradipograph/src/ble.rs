use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp32c3_hal::peripherals::BT;
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, EspWifiInitialization};
use num_derive::FromPrimitive;
use serde::{Deserialize, Serialize};

// TODO: define these in a shared crate
// const UUID: &str = "68a79627-2609-4569-8d7d-3b29fde28877";
// const MANUAL_CONTROL_UUID: &str = "68a79628-2609-4569-8d7d-3b29fde28877";

pub type CmdReceiver = embassy_sync::channel::Receiver<'static, CriticalSectionRawMutex, Cmd, 16>;
pub type CmdSender = embassy_sync::channel::Sender<'static, CriticalSectionRawMutex, Cmd, 16>;
pub type CmdChannel = embassy_sync::channel::Channel<CriticalSectionRawMutex, Cmd, 16>;

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq, FromPrimitive, Serialize, Deserialize)]
pub enum ManualControl {
    ShortenLeft,
    LengthenLeft,
    StopLeft,
    ShortenRight,
    LengthenRight,
    StopRight,
}

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum Calibration {
    MarkLeft,
    Finish { y_offset: f32, x_offset: f32 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Cmd {
    Manual(ManualControl),
    Calibrate(Calibration),
    // Temporary, to test calibration
    MoveTo(f32, f32),
}

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

        // TODO: make a builder pattern that allows us to make these without requiring literal uuids
        bleps::gatt!([service {
            uuid: "68a79627-2609-4569-8d7d-3b29fde28877",
            characteristics: [characteristic {
                uuid: "68a79628-2609-4569-8d7d-3b29fde28877",
                write: manual_control,
            },],
        },]);

        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes);
        let mut notifier = core::future::pending;
        srv.run(&mut notifier).await.unwrap();
    }
}
