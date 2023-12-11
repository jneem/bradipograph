use anyhow::anyhow;
use bradipous_protocol::{CalibrationStatus, Cmd};
use btleplug::{
    api::{Central, Characteristic, Peripheral as _},
    platform::{Adapter, Peripheral},
};

use uuid::{uuid, Uuid};

pub const CONTROL_UUID: Uuid = uuid!("68a79628-2609-4569-8d7d-3b29fde28877");
pub const CALIBRATION_UUID: Uuid = uuid!("68a79629-2609-4569-8d7d-3b29fde28877");

pub struct Bradipograph {
    pub peripheral: Peripheral,
    pub control: Characteristic,
    pub calibration: Characteristic,
}

impl Bradipograph {
    pub async fn new(peripheral: Peripheral) -> anyhow::Result<Self> {
        let control = peripheral
            .characteristics()
            .into_iter()
            .find(|ch| ch.uuid == CONTROL_UUID)
            .ok_or_else(|| anyhow!("Bradipous was missing the control characteristic"))?;

        let calibration = peripheral
            .characteristics()
            .into_iter()
            .find(|ch| ch.uuid == CALIBRATION_UUID)
            .ok_or_else(|| anyhow!("Bradipous was missing the calibration characteristic"))?;

        Ok(Bradipograph {
            peripheral,
            control,
            calibration,
        })
    }

    pub async fn read_calibration(&self) -> anyhow::Result<CalibrationStatus> {
        let calibration = self.peripheral.read(&self.calibration).await?;
        Ok(postcard::from_bytes(&calibration)?)
    }

    pub async fn send_cmd(&self, cmd: Cmd) -> anyhow::Result<()> {
        let buf = postcard::to_allocvec(&cmd)?;
        self.peripheral
            .write(
                &self.control,
                &buf,
                btleplug::api::WriteType::WithoutResponse,
            )
            .await?;
        Ok(())
    }
}

pub async fn find_bradipograph(adapter: &mut Adapter) -> anyhow::Result<Peripheral> {
    loop {
        let peripherals = adapter.peripherals().await?;
        for p in peripherals {
            if let Some(props) = p.properties().await? {
                if props.local_name.as_deref() == Some("Bradipograph") {
                    return Ok(p);
                }
            }
        }
    }
}
