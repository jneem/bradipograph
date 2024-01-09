use std::time::Duration;

use anyhow::anyhow;
use bradipous_protocol::{CalibrationStatus, Cmd};
use btleplug::{
    api::{Central, Characteristic, Peripheral as _, WriteType},
    platform::{Adapter, Peripheral},
};

use indicatif::{MultiProgress, ProgressBar};
use uuid::{uuid, Uuid};

pub const CONTROL_UUID: Uuid = uuid!("68a79628-2609-4569-8d7d-3b29fde28877");
pub const CALIBRATION_UUID: Uuid = uuid!("68a79629-2609-4569-8d7d-3b29fde28877");
pub const CAPACITY_UUID: Uuid = uuid!("68a7962a-2609-4569-8d7d-3b29fde28877");

pub struct Bradipograph {
    pub adapter: Adapter,
    pub peripheral: Peripheral,
    pub control: Characteristic,
    pub calibration: Characteristic,
    pub capacity: Characteristic,
}

async fn connect(adapter: &mut Adapter) -> anyhow::Result<Peripheral> {
    let progress = MultiProgress::new();
    let mut bar = progress.add(ProgressBar::new_spinner().with_message("Searching..."));
    bar.enable_steady_tick(crate::TICK);

    let peripheral = find_bradipograph(adapter).await?;
    bar.finish_with_message("found!");
    bar = progress.add(ProgressBar::new_spinner().with_message("Connecting..."));
    bar.enable_steady_tick(crate::TICK);

    peripheral.connect().await?;
    peripheral.discover_services().await?;
    bar.finish_with_message("connected!");

    Ok(peripheral)
}

impl Bradipograph {
    pub async fn new(mut adapter: Adapter) -> anyhow::Result<Self> {
        let peripheral = connect(&mut adapter).await?;

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

        let capacity = peripheral
            .characteristics()
            .into_iter()
            .find(|ch| ch.uuid == CAPACITY_UUID)
            .ok_or_else(|| anyhow!("Bradipous was missing the capacity characteristic"))?;

        Ok(Bradipograph {
            adapter,
            peripheral,
            control,
            calibration,
            capacity,
        })
    }

    async fn read(&mut self, ch: Characteristic) -> anyhow::Result<Vec<u8>> {
        let mut sleep = Duration::from_millis(10);
        for _ in 0..4 {
            match self.peripheral.read(&ch).await {
                Ok(ret) => {
                    return Ok(ret);
                }
                Err(e) => {
                    tokio::time::sleep(sleep).await;
                    sleep *= 2;
                    eprintln!("connection error: {e}, retrying...");
                    self.peripheral = connect(&mut self.adapter).await?;
                }
            }
        }
        Ok(self.peripheral.read(&ch).await?)
    }

    // This is mostly copy-pasted from `read`, but it's tricky to deduplicate them
    // because `Peripheral::read` and `Peripheral::write` both return a future that borrows
    // the peripheral, and I can't parametrize that future over the peripheral's lifetime.
    async fn write(&mut self, ch: Characteristic, buf: &[u8]) -> anyhow::Result<()> {
        let mut sleep = Duration::from_millis(10);
        for _ in 0..4 {
            match self
                .peripheral
                .write(&ch, buf, WriteType::WithResponse)
                .await
            {
                Ok(ret) => {
                    return Ok(ret);
                }
                Err(e) => {
                    tokio::time::sleep(sleep).await;
                    sleep *= 2;
                    eprintln!("connection error: {e}, retrying...");
                    self.peripheral = connect(&mut self.adapter).await?;
                }
            }
        }
        Ok(self
            .peripheral
            .write(&ch, buf, WriteType::WithResponse)
            .await?)
    }

    pub async fn read_state(&mut self) -> anyhow::Result<CalibrationStatus> {
        let calibration = self.read(self.calibration.clone()).await?;
        Ok(postcard::from_bytes(&calibration)?)
    }

    pub async fn send_cmd(&self, cmd: Cmd) -> anyhow::Result<()> {
        let buf = postcard::to_allocvec(&cmd)?;
        self.peripheral
            .write(&self.control, &buf, WriteType::WithoutResponse)
            .await?;
        Ok(())
    }

    pub async fn send_cmd_and_wait(&mut self, cmd: Cmd) -> anyhow::Result<()> {
        let buf = postcard::to_allocvec(&cmd)?;
        self.write(self.control.clone(), &buf).await?;
        Ok(())
    }

    pub async fn wait_for_capacity(&mut self, count: usize) -> anyhow::Result<()> {
        loop {
            let data = self.read(self.capacity.clone()).await?;
            if data.len() == 1 && data[0] as usize >= count {
                return Ok(());
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
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
