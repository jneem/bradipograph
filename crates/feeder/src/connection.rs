use std::{cell::RefCell, time::Duration};

use anyhow::anyhow;
use bradipo_geom::{ConfigBuilder, Point, StepperPositions};
use bradipo_protocol::{CalibrationStatus, Cmd, State, StepperSegment};
use btleplug::{
    api::{Central, Characteristic, Peripheral as _, WriteType},
    platform::{Adapter, Peripheral},
};
use indicatif::{MultiProgress, ProgressBar};
use svg::{
    node::element::{path::Data, Circle, Path},
    Document,
};
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

pub trait BradipographLike {
    async fn read_state(&mut self) -> anyhow::Result<CalibrationStatus>;
    async fn send_cmd(&self, cmd: Cmd) -> anyhow::Result<()>;
    async fn send_cmd_and_wait(&mut self, cmd: Cmd) -> anyhow::Result<()>;
    async fn wait_for_capacity(&mut self, count: usize) -> anyhow::Result<()>;
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
}

impl BradipographLike for Bradipograph {
    async fn read_state(&mut self) -> anyhow::Result<CalibrationStatus> {
        let calibration = self.read(self.calibration.clone()).await?;
        Ok(postcard::from_bytes(&calibration)?)
    }

    async fn send_cmd(&self, cmd: Cmd) -> anyhow::Result<()> {
        let buf = postcard::to_allocvec(&cmd)?;
        self.peripheral
            .write(&self.control, &buf, WriteType::WithoutResponse)
            .await?;
        Ok(())
    }

    async fn send_cmd_and_wait(&mut self, cmd: Cmd) -> anyhow::Result<()> {
        let buf = postcard::to_allocvec(&cmd)?;
        self.write(self.control.clone(), &buf).await?;
        Ok(())
    }

    async fn wait_for_capacity(&mut self, count: usize) -> anyhow::Result<()> {
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

#[derive(Clone)]
pub struct RecordedSegment {
    pub seg: StepperSegment,
    pub pen_down: bool,
}

struct MockState {
    config: bradipo_geom::Config,
    arm_lengths: StepperPositions,
    pen_down: bool,
    segs: Vec<RecordedSegment>,
}

pub struct MockBradipograph {
    out: Box<dyn std::io::Write>,
    inner: std::cell::RefCell<MockState>,
}

impl BradipographLike for MockBradipograph {
    async fn read_state(&mut self) -> anyhow::Result<CalibrationStatus> {
        let cfg = self.inner.borrow().config;
        Ok(CalibrationStatus::Calibrated(State {
            geom: cfg.into(),
            position: self.inner.borrow().arm_lengths,
            pen_down: self.inner.borrow().pen_down,
        }))
    }

    async fn send_cmd(&self, cmd: Cmd) -> anyhow::Result<()> {
        match cmd {
            Cmd::Calibrate(c) => {
                let mut inner = self.inner.borrow_mut();
                inner.config.set_claw_distance(c.claw_distance);

                let angles = inner.config.arm_lengths_to_rotor_angles(&c.arm_lengths);
                inner.arm_lengths = inner.config.rotor_angles_to_stepper_steps(&angles);
            }
            Cmd::SetMaxHang(h) => {
                self.inner.borrow_mut().config.max_hang = h;
            }
            Cmd::SetMinAngle(a) => {
                self.inner.borrow_mut().config.set_min_angle(a);
            }
            Cmd::Segment(seg) => {
                let pen_down = self.inner.borrow().pen_down;
                let mut inner = self.inner.borrow_mut();
                inner.arm_lengths.left = inner
                    .arm_lengths
                    .left
                    .checked_add_signed(seg.left_steps)
                    .unwrap();
                inner.arm_lengths.right = inner
                    .arm_lengths
                    .right
                    .checked_add_signed(seg.right_steps)
                    .unwrap();
                inner.segs.push(RecordedSegment { seg, pen_down });
            }
            Cmd::PenUp => {
                self.inner.borrow_mut().pen_down = false;
            }
            Cmd::PenDown => {
                self.inner.borrow_mut().pen_down = true;
            }
        }
        Ok(())
    }

    async fn send_cmd_and_wait(&mut self, cmd: Cmd) -> anyhow::Result<()> {
        self.send_cmd(cmd).await
    }

    async fn wait_for_capacity(&mut self, _count: usize) -> anyhow::Result<()> {
        Ok(())
    }
}

impl MockBradipograph {
    pub fn with_out_path(out: std::path::PathBuf) -> anyhow::Result<Self> {
        let config = ConfigBuilder::default().build();
        let init_steps = config.point_to_steps(&Point::new(0.0, 0.0));
        Ok(Self {
            out: Box::new(std::fs::File::create(out)?),
            inner: RefCell::new(MockState {
                config,
                arm_lengths: init_steps,
                pen_down: false,
                segs: vec![],
            }),
        })
    }

    pub fn illustrate(&mut self) -> anyhow::Result<()> {
        let cfg = self.inner.borrow().config;
        let segs = self.inner.borrow().segs.clone();
        // Multiply all dimensions by 10 because firefox doesn't like to see small svgs.
        let w = cfg.claw_distance.get() * 10.0;

        let mut steps = cfg.point_to_steps(&Point::new(0.0, 0.0));
        let mut lines = Vec::new();
        let mut points = Vec::new();

        for seg in segs {
            let start_steps = steps;

            steps.left = steps.left.checked_add_signed(seg.seg.left_steps).unwrap();
            steps.right = steps.right.checked_add_signed(seg.seg.right_steps).unwrap();

            let start = cfg.steps_to_point(&start_steps) * 10.0;
            let end = cfg.steps_to_point(&steps) * 10.0;

            points.push(
                Circle::new()
                    .set("cx", start.x)
                    .set("cy", start.y)
                    .set("r", 2.0)
                    .set("fill", "blue"),
            );
            points.push(
                Circle::new()
                    .set("cx", start.x)
                    .set("cy", start.y)
                    .set("fill", "blue"),
            );

            let data = Data::new()
                .move_to((start.x, start.y))
                .line_to((end.x, end.y));
            let line = Path::new()
                .set("fill", "none")
                .set("stroke", "black")
                .set("stroke-width", 1)
                .set("d", data);
            lines.push(line);
        }

        let mut document = Document::new().set("viewBox", (-w / 2.0, 0.0, w, cfg.max_hang.get()));
        for line in lines {
            document = document.add(line);
        }
        for p in points {
            document = document.add(p);
        }
        svg::write(&mut self.out, &document)?;

        Ok(())
    }
}

impl Drop for MockBradipograph {
    fn drop(&mut self) {
        self.illustrate().unwrap();
    }
}
