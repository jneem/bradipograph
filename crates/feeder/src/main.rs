#![allow(clippy::redundant_closure_call)]

use std::{
    path::{Path, PathBuf},
    time::Duration,
};

use anyhow::anyhow;
use bradipous_geom::Config;
use bradipous_protocol::{Calibration, CalibrationStatus, Cmd, StepperSegment};
use btleplug::{
    api::{Central as _, Manager as _, Peripheral as _, ScanFilter},
    platform::{Adapter, Manager, Peripheral},
};
use clap::Parser;
use indicatif::{MultiProgress, ProgressBar};
use kurbo::{BezPath, Point, Shape as _};
use reedline::{DefaultPrompt, DefaultPromptSegment, Prompt, Reedline};
use svg::plan;

use crate::connection::Bradipograph;

mod connection;
mod svg;

const TICK: Duration = Duration::from_millis(50);
const MAX_STEPS_PER_SEC: u32 = 500;
// what fraction of a second does it take to reach max velocity
const MAX_VELOCITY_PER_SEC: u32 = 1;

#[derive(Parser)]
struct Args {
    path: Option<PathBuf>,
}

#[derive(Debug)]
enum Error {
    Exit,
    Err(anyhow::Error),
}

impl<E> From<E> for Error
where
    E: Into<anyhow::Error>,
{
    fn from(e: E) -> Self {
        Error::Err(e.into())
    }
}

type Result<T> = std::result::Result<T, Error>;

async fn connect(adapter: &mut Adapter) -> anyhow::Result<Peripheral> {
    let progress = MultiProgress::new();
    let mut bar = progress.add(ProgressBar::new_spinner().with_message("Searching..."));
    bar.enable_steady_tick(TICK);

    let peripheral = connection::find_bradipograph(adapter).await?;
    bar.finish_with_message("found!");
    bar = progress.add(ProgressBar::new_spinner().with_message("Connecting..."));
    bar.enable_steady_tick(TICK);

    peripheral.connect().await?;
    peripheral.discover_services().await?;
    bar.finish_with_message("connected!");

    Ok(peripheral)
}

async fn handle_connection(adapter: &mut Adapter, args: &Args) -> Result<()> {
    let peripheral = connect(adapter).await?;
    let brad = Bradipograph::new(peripheral).await?;

    let bar = ProgressBar::new_spinner().with_message("Checking calibration...");
    bar.enable_steady_tick(TICK);

    let mut calibration = brad.read_calibration().await?;
    bar.finish_with_message("received!");

    while matches!(calibration, CalibrationStatus::Uncalibrated) {
        eprintln!("Uncalibrated");
        command_mode(&brad).await?;
        calibration = brad.read_calibration().await?;
    }
    let CalibrationStatus::Calibrated(calib, pos) = calibration else {
        unreachable!();
    };
    let config = Config::from(calib);
    let init_pos = config.steps_to_point(&pos);
    eprintln!("Calibration {config:?}, steps {pos:?}, pos {init_pos:?}");

    if let Some(path) = &args.path {
        // When sending a file, quit on error (otherwise we keep trying to send it
        // on reconnection).
        match send_file(&brad, &config, init_pos, path).await {
            Err(Error::Err(e)) => {
                eprintln!("Sending the file failed with error {e}");
                Err(Error::Exit)
            }
            x => x,
        }
    } else {
        command_mode(&brad).await
    }
}

async fn send_file(
    brad: &Bradipograph,
    config: &bradipous_geom::Config,
    initial_position: Point,
    path: &Path,
) -> Result<()> {
    let mut p = svg::load_svg(path)?;

    svg::transform(&mut p, config);

    let cmds = plan(&p, &initial_position, config)?;
    let chunks = cmds.chunks(32);

    for chunk in chunks {
        brad.wait_for_capacity(32).await?;
        for cmd in chunk {
            brad.send_cmd_and_wait(cmd.clone()).await?;
        }
    }

    Ok(())
}

async fn draw_box(
    brad: &Bradipograph,
    config: &bradipous_geom::Config,
    initial_position: Point,
) -> Result<()> {
    let bbox = config.draw_box();
    let path: BezPath = bbox.path_elements(0.01).collect();
    let cmds = plan(&path, &initial_position, config)?;
    for cmd in cmds {
        brad.send_cmd_and_wait(cmd).await?;
    }

    Ok(())
}

fn string_prompt(s: &str) -> DefaultPrompt {
    DefaultPrompt::new(
        DefaultPromptSegment::Basic(s.to_owned()),
        DefaultPromptSegment::Empty,
    )
}

fn read_cmd(reed: &mut Reedline, prompt: &dyn Prompt) -> Result<String> {
    let s = reed.read_line(prompt)?;
    match s {
        reedline::Signal::Success(s) => Ok(s),
        reedline::Signal::CtrlC | reedline::Signal::CtrlD => Err(Error::Exit),
    }
}

fn read_number(reed: &mut Reedline, prompt: &str) -> Result<Option<f32>> {
    loop {
        let s = read_cmd(reed, &string_prompt(prompt))?;
        if s == "q" {
            return Ok(None);
        }
        match s.parse::<f32>() {
            Ok(x) => return Ok(Some(x)),
            Err(_) => {
                eprintln!("That isn't a number; try again, or enter 'q' to go back")
            }
        }
    }
}

async fn command_mode(brad: &Bradipograph) -> Result<()> {
    let mut reed = Reedline::create();
    let prompt = string_prompt("");
    loop {
        let s = read_cmd(&mut reed, &prompt)?;
        let s = s.trim();

        if s == "quit" {
            return Err(Error::Exit);
        } else if s == "continue" {
            return Ok(());
        } else if s == "move" {
            let Some(x) = read_number(&mut reed, "x")? else {
                continue;
            };
            let Some(y) = read_number(&mut reed, "y")? else {
                continue;
            };
            let calib = brad.read_calibration().await?;
            let CalibrationStatus::Calibrated(calib, steps) = calib else {
                eprintln!("Cannot move, you must calibrate first");
                continue;
            };
            let config = Config::from(calib);
            let init_pos = config.steps_to_point(&steps);
            let init_steps = config.point_to_steps(&init_pos);
            let pos = Point::new(x.into(), y.into());
            let steps = config.point_to_steps(&pos);
            let right_steps = steps.right as i32 - init_steps.right as i32;
            let left_steps = steps.left as i32 - init_steps.left as i32;
            let seg = StepperSegment {
                left_steps,
                right_steps,
                start_steps_per_sec: 34,
                end_steps_per_sec: 34,
                steps_per_sec_per_sec: MAX_STEPS_PER_SEC * MAX_VELOCITY_PER_SEC,
            };
            eprintln!("from {init_pos:?} to {pos:?}, seg {seg:?}");
            if let Some((accel, decel)) = seg.split(MAX_STEPS_PER_SEC as u16) {
                brad.send_cmd(Cmd::Segment(accel)).await?;
                brad.send_cmd(Cmd::Segment(decel)).await?;
            }
        } else if s == "query" {
            let calib = brad.read_calibration().await?;
            eprintln!("calibration: {calib:?}");
        } else if s == "calibrate" {
            let Some(d) = read_number(&mut reed, "How far apart (in cm) are the two claws? ")?
            else {
                continue;
            };
            let Some(left) = read_number(
                &mut reed,
                "How far apart (in cm) is the head from the left claw? ",
            )?
            else {
                continue;
            };
            let Some(right) = read_number(
                &mut reed,
                "How far apart (in cm) is the head from the right claw? ",
            )?
            else {
                continue;
            };
            brad.send_cmd_and_wait(Cmd::Calibrate(Calibration {
                claw_distance_cm: d,
                left_arm_cm: left,
                right_arm_cm: right,
            }))
            .await?;
        } else if s == "square" {
            let calib = brad.read_calibration().await?;

            let CalibrationStatus::Calibrated(calib, steps) = calib else {
                eprintln!("Cannot draw square, you must calibrate first");
                continue;
            };
            let config = Config::from(calib);
            let init_pos = config.steps_to_point(&steps);
            draw_box(brad, &config, init_pos).await?;
        } else {
            eprintln!("unknown command");
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let manager = Manager::new().await?;
    let mut adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or(anyhow!("no bluetooth adapter"))?;
    adapter.start_scan(ScanFilter::default()).await?;
    loop {
        if let Err(Error::Err(e)) = handle_connection(&mut adapter, &args).await {
            eprintln!("lost connection, restarting (cause: {e})");
        } else {
            eprintln!("exiting...");
            break;
        }
    }

    Ok(())
}
