#![allow(clippy::redundant_closure_call)]

use std::{
    path::{Path, PathBuf},
    time::Duration,
};

use anyhow::anyhow;
use bradipous_curves::Curve;
use bradipous_geom::ConfigBuilder;
use bradipous_protocol::{Calibration, CalibrationStatus, Cmd};
use btleplug::{
    api::{Central as _, Manager as _, Peripheral as _, ScanFilter},
    platform::{Adapter, Manager, Peripheral},
};
use clap::Parser;
use indicatif::{MultiProgress, ProgressBar};
use reedline::{DefaultPrompt, DefaultPromptSegment, Prompt, Reedline};

use crate::connection::Bradipograph;

mod connection;
mod svg;

const TICK: Duration = Duration::from_millis(50);

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
    let calib = match &calibration {
        CalibrationStatus::Uncalibrated => unreachable!(),
        CalibrationStatus::Calibrated(c) => c,
        CalibrationStatus::CalibratedAndPositioned(c, _) => c,
    };
    eprintln!("Calibration {calibration:?}");
    let config = ConfigBuilder::default()
        .with_max_hang(calib.max_hang as f64)
        .with_spool_radius(calib.spool_radius as f64)
        .with_claw_distance(calib.claw_distance as f64)
        .build();

    eprintln!("Calibration {calibration:?}");

    if let Some(path) = &args.path {
        send_file(&brad, &config, path).await?;
    } else {
        command_mode(&brad).await?;
    }
    Ok(())
}

// This is a hacked-up thing that flattens the curves in xy coordinates and sends them
// as a sequence of move-tos. We should instead send the curves using the representation
// in bradipous-curves and do the flattening on the bradipous.
async fn send_file(
    brad: &Bradipograph,
    config: &bradipous_geom::Config,
    path: &Path,
) -> Result<()> {
    let mut paths = svg::load_svg(path)?;

    svg::transform(&mut paths, config);
    // TODO: support a bigger curve, and break it into pieces before sending.
    let mut curve = Curve::<32>::default();
    for p in &paths {
        if curve.extend(p.elements()).is_err() {
            eprintln!("This curve is too complicated! Truncating it.");
            break;
        }
    }

    brad.send_cmd(Cmd::Draw(curve)).await?;

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

            brad.send_cmd(Cmd::MoveTo(x, y)).await?;
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
