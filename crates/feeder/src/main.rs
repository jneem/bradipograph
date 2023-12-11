#![allow(clippy::redundant_closure_call)]

use std::{
    path::{Path, PathBuf},
    time::Duration,
};

use anyhow::anyhow;
use bradipous_geom::ConfigBuilder;
use bradipous_protocol::{Calibrate, CalibrationStatus, Cmd, ManualControl};
use btleplug::{
    api::{Central as _, Manager as _, Peripheral as _, ScanFilter},
    platform::{Adapter, Manager, Peripheral},
};
use clap::Parser;
use crossterm::event::{Event, EventStream, KeyCode, KeyEventKind};
use crossterm::terminal::{disable_raw_mode, enable_raw_mode};
use futures::StreamExt;
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

async fn manual_move(events: &mut EventStream, brad: &Bradipograph) -> Result<()> {
    use ManualControl::*;
    while let Some(ev) = events.next().await.transpose()? {
        let Event::Key(ev) = ev else {
            continue;
        };
        if ev.kind != KeyEventKind::Press {
            continue;
        }
        let cmd = match ev.code {
            KeyCode::Char('q') => {
                return Err(Error::Exit);
            }
            KeyCode::Char('s') => Some(Cmd::Manual(ShortenLeft)),
            KeyCode::Char('d') => Some(Cmd::Manual(StopLeft)),
            KeyCode::Char('f') => Some(Cmd::Manual(LengthenLeft)),
            KeyCode::Char('l') => Some(Cmd::Manual(ShortenRight)),
            KeyCode::Char('k') => Some(Cmd::Manual(StopRight)),
            KeyCode::Char('j') => Some(Cmd::Manual(LengthenRight)),
            KeyCode::Enter => {
                return Ok(());
            }
            _ => None,
        };
        if let Some(cmd) = cmd {
            brad.send_cmd(cmd).await?;
        }
    }
    Err(anyhow!("event stream ended").into())
}

async fn handle_calibration(brad: &Bradipograph) -> Result<()> {
    eprintln!("Entering calibration mode. Keys s, d, f, j, k, l to move, <enter> to confirm.");
    eprintln!("Move to the left claw (20cm below, 10cm to the side)");
    enable_raw_mode()?;

    let err: Result<()> = (|| async move {
        let mut events = EventStream::new();

        manual_move(&mut events, brad).await?;
        brad.send_cmd(Cmd::Calibrate(Calibrate::MarkLeft)).await?;

        eprintln!("Move to the right claw (20cm below, 10cm to the side)");
        manual_move(&mut events, brad).await?;
        brad.send_cmd_and_wait(Cmd::Calibrate(Calibrate::Finish {
            y_offset: 20.0,
            x_offset: 10.0,
        }))
        .await?;

        Ok(())
    })()
    .await;

    disable_raw_mode()?;

    err
}

async fn handle_connection(adapter: &mut Adapter, args: &Args) -> Result<()> {
    let peripheral = connect(adapter).await?;
    let brad = Bradipograph::new(peripheral).await?;

    let bar = ProgressBar::new_spinner().with_message("Checking calibration...");
    bar.enable_steady_tick(TICK);

    let mut calibration = brad.read_calibration().await?;
    bar.finish_with_message("received!");

    while matches!(calibration, CalibrationStatus::Uncalibrated) {
        eprintln!("Uncalibrated, entering calibration mode");
        handle_calibration(&brad).await?;

        calibration = brad.read_calibration().await?;
    }
    let calib = match &calibration {
        CalibrationStatus::Uncalibrated => unreachable!(),
        CalibrationStatus::Calibrated(c) => c,
        //CalibrationStatus::CalibratedAndPositioned(c, _) => c,
    };
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
    let mut points = Vec::new();
    for p in &paths {
        p.flatten(0.5, |el| match el {
            kurbo::PathEl::MoveTo(pt) => points.push(pt),
            kurbo::PathEl::LineTo(pt) => points.push(pt),
            _ => unreachable!(),
        });
    }

    if points.len() > 64 {
        Err(anyhow!("{} points is too many :(", points.len()))?;
    }

    for p in points {
        brad.send_cmd(Cmd::MoveTo(p.x as f32, p.y as f32)).await?;
    }

    Ok(())
}

fn string_prompt(s: &str) -> DefaultPrompt {
    DefaultPrompt::new(
        DefaultPromptSegment::Basic(s.to_owned()),
        DefaultPromptSegment::Basic(s.to_owned()),
    )
}

fn read_cmd(reed: &mut Reedline, prompt: &dyn Prompt) -> Result<String> {
    let s = reed.read_line(prompt)?;
    match s {
        reedline::Signal::Success(s) => Ok(s),
        reedline::Signal::CtrlC | reedline::Signal::CtrlD => Err(Error::Exit),
    }
}

async fn command_mode(brad: &Bradipograph) -> Result<()> {
    let mut reed = Reedline::create();
    let prompt = DefaultPrompt::default();
    loop {
        let s = read_cmd(&mut reed, &prompt)?;
        let s = s.trim();

        if s == "quit" {
            break;
        } else if s == "move" {
            let x = read_cmd(&mut reed, &string_prompt("x? "))?;
            let Ok(x) = x.parse::<f32>() else {
                eprintln!("error: expected a number");
                continue;
            };
            let y = read_cmd(&mut reed, &string_prompt("y? "))?;
            let Ok(y) = y.parse::<f32>() else {
                eprintln!("error: expected a number");
                continue;
            };

            brad.send_cmd(Cmd::MoveTo(x, y)).await?;
        } else if s == "query" {
            let calib = brad.read_calibration().await?;
            eprintln!("calibration: {calib:?}");
        }
    }

    Ok(())
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
