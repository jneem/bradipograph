#![allow(clippy::redundant_closure_call)]

use std::{
    path::{Path, PathBuf},
    time::Duration,
};

use anyhow::anyhow;
use bradipous_protocol::{Calibration, CalibrationStatus, Cmd};
use btleplug::{
    api::{Central as _, Manager as _, Peripheral as _, ScanFilter},
    platform::{Adapter, Manager, Peripheral},
};
use clap::Parser;
use indicatif::{MultiProgress, ProgressBar};
use kurbo::{BezPath, Point, Shape as _};
use reedline::{DefaultPrompt, DefaultPromptSegment, Prompt, Reedline};

use crate::{connection::Bradipograph, simulator::Simulation};

mod connection;
mod simulator;
mod svg;

const TICK: Duration = Duration::from_millis(50);
const MAX_STEPS_PER_SEC: u32 = 400;
// what fraction of a second does it take to reach max velocity
const MAX_VELOCITY_PER_SEC: u32 = 2;

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

fn make_simulation(state: &bradipous_protocol::State) -> Simulation {
    let config = state.geom();
    eprintln!("Calibration {state:?}");

    Simulation {
        geom: config,
        max_steps_per_sec: MAX_STEPS_PER_SEC as u16,
        max_steps_per_sec_per_sec: MAX_STEPS_PER_SEC * MAX_VELOCITY_PER_SEC,
        pen_down: false, // TODO: make the bradipo report this too
        position: state.position,
        steps_per_sec: 0,
    }
}

async fn handle_connection(adapter: &mut Adapter, args: &Args) -> Result<()> {
    let peripheral = connect(adapter).await?;
    let brad = Bradipograph::new(peripheral).await?;

    let bar = ProgressBar::new_spinner().with_message("Checking calibration...");
    bar.enable_steady_tick(TICK);

    let mut calibration = brad.read_state().await?;
    bar.finish_with_message("received!");

    while matches!(calibration, CalibrationStatus::Uncalibrated) {
        eprintln!("Uncalibrated");
        command_mode(&brad, None).await?;
        calibration = brad.read_state().await?;
    }
    let CalibrationStatus::Calibrated(state) = calibration else {
        unreachable!();
    };
    let simulation = make_simulation(&state);

    if let Some(path) = &args.path {
        // When sending a file, quit on error (otherwise we keep trying to send it
        // on reconnection).
        match send_file(&brad, simulation, path).await {
            Err(Error::Err(e)) => {
                eprintln!("Sending the file failed with error {e}");
                Err(Error::Exit)
            }
            x => x,
        }
    } else {
        command_mode(&brad, Some(simulation)).await
    }
}

async fn send_file(brad: &Bradipograph, mut simulation: Simulation, path: &Path) -> Result<()> {
    let mut p = svg::load_svg(path)?;

    svg::transform(&mut p, &simulation.geom);

    let cmds = simulation.draw_path(&p, 0.05);
    dbg!(&cmds);
    let chunks = cmds.chunks(32);

    for chunk in chunks {
        brad.wait_for_capacity(32).await?;
        for cmd in chunk {
            brad.send_cmd_and_wait(cmd.clone()).await?;
        }
    }

    Ok(())
}

async fn draw_box(brad: &Bradipograph, simulation: &mut Simulation) -> Result<()> {
    let bbox = simulation.geom.draw_box();
    let path: BezPath = bbox.path_elements(0.01).collect();
    let cmds = simulation.draw_path(&path, 0.05);
    dbg!(&cmds);
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

async fn command_mode(brad: &Bradipograph, mut simulation: Option<Simulation>) -> Result<()> {
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
            let Some(sim) = simulation.as_mut() else {
                eprintln!("Cannot draw square, you must calibrate first");
                continue;
            };
            let pos = Point::new(x.into(), y.into());
            for cmd in sim.move_to(pos) {
                brad.send_cmd(cmd).await?;
            }
        } else if s == "query" {
            let calib = brad.read_state().await?;
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
            let calib = brad.read_state().await?;
            let CalibrationStatus::Calibrated(state) = calib else {
                eprintln!("Calibration failed?? Try again, I guess...");
                continue;
            };
            simulation = Some(make_simulation(&state));
        } else if s == "square" {
            let Some(sim) = simulation.as_mut() else {
                eprintln!("Cannot draw square, you must calibrate first");
                continue;
            };
            draw_box(brad, sim).await?;
        } else if s == "min-angle" {
            let Some(deg) = read_number(&mut reed, "degrees")? else {
                continue;
            };
            brad.send_cmd_and_wait(Cmd::SetMinAngleDegrees(deg)).await?;
            let calib = brad.read_state().await?;
            let CalibrationStatus::Calibrated(state) = calib else {
                eprintln!("Calibration failed?? Try again, I guess...");
                continue;
            };
            simulation = Some(make_simulation(&state));
        } else if s == "max-hang" {
            let Some(hang) = read_number(&mut reed, "cm")? else {
                continue;
            };
            brad.send_cmd_and_wait(Cmd::SetMaxHang(hang)).await?;
            let calib = brad.read_state().await?;
            let CalibrationStatus::Calibrated(state) = calib else {
                eprintln!("Calibration failed?? Try again, I guess...");
                continue;
            };
            simulation = Some(make_simulation(&state));
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
