#![allow(clippy::redundant_closure_call)]

use std::{
    path::{Path, PathBuf},
    time::Duration,
};

use anyhow::anyhow;
use bradipo_geom::{Angle, ArmLengths, LenExt as _, Point};
use bradipo_protocol::{Calibration, CalibrationStatus, Cmd};
use btleplug::{
    api::{Central as _, Manager as _, ScanFilter},
    platform::Manager,
};
use clap::{builder::ValueParser, CommandFactory, FromArgMatches, Parser, Subcommand};
use connection::{BradipographLike, MockBradipograph};
use indicatif::ProgressBar;
use kurbo::{BezPath, Rect, Shape as _};
use reedline::{DefaultPrompt, DefaultPromptSegment, Prompt, Reedline};

use crate::{connection::Bradipograph, simulator::Simulation};

mod connection;
mod simulator;
mod svg;

const TICK: Duration = Duration::from_millis(50);
const MAX_STEPS_PER_SEC: u32 = 450;
// what fraction of a second does it take to reach max velocity
const MAX_VELOCITY_PER_SEC: f64 = 1.5;

#[derive(Parser)]
struct Args {
    /// If set, simulates a bradipograph and draws its output to an
    /// svg file at the given path.
    #[arg(long)]
    simulate: Option<PathBuf>,

    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand)]
enum Command {
    /// Calibrate the bradipograph from scratch
    Calibrate {
        /// distance (in cm) between the two claws
        #[arg(value_parser = ValueParser::new(sane_f32))]
        claw_distance: f32,
        /// length (in cm) of the left arm
        #[arg(value_parser = ValueParser::new(sane_f32))]
        left: f32,
        /// length (in cm) of the right arm
        #[arg(value_parser = ValueParser::new(sane_f32))]
        right: f32,
    },

    /// Re-calibrate just the arm lengths
    SetArmLengths {
        /// length (in cm) of the left arm
        #[arg(value_parser = ValueParser::new(sane_f32))]
        left: f32,
        /// length (in cm) of the right arm
        #[arg(value_parser = ValueParser::new(sane_f32))]
        right: f32,
    },

    /// Set the maximum vertical hang
    SetMaxHang {
        /// maximum distance (in cm) that we can hang below the claws
        #[arg(value_parser = ValueParser::new(sane_f32))]
        hang: f32,
    },

    /// Set the minimum hanging angle
    SetMinAngle {
        /// minimum angle (in degrees)
        #[arg(value_parser = ValueParser::new(sane_f32))]
        angle: f32,
    },

    /// Move to a coordinate
    Move {
        /// `x` coordinate (in cm), with zero being exactly between the two claws
        #[arg(value_parser = ValueParser::new(sane_signed_f32))]
        x: f32,
        /// `y` coordinate (in cm), with zero being the highest drawable position,
        /// and positive values being lower.
        #[arg(value_parser = ValueParser::new(sane_f32))]
        y: f32,
    },

    /// Draw an SVG file
    Svg {
        /// path to the svg input
        path: PathBuf,

        /// scale the output to this width (in cm)
        #[arg(value_parser = ValueParser::new(sane_f32))]
        width: Option<f32>,

        /// scale the output to this width (in cm)
        #[arg(value_parser = ValueParser::new(sane_f32))]
        height: Option<f32>,

        #[arg(long)]
        fill: bool,
    },

    /// Draw a square showing the drawable region
    Square,

    /// Query the current bradipograph status
    Query,
}

async fn reload_simulation(brad: &mut impl BradipographLike) -> Result<Simulation> {
    match brad.read_state().await? {
        CalibrationStatus::Uncalibrated => Err(anyhow!("Calibration mysteriously failed").into()),
        CalibrationStatus::Calibrated(state) => Ok(make_simulation(&state)),
    }
}

async fn calibrate(brad: &mut impl BradipographLike, calib: Calibration) -> Result<Simulation> {
    brad.send_cmd_and_wait(Cmd::Calibrate(calib)).await?;
    reload_simulation(brad).await
}

impl Command {
    async fn execute(
        &self,
        brad: &mut impl BradipographLike,
        simulation: &mut Option<Simulation>,
    ) -> Result<()> {
        match self {
            Command::Calibrate {
                claw_distance,
                left,
                right,
            } => {
                let calib = Calibration {
                    claw_distance: claw_distance.cm(),
                    arm_lengths: ArmLengths {
                        left: left.cm(),
                        right: right.cm(),
                    },
                };
                *simulation = Some(calibrate(brad, calib).await?);
            }
            Command::SetArmLengths { left, right } => match brad.read_state().await? {
                CalibrationStatus::Uncalibrated => {
                    return Err(anyhow!("set-arm-lengths requires calibration").into())
                }
                CalibrationStatus::Calibrated(state) => {
                    let calib = Calibration {
                        claw_distance: state.claw_distance,
                        arm_lengths: ArmLengths {
                            left: left.cm(),
                            right: right.cm(),
                        },
                    };
                    *simulation = Some(calibrate(brad, calib).await?);
                }
            },
            Command::SetMaxHang { hang } => {
                brad.send_cmd_and_wait(Cmd::SetMaxHang(hang.cm())).await?;
                *simulation = Some(reload_simulation(brad).await?);
            }
            Command::SetMinAngle { angle } => {
                brad.send_cmd_and_wait(Cmd::SetMinAngle(Angle::degrees(*angle)))
                    .await?;
                *simulation = Some(reload_simulation(brad).await?);
            }
            Command::Move { x, y } => {
                let Some(sim) = simulation.as_mut() else {
                    return Err(anyhow!("move requires calibration").into());
                };
                let pos = Point::new(*x, *y);
                for cmd in sim.move_to(pos) {
                    brad.send_cmd(cmd).await?;
                }
            }
            Command::Svg {
                path,
                width,
                height,
                fill,
            } => {
                let Some(sim) = simulation.as_mut() else {
                    return Err(anyhow!("svg requires calibration").into());
                };
                let mut rect = sim.geom.draw_box();
                if let Some(width) = width.map(f64::from) {
                    if width < rect.width() {
                        rect.x0 = -width / 2.0;
                        rect.x1 = width / 2.0;
                    }
                }
                if let Some(height) = height.map(f64::from) {
                    if height < rect.height() {
                        let center = rect.center().y;
                        rect.y0 = center - height / 2.0;
                        rect.y1 = center + height / 2.0;
                    }
                }

                send_file(brad, sim, path, rect, *fill).await?;
            }
            Command::Square => {
                let Some(sim) = simulation.as_mut() else {
                    return Err(anyhow!("square requires calibration").into());
                };
                draw_box(brad, sim).await?;
            }
            Command::Query => {
                let calib = brad.read_state().await?;
                eprintln!("calibration: {calib:?}");
            }
        }
        Ok(())
    }
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

// A clap ValueParser for f64s that are non-negative and not insanely large.
fn sane_f32(s: &str) -> std::result::Result<f32, clap::Error> {
    sane_signed_f32(s).and_then(|x| {
        if x >= 0.0 {
            Ok(x)
        } else {
            Err(clap::Error::new(clap::error::ErrorKind::InvalidValue))
        }
    })
}

fn sane_signed_f32(s: &str) -> std::result::Result<f32, clap::Error> {
    s.parse::<f32>()
        .map_err(|_| clap::Error::new(clap::error::ErrorKind::InvalidValue))
        .and_then(|x| {
            if (-10_000.0..=10_000.0).contains(&x) {
                Ok(x)
            } else {
                Err(clap::Error::new(clap::error::ErrorKind::InvalidValue))
            }
        })
}

type Result<T> = std::result::Result<T, Error>;

fn make_simulation(state: &bradipo_protocol::State) -> Simulation {
    let config = state.geom();
    eprintln!("Calibration {state:?}");

    Simulation {
        geom: config,
        max_steps_per_sec: MAX_STEPS_PER_SEC as u16,
        max_steps_per_sec_per_sec: (MAX_STEPS_PER_SEC as f64 * MAX_VELOCITY_PER_SEC) as u32,
        pen_down: false, // TODO: make the bradipo report this too
        position: state.position,
        steps_per_sec: 0,
    }
}

async fn handle_connection(brad: &mut impl BradipographLike, args: &Args) -> Result<()> {
    let bar = ProgressBar::new_spinner().with_message("Checking calibration...");
    bar.enable_steady_tick(TICK);

    let calibration = brad.read_state().await?;
    bar.finish_with_message("received!");

    let mut simulation = match calibration {
        CalibrationStatus::Uncalibrated => None,
        CalibrationStatus::Calibrated(state) => Some(make_simulation(&state)),
    };

    if let Some(cmd) = args.command.as_ref() {
        cmd.execute(brad, &mut simulation).await
    } else {
        command_mode(brad, simulation).await
    }
}

async fn send_path(
    brad: &mut impl BradipographLike,
    simulation: &mut Simulation,
    path: &BezPath,
) -> Result<()> {
    let cmds = simulation.draw_path(path, 0.05);
    let chunks = cmds.chunks(32);

    for chunk in chunks {
        brad.wait_for_capacity(32).await?;
        for cmd in chunk {
            brad.send_cmd_and_wait(cmd.clone()).await?;
        }
    }
    Ok(())
}

async fn send_file(
    brad: &mut impl BradipographLike,
    simulation: &mut Simulation,
    path: &Path,
    target_rect: Rect,
    fill: bool,
) -> Result<()> {
    let mut p = svg::load_svg(path)?;

    // TODO: allow pauses for "refreshing" the marker
    svg::transform(&mut p, &target_rect);
    send_path(brad, simulation, &p).await?;

    if fill {
        let zigzag = bradipo_sketcher::Zigzag::default().clipped_to(&p);

        for lines in zigzag {
            send_path(brad, simulation, &lines).await?;
        }
    }

    Ok(())
}

async fn draw_box(brad: &mut impl BradipographLike, simulation: &mut Simulation) -> Result<()> {
    let bbox = simulation.geom.draw_box();
    let path: BezPath = bbox.path_elements(0.01).collect();
    let cmds = simulation.draw_path(&path, 0.05);
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

fn read_cmd(reed: &mut Reedline, prompt: &dyn Prompt) -> Result<Command> {
    let s = reed.read_line(prompt)?;
    match s {
        reedline::Signal::Success(s) => {
            if s.trim() == "quit" || s.trim() == "q" {
                return Err(Error::Exit);
            }
            let matches = <Args as CommandFactory>::command()
                .no_binary_name(true)
                .try_get_matches_from(s.split_whitespace())?;
            Ok(<Command as FromArgMatches>::from_arg_matches(&matches)?)
        }
        reedline::Signal::CtrlC | reedline::Signal::CtrlD => Err(Error::Exit),
    }
}

async fn command_mode(
    brad: &mut impl BradipographLike,
    mut simulation: Option<Simulation>,
) -> Result<()> {
    let mut reed = Reedline::create();
    let prompt = string_prompt("");
    loop {
        let cmd = match read_cmd(&mut reed, &prompt) {
            Ok(c) => c,
            Err(Error::Err(e)) => {
                eprintln!("{e}");
                continue;
            }
            Err(Error::Exit) => {
                return Ok(());
            }
        };

        match cmd.execute(brad, &mut simulation).await {
            Ok(_) => {}
            Err(Error::Err(e)) => {
                eprintln!("error: {e}");
            }
            Err(Error::Exit) => {
                return Ok(());
            }
        }
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    pretty_env_logger::init();

    let args = Args::parse();

    if let Some(out) = &args.simulate {
        let mut brad = MockBradipograph::with_out_path(out.clone())?;
        handle_connection(&mut brad, &args).await.unwrap();
    } else {
        let manager = Manager::new().await?;
        let adapter = manager
            .adapters()
            .await?
            .into_iter()
            .next()
            .ok_or(anyhow!("no bluetooth adapter"))?;
        adapter.start_scan(ScanFilter::default()).await?;
        let mut brad = Bradipograph::new(adapter).await?;
        if let Err(Error::Err(_)) = handle_connection(&mut brad, &args).await {
            eprintln!("lost connection, exiting...");
        }
    }

    Ok(())
}
