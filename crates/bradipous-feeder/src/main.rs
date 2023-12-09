use std::io::{self, stdout};

use bradipous_protocol::ManualControl;
use btleplug::{api::Peripheral as _, platform::Peripheral};
use crossterm::{
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use futures::Stream;
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Layout},
    style::{Style, Stylize as _},
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Frame, Terminal,
};
use tokio::sync::mpsc::{self, Sender};
use tokio_stream::StreamExt as _;

mod connection;

use connection::{Cmd, Event};

use crate::connection::CalibrationCmd;

#[derive(Clone, Debug)]
struct LoadingState {
    status: String,
    error: Option<String>,
}

#[derive(Clone, Debug)]
struct ConnectedState {
    dev: Peripheral,
}

#[derive(Debug)]
struct App {
    state: AppState,
    cmd_tx: mpsc::Sender<connection::Cmd>,
}

#[derive(Clone, Debug)]
enum AppState {
    Loading(LoadingState),
    Connected(ConnectedState),
    Exiting,
}

impl App {
    fn draw(&mut self, frame: &mut Frame) {
        use AppState::*;
        match &self.state {
            Loading(state) => loading(frame, state),
            Connected(state) => connected(frame, state),
            Exiting => todo!(),
        }
    }

    async fn handle_events(
        &mut self,
        events: &mut (impl Stream<Item = Event> + Unpin),
    ) -> io::Result<()> {
        use AppState::*;

        let state = std::mem::replace(&mut self.state, Exiting);

        self.state = match state {
            Loading(state) => loading_events(events, state).await?,
            Connected(state) => connected_events(events, &mut self.cmd_tx, state).await?,
            Exiting => todo!(),
        };

        Ok(())
    }
}

fn loading(frame: &mut Frame, state: &LoadingState) {
    let block = Block::default().title("Bradiograph").borders(Borders::ALL);
    let inner = block.inner(frame.size());
    let layout = Layout::new()
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(inner);

    frame.render_widget(block, frame.size());

    frame.render_widget(Paragraph::new(state.status.clone()), layout[0]);
    if let Some(err) = &state.error {
        frame.render_widget(
            Paragraph::new(err.clone()).style(Style::new().red().on_white()),
            layout[1],
        )
    }
}

fn connected(frame: &mut Frame, state: &ConnectedState) {
    let block = Block::default().title("Bradiograph").borders(Borders::ALL);
    let inner = block.inner(frame.size());
    let layout = Layout::new()
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(inner);

    frame.render_widget(block, frame.size());

    frame.render_widget(
        Paragraph::new(format!("Found bradiograph at id {}", state.dev.id())),
        layout[0],
    );

    let chars: Vec<_> = state
        .dev
        .characteristics()
        .into_iter()
        .map(|ch| ListItem::new(ch.uuid.to_string()))
        .collect();
    frame.render_widget(List::new(chars), layout[1]);
}

async fn loading_events(
    events: &mut (impl Stream<Item = Event> + Unpin),
    state: LoadingState,
) -> io::Result<AppState> {
    while let Some(ev) = events.next().await {
        match ev {
            Event::Key('q') => return Ok(AppState::Exiting),
            Event::Found => {
                return Ok(AppState::Loading(LoadingState {
                    status: "Found bradipograph, connnecting...".to_string(),
                    ..state
                }))
            }
            Event::Connected(dev) => return Ok(AppState::Connected(ConnectedState { dev })),
            _ => {}
        }
    }
    Ok(AppState::Loading(state))
}

async fn connected_events(
    events: &mut (impl Stream<Item = Event> + Unpin),
    cmds: &mut Sender<Cmd>,
    state: ConnectedState,
) -> io::Result<AppState> {
    use CalibrationCmd::*;
    use ManualControl::*;
    while let Some(ev) = events.next().await {
        let cmd = match ev {
            Event::Key('q') => {
                return Ok(AppState::Exiting);
            }
            Event::Key('s') => Some(Move(ShortenLeft)),
            Event::Key('d') => Some(Move(StopLeft)),
            Event::Key('f') => Some(Move(LengthenLeft)),
            Event::Key('l') => Some(Move(ShortenRight)),
            Event::Key('k') => Some(Move(StopRight)),
            Event::Key('j') => Some(Move(LengthenRight)),
            _ => None,
        };
        if let Some(cmd) = cmd {
            cmds.send(Cmd::Calibration(cmd)).await.unwrap();
        }
    }
    Ok(AppState::Connected(state))
}

async fn run() -> io::Result<()> {
    let mut terminal = Terminal::new(CrosstermBackend::new(stdout()))?;
    let (cmd_tx, cmd_rx) = mpsc::channel(8);
    let mut events = connection::init(cmd_rx);

    let mut app = App {
        cmd_tx,
        state: AppState::Loading(LoadingState {
            status: "Searching...".to_owned(),
            error: None,
        }),
    };
    while !matches!(app.state, AppState::Exiting) {
        terminal.draw(|f| app.draw(f))?;
        app.handle_events(&mut events).await?;
    }
    Ok(())
}

#[tokio::main]
async fn main() -> io::Result<()> {
    enable_raw_mode()?;
    stdout().execute(EnterAlternateScreen)?;

    let result = run().await;

    disable_raw_mode()?;
    stdout().execute(LeaveAlternateScreen)?;

    if let Err(e) = result {
        eprintln!("Error: {e}");
    }

    Ok(())
}
