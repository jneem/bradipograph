use std::io::{self, stdout};

use bradipous_protocol::{Calibrate, Cmd, ManualControl};
use btleplug::{api::Peripheral as _, platform::Peripheral};
use crossterm::{
    event::KeyCode,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
    ExecutableCommand,
};
use futures::Stream;
use log::LevelFilter;
use ratatui::{
    backend::CrosstermBackend,
    layout::{Constraint, Direction, Layout},
    style::{Style, Stylize},
    text::Text,
    widgets::{Block, Borders, List, ListItem, Paragraph},
    Frame, Terminal,
};
use tokio::sync::mpsc::{self, Sender};
use tokio_stream::StreamExt as _;

mod connection;

use connection::Event;

#[derive(Clone, Debug)]
struct LoadingState {
    status: String,
    error: Option<String>,
}

impl Default for LoadingState {
    fn default() -> Self {
        LoadingState {
            status: "Searching...".to_owned(),
            error: None,
        }
    }
}

#[derive(Clone, Debug)]
struct CalibratingState {
    left_finished: bool,
    dev: Peripheral,
}

#[derive(Clone, Debug)]
struct ReadyState {
    dev: Peripheral,
    x_focused: bool,
    x: String,
    y: String,
}

#[derive(Debug)]
struct App {
    state: AppState,
    cmd_tx: mpsc::Sender<Cmd>,
}

#[derive(Clone, Debug)]
enum AppState {
    Loading(LoadingState),
    Calibrating(CalibratingState),
    Ready(ReadyState),
    Exiting,
}

impl App {
    fn draw(&mut self, frame: &mut Frame) {
        use AppState::*;
        match &self.state {
            Loading(state) => loading(frame, state),
            Calibrating(state) => calibrating(frame, state),
            Ready(state) => ready(frame, state),
            Exiting => unreachable!(),
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
            Calibrating(state) => calibrating_events(events, &mut self.cmd_tx, state).await?,
            Ready(state) => ready_events(events, &mut self.cmd_tx, state).await?,
            Exiting => unreachable!(),
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

fn calibrating(frame: &mut Frame, state: &CalibratingState) {
    let block = Block::default().title("Bradiograph").borders(Borders::ALL);
    let inner = block.inner(frame.size());
    let layout = Layout::new()
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(inner);

    frame.render_widget(block, frame.size());

    let mut text = Text::from(format!("Found bradiograph at id {}.", state.dev.id()));

    if state.left_finished {
        text.extend(Text::from(
            "Position the pen 20cm below and 10cm to the left of the right claw. Press <enter> when finished.",
        ));
    } else {
        text.extend(Text::from(
            "Position the pen 20cm below and 10cm to the right of the left claw. Press <enter> when finished.",
        ));
    }

    frame.render_widget(Paragraph::new(text), layout[0]);

    let chars: Vec<_> = state
        .dev
        .characteristics()
        .into_iter()
        .map(|ch| ListItem::new(ch.uuid.to_string()))
        .collect();
    frame.render_widget(List::new(chars), layout[1]);
}

fn ready(frame: &mut Frame, state: &ReadyState) {
    let block = Block::default().title("Bradiograph").borders(Borders::ALL);
    let inner = block.inner(frame.size());

    let vlayout = Layout::new()
        .constraints([Constraint::Length(1), Constraint::Min(3)])
        .split(inner);

    let hlayout = Layout::new()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
        .split(vlayout[1]);

    frame.render_widget(block, frame.size());
    frame.render_widget(Paragraph::new("Enter a position:"), vlayout[0]);

    let focused_style = Style::new().blue().on_white();
    let unfocused_style = Style::new().white().on_black();
    let (x_style, y_style) = if state.x_focused {
        (focused_style, unfocused_style)
    } else {
        (unfocused_style, focused_style)
    };

    log::debug!("rendering ready: {}, {}", state.x, state.y);

    let x_text = Text::styled(state.x.clone(), x_style);
    let y_text = Text::styled(state.y.clone(), y_style);

    frame.render_widget(
        Paragraph::new(x_text).block(Block::default().title("x").borders(Borders::BOTTOM)),
        hlayout[0],
    );
    frame.render_widget(
        Paragraph::new(y_text).block(Block::default().title("y").borders(Borders::BOTTOM)),
        hlayout[1],
    );
}

async fn loading_events(
    events: &mut (impl Stream<Item = Event> + Unpin),
    mut state: LoadingState,
) -> io::Result<AppState> {
    while let Some(ev) = events.next().await {
        match ev {
            Event::Key(KeyCode::Char('q')) => return Ok(AppState::Exiting),
            Event::Found => {
                state.status = "Found brachiograph, connecting...".to_string();
                return Ok(AppState::Loading(state));
            }
            Event::Disconnected => {
                state.status = LoadingState::default().status;
            }
            Event::Connected(dev) => {
                return Ok(AppState::Calibrating(CalibratingState {
                    dev,
                    left_finished: false,
                }))
            }
            _ => {}
        }
    }
    Ok(AppState::Loading(state))
}

async fn calibrating_events(
    events: &mut (impl Stream<Item = Event> + Unpin),
    cmds: &mut Sender<Cmd>,
    mut state: CalibratingState,
) -> io::Result<AppState> {
    use ManualControl::*;
    let mut done_calibrating = false;

    if let Some(ev) = events.next().await {
        let cmd = match ev {
            Event::Key(KeyCode::Char('q')) => {
                return Ok(AppState::Exiting);
            }
            Event::Key(KeyCode::Char('s')) => Some(Cmd::Manual(ShortenLeft)),
            Event::Key(KeyCode::Char('d')) => Some(Cmd::Manual(StopLeft)),
            Event::Key(KeyCode::Char('f')) => Some(Cmd::Manual(LengthenLeft)),
            Event::Key(KeyCode::Char('l')) => Some(Cmd::Manual(ShortenRight)),
            Event::Key(KeyCode::Char('k')) => Some(Cmd::Manual(StopRight)),
            Event::Key(KeyCode::Char('j')) => Some(Cmd::Manual(LengthenRight)),
            Event::Key(KeyCode::Enter) => {
                if state.left_finished {
                    done_calibrating = true;
                    Some(Cmd::Calibrate(Calibrate::Finish {
                        y_offset: 20.0,
                        x_offset: 10.0,
                    }))
                } else {
                    state.left_finished = true;
                    Some(Cmd::Calibrate(Calibrate::MarkLeft))
                }
            }
            Event::Disconnected => return Ok(AppState::Loading(LoadingState::default())),
            _ => None,
        };
        if let Some(cmd) = cmd {
            cmds.send(cmd).await.unwrap();
        }
        if done_calibrating {
            return Ok(AppState::Ready(ReadyState {
                dev: state.dev,
                x: String::new(),
                y: String::new(),
                x_focused: true,
            }));
        }
    }
    Ok(AppState::Calibrating(state))
}

async fn ready_events(
    events: &mut (impl Stream<Item = Event> + Unpin),
    cmds: &mut Sender<Cmd>,
    mut state: ReadyState,
) -> io::Result<AppState> {
    if let Some(ev) = events.next().await {
        let cmd = match ev {
            Event::Disconnected => return Ok(AppState::Loading(LoadingState::default())),
            Event::Key(KeyCode::Char('q')) => {
                return Ok(AppState::Exiting);
            }
            Event::Key(KeyCode::Char(ch)) if ch.is_ascii_digit() => {
                if state.x_focused {
                    state.x.push(ch);
                } else {
                    state.y.push(ch);
                }
                None
            }
            Event::Key(KeyCode::Backspace) => {
                if state.x_focused {
                    state.x.pop();
                } else {
                    state.y.pop();
                }
                None
            }
            Event::Key(KeyCode::Tab) => {
                state.x_focused = !state.x_focused;
                None
            }
            Event::Key(KeyCode::Enter) => {
                if state.x_focused {
                    state.x_focused = false;
                    None
                } else {
                    let x: f32 = state.x.parse().unwrap_or(0.0);
                    let y: f32 = state.y.parse().unwrap_or(0.0);
                    Some(Cmd::MoveTo(x, y))
                }
            }
            _ => None,
        };
        if let Some(cmd) = cmd {
            cmds.send(cmd).await.unwrap();
        }
    }
    Ok(AppState::Ready(state))
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
    simple_logging::log_to_file("debug.log", LevelFilter::Debug).unwrap();
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
