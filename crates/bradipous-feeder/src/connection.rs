use anyhow::anyhow;
use btleplug::{
    api::{Central, Manager as _, Peripheral as _, ScanFilter},
    platform::{Manager, Peripheral},
};
use crossterm::event::{self, EventStream, KeyCode};
use futures::Stream;
use tokio::sync::mpsc;
use tokio_stream::wrappers::UnboundedReceiverStream;
use tokio_stream::StreamExt as _;
use uuid::Uuid;

use bradipous_protocol::{Calibration, ManualControl};

pub enum Event {
    Found,
    Disconnected,
    Connected(Peripheral),
    Error(anyhow::Error),
    Key(char),
}

#[derive(Debug)]
pub enum Cmd {
    Calibration(CalibrationCmd),
}

/// The commands that are valid when we're in calibration mode.
#[derive(Debug)]
pub enum CalibrationCmd {
    Move(ManualControl),
    Calib(Calibration),
}

const CONTROL_UUID: &str = "68a79628-2609-4569-8d7d-3b29fde28877";

pub fn init(cmds: mpsc::Receiver<Cmd>) -> impl Stream<Item = Event> + Unpin {
    let (event_tx, event_rx) = mpsc::unbounded_channel();
    tokio::spawn(ble_connection(cmds, event_tx));

    let event_stream = EventStream::new().filter_map(|ev| {
        if let Ok(event::Event::Key(key)) = ev {
            if key.kind == event::KeyEventKind::Press {
                if let KeyCode::Char(ch) = key.code {
                    return Some(Event::Key(ch));
                }
            }
        }
        None
    });

    UnboundedReceiverStream::new(event_rx).merge(event_stream)
}

pub async fn ble_connection(
    mut cmds: mpsc::Receiver<Cmd>,
    events: mpsc::UnboundedSender<Event>,
) -> anyhow::Result<()> {
    let manager = Manager::new().await?;
    let adapter = manager
        .adapters()
        .await?
        .into_iter()
        .next()
        .ok_or(anyhow!("no bluetooth adapter"))?;
    adapter.start_scan(ScanFilter::default()).await?;

    'reconnect: loop {
        events.send(Event::Disconnected).unwrap();

        let peripheral;
        'search: loop {
            let peripherals = adapter.peripherals().await?;
            for p in peripherals {
                match p.properties().await {
                    Ok(Some(props)) => {
                        if props.local_name.as_deref() == Some("Bradipograph") {
                            peripheral = p;
                            break 'search;
                        }
                    }
                    Ok(None) => {}
                    Err(e) => events.send(Event::Error(e.into())).unwrap(),
                }
            }
        }
        events.send(Event::Found).unwrap();

        if let Err(e) = peripheral.connect().await {
            events.send(Event::Error(e.into())).unwrap();
            continue 'reconnect;
        };

        if let Err(e) = peripheral.discover_services().await {
            events.send(Event::Error(e.into())).unwrap();
            continue 'reconnect;
        };
        events.send(Event::Connected(peripheral.clone())).unwrap();

        let control_uuid = Uuid::parse_str(CONTROL_UUID).unwrap();
        let control = match peripheral
            .characteristics()
            .into_iter()
            .find(|ch| ch.uuid == control_uuid)
        {
            Some(c) => c,
            None => {
                events
                    .send(Event::Error(anyhow!(
                        "Bradipous was missing the characteristic {}",
                        control_uuid
                    )))
                    .unwrap();
                continue 'reconnect;
            }
        };

        while let Some(cmd) = cmds.recv().await {
            match cmd {
                Cmd::Calibration(_) => todo!(),
                _ => {
                    events
                        .send(Event::Error(anyhow!(
                            "invalid command {:?} during calibration",
                            cmd
                        )))
                        .unwrap();
                }
            }
        }
    }
}
