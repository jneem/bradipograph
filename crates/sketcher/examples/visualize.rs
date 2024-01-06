use anyhow::anyhow;
use bradipous_sketcher::Zigzag;
use bradipous_sketcher::{clip_path, load_svg, transform};
use clap::value_parser;
use clap::Parser as _;
use kurbo::{BezPath, Shape as _};
use piet::{Color, RenderContext as _};
use std::{path::PathBuf, str::FromStr};

impl FromStr for Scale {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let (before, after) = s.split_once(',').ok_or_else(|| anyhow!("no comma"))?;

        Ok(Scale {
            width: before.parse()?,
            height: after.parse()?,
        })
    }
}

#[derive(Clone, Debug)]
struct Scale {
    width: f64,
    height: f64,
}

#[derive(clap::Parser, Debug)]
struct Args {
    #[arg(long)]
    svg: Option<PathBuf>,

    #[arg(long, value_parser=value_parser!(Scale))]
    scale: Option<Scale>,

    #[arg(long)]
    output: PathBuf,
}

fn extrema(xs: impl Iterator<Item = f64> + Clone) -> (f64, f64) {
    let max = xs.clone().max_by(|x, y| x.partial_cmp(y).unwrap()).unwrap();
    let min = xs.clone().min_by(|x, y| x.partial_cmp(y).unwrap()).unwrap();
    (min, max)
}

pub fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    let clip = if let Some(svg) = args.svg {
        let mut path = load_svg(&svg)?;
        if let Some(scale) = args.scale {
            transform(
                &mut path,
                &kurbo::Rect::new(0.0, 0.0, scale.width, scale.height),
            );
        }
        path
    } else {
        kurbo::Circle::new((0.0, 0.0), 100.0).into_path(0.01)
    };
    //let clip = kurbo::Rect::new(-60.0, -80.0, 70.0, 80.0).into_path(0.01);
    //let zz_box = kurbo::Rect::new(-80.0, -80.0, 80.0, 80.0);
    let zz_box = clip.bounding_box();
    let mut zigzag = Zigzag::default().points(&zz_box).into_iter();
    let mut zigzag_path = BezPath::new();
    zigzag_path.move_to(zigzag.next().unwrap());
    for p in zigzag {
        zigzag_path.line_to(p);
    }

    let poly = bradipous_sketcher::to_polygon(&clip, 0.1);
    let clipped = clip_path(&zigzag_path, &poly, 0.1);

    let points: Vec<_> = clipped.iter().flat_map(|lines| lines.points()).collect();
    let (x_min, x_max) = extrema(points.iter().map(|p| p.x()));
    let (y_min, y_max) = extrema(points.iter().map(|p| p.y()));

    let mut piet = piet_svg::RenderContext::new(piet::kurbo::Size::new(
        x_max - x_min + 2.0,
        y_max - y_min + 2.0,
    ));

    let pt = |p: &geo_types::Point<f64>| {
        piet::kurbo::Point::new(p.x() - x_min + 1.0, p.y() - y_min + 1.0)
    };
    for p in &points {
        let p = pt(p);
        piet.fill(piet::kurbo::Circle::new(p, 0.2), &Color::BLUE);
    }
    for line in clipped.iter().flat_map(|lines| lines.lines()) {
        let p0 = pt(&line.start.into());
        let p1 = pt(&line.end.into());
        let line = piet::kurbo::Line::new(p0, p1);
        piet.stroke(line, &Color::BLACK, 0.05);
    }

    piet.finish().unwrap();
    let file = std::fs::File::create(args.output)?;
    piet.write(file)?;
    Ok(())
}
