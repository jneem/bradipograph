use anyhow::anyhow;
use bradipous_sketcher::Zigzag;
use bradipous_sketcher::{load_svg, transform};
use clap::value_parser;
use clap::Parser as _;
use kurbo::{ParamCurve as _, Shape as _};
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
    let orig_bbox = clip.bounding_box().inset(1.0);
    let transform = kurbo::TranslateScale::translate((-orig_bbox.x0, -orig_bbox.y0));
    let bbox = transform * orig_bbox;
    let clip = transform * clip;

    let polys = bradipous_sketcher::to_polygons(&clip, 0.1);
    let clipped = Zigzag::default().clipped_to(&clip);

    let points: Vec<_> = clipped
        .iter()
        .flat_map(|path| path.segments().map(|seg| seg.eval(0.0)))
        .collect();

    let mut piet = piet_svg::RenderContext::new(piet::kurbo::Size {
        width: bbox.width(),
        height: bbox.height(),
    });

    let pt = |p: &kurbo::Point| piet::kurbo::Point::new(p.x, p.y);
    for p in &points {
        let p = pt(p);
        piet.fill(piet::kurbo::Circle::new(p, 0.2), &Color::BLUE);
    }
    for p in &clipped {
        for line in p.segments() {
            let kurbo::PathSeg::Line(line) = line else {
                panic!("expected a polyline");
            };
            let line = piet::kurbo::Line::new((line.p0.x, line.p0.y), (line.p1.x, line.p1.y));
            piet.stroke(line, &Color::BLACK, 0.05);
        }
    }

    let poly_colors = [
        Color::PURPLE,
        Color::RED,
        Color::BLUE,
        Color::GREEN,
        Color::FUCHSIA,
    ]
    .into_iter()
    .cycle();
    let pt = |p: &geo_types::Coord| piet::kurbo::Point::new(p.x, p.y);
    for (poly, color) in polys.0.iter().zip(poly_colors) {
        for line in poly.exterior().lines() {
            let p0 = pt(&line.start);
            let p1 = pt(&line.end);
            let line = piet::kurbo::Line::new(p0, p1);
            piet.stroke(line, &color, 0.15);
        }
    }

    piet.finish().unwrap();
    let file = std::fs::File::create(args.output)?;
    piet.write(file)?;
    Ok(())
}
