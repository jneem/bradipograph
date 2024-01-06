use geo_booleanop::boolean::{
    fill_queue::fill_queue, subdivide_segments::subdivide, BoundingBox, Float, Operation,
};
use geo_types::{Coord, LineString, MultiPolygon, Polygon};

pub fn decompose<F>(non_poly: &[LineString<F>]) -> MultiPolygon<F>
where
    F: Float,
{
    let mut sbbox = BoundingBox {
        min: Coord {
            x: F::infinity(),
            y: F::infinity(),
        },
        max: Coord {
            x: F::neg_infinity(),
            y: F::neg_infinity(),
        },
    };
    let mut cbbox = sbbox;
    let fake_polys = non_poly
        .iter()
        .cloned()
        .map(|lines| Polygon::new(lines, vec![]))
        .collect::<Vec<_>>();

    let mut event_queue = fill_queue(&fake_polys, &[], &mut sbbox, &mut cbbox, Operation::Union);

    let sorted_events = subdivide(&mut event_queue, &sbbox, &cbbox, Operation::Union);

    let contours = crate::connect_edges::connect_edges(&sorted_events);

    // Convert contours into polygons
    let polygons: Vec<Polygon<F>> = contours
        .iter()
        .filter(|contour| contour.is_exterior())
        .map(|contour| {
            let exterior = LineString(contour.points.clone());
            let mut interios: Vec<LineString<F>> = Vec::new();
            for hole_id in &contour.hole_ids {
                interios.push(LineString(contours[*hole_id as usize].points.clone()));
            }
            Polygon::new(exterior, interios)
        })
        .collect();

    MultiPolygon(polygons)
}
