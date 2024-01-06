use geo_booleanop::boolean::sweep_event::SweepEvent;
use geo_booleanop::boolean::{BoundingBox, Float, Operation};
use geo_types::{Coord, LineString, Polygon};
use std::collections::{BinaryHeap, HashSet};
use std::rc::{Rc, Weak};

pub fn fill_queue_lines<F>(
    subject: &LineString<F>,
    clipping: &[Polygon<F>],
    sbbox: &mut BoundingBox<F>,
    cbbox: &mut BoundingBox<F>,
    operation: Operation,
) -> BinaryHeap<Rc<SweepEvent<F>>>
where
    F: Float,
{
    let mut event_queue: BinaryHeap<Rc<SweepEvent<F>>> = BinaryHeap::new();
    let mut contour_id = 1u32;

    process_polygon(subject, true, contour_id, &mut event_queue, sbbox, true);

    for polygon in clipping {
        let exterior = operation != Operation::Difference;
        if exterior {
            contour_id += 1;
        }
        process_polygon(
            polygon.exterior(),
            false,
            contour_id,
            &mut event_queue,
            cbbox,
            exterior,
        );
        for interior in polygon.interiors() {
            process_polygon(interior, false, contour_id, &mut event_queue, cbbox, false);
        }
    }

    event_queue
}

fn process_polygon<F>(
    contour_or_hole: &LineString<F>,
    is_subject: bool,
    contour_id: u32,
    event_queue: &mut BinaryHeap<Rc<SweepEvent<F>>>,
    bbox: &mut BoundingBox<F>,
    is_exterior_ring: bool,
) where
    F: Float,
{
    for line in contour_or_hole.lines() {
        if line.start == line.end {
            continue; // skip collapsed edges
        }

        let e1 = SweepEvent::new_rc(
            contour_id,
            line.start,
            false,
            Weak::new(),
            is_subject,
            is_exterior_ring,
        );
        let e2 = SweepEvent::new_rc(
            contour_id,
            line.end,
            false,
            Rc::downgrade(&e1),
            is_subject,
            is_exterior_ring,
        );
        e1.set_other_event(&e2);

        if e1 < e2 {
            e2.set_left(true)
        } else {
            e1.set_left(true)
        }

        bbox.min.x = bbox.min.x.min(line.start.x);
        bbox.min.y = bbox.min.y.min(line.start.y);
        bbox.max.x = bbox.max.x.max(line.start.x);
        bbox.max.y = bbox.max.y.max(line.start.y);

        event_queue.push(e1);
        event_queue.push(e2);
    }
}

fn order_events<F>(sorted_events: &[Rc<SweepEvent<F>>]) -> Vec<Rc<SweepEvent<F>>>
where
    F: Float,
{
    let mut result_events: Vec<Rc<SweepEvent<F>>> = Vec::new();

    for event in sorted_events {
        if (event.is_left() && event.is_in_result())
            || (!event.is_left()
                && event
                    .get_other_event()
                    .map(|o| o.is_in_result())
                    .unwrap_or(false))
        {
            result_events.push(event.clone());
        }
    }

    let mut sorted = false;
    while !sorted {
        sorted = true;
        for i in 1..result_events.len() {
            if result_events[i - 1] < result_events[i] {
                result_events.swap(i - 1, i);
                sorted = false;
            }
        }
    }

    // Populate `other_pos` by initializing with index and swapping with other event.
    for (pos, event) in result_events.iter().enumerate() {
        event.set_other_pos(pos as i32)
    }
    for event in &result_events {
        if event.is_left() {
            if let Some(other) = event.get_other_event() {
                let (a, b) = (event.get_other_pos(), other.get_other_pos());
                event.set_other_pos(b);
                other.set_other_pos(a);
            }
        }
    }

    result_events
}

/// Helper function that identifies groups of sweep event that belong to one
/// vertex, and precomputes in which order the events within one group should
/// be iterated. The result is a vector with the semantics:
///
/// map[i] = index of next event belonging to vertex
///
/// Iteration order is in positive index direction for right events, but in
/// reverse index direction for left events in order to ensure strict clockwise
/// iteration around the vertex.
#[allow(clippy::needless_range_loop)] // Check has false positive here
fn precompute_iteration_order<T, I, L>(data: &[T], is_identical: I, is_left: L) -> Vec<usize>
where
    I: Fn(&T, &T) -> bool,
    L: Fn(&T) -> bool,
{
    let mut map = vec![0; data.len()];

    let mut i = 0;
    while i < data.len() {
        let x_ref = &data[i];

        // Find index range of R events
        let r_from = i;
        while i < data.len() && is_identical(x_ref, &data[i]) && !is_left(&data[i]) {
            i += 1;
        }
        let r_upto_exclusive = i;

        // Find index range of L event
        let l_from = i;
        while i < data.len() && is_identical(x_ref, &data[i]) {
            debug_assert!(is_left(&data[i]));
            i += 1;
        }
        let l_upto_exclusive = i;

        let has_r_events = r_upto_exclusive > r_from;
        let has_l_events = l_upto_exclusive > l_from;

        if has_r_events {
            let r_upto = r_upto_exclusive - 1;
            // Connect elements in [r_from, r_upto) to larger index
            for j in r_from..r_upto {
                map[j] = j + 1;
            }
            // Special handling of *last* element: Connect either the last L event
            // or loop back to start of R events (if no L events).
            if has_l_events {
                map[r_upto] = l_upto_exclusive - 1;
            } else {
                map[r_upto] = r_from;
            }
        }
        if has_l_events {
            let l_upto = l_upto_exclusive - 1;
            // Connect elements in (l_from, l_upto] to lower index
            for j in l_from + 1..=l_upto {
                map[j] = j - 1;
            }
            // Special handling of *first* element: Connect either to the first R event
            // or loop back to end of L events (if no R events).
            if has_r_events {
                map[l_from] = r_from;
            } else {
                map[l_from] = l_upto;
            }
        }
    }

    map
}

fn get_next_pos_unprocessed(
    pos: i32,
    unprocessed: &HashSet<i32>,
    iteration_map: &[usize],
) -> Option<i32> {
    let mut pos = pos;
    let start_pos = pos;

    loop {
        pos = iteration_map[pos as usize] as i32;
        if pos == start_pos {
            // Entire group is already processed?
            return None;
        } else if unprocessed.contains(&pos) {
            return Some(pos);
        }
    }
}

pub fn connect_edges_to_events<F>(sorted_events: &[Rc<SweepEvent<F>>]) -> Vec<Vec<SweepEvent<F>>>
where
    F: Float,
{
    let result_events = order_events(sorted_events);

    let iteration_map =
        precompute_iteration_order(&result_events, |a, b| a.point == b.point, |e| e.is_left());
    let mut contours = Vec::new();
    let mut unprocessed: HashSet<i32> = (0..(result_events.len() as i32)).collect();

    let mut pos = 0;
    while !unprocessed.is_empty() {
        let mut contour = Vec::new();

        let initial = result_events[pos as usize].point;
        contour.push(result_events[pos as usize].as_ref().clone());

        loop {
            // Loop clarifications:
            // - An iteration has two kinds of `pos` advancements:
            //   (A) following a segment via `other_pos`, and
            //   (B) searching for the next outgoing edge on same point.
            // - Therefore, the loop contains two "mark pos as processed" steps, using the
            //   convention that at beginning of the loop, `pos` isn't marked yet.
            // - The contour is extended after following a segment.
            // - Hitting pos == orig_pos after search (B) indicates no continuation and
            //   terminates the loop.
            unprocessed.remove(&pos);

            // pos advancement (A)
            pos = result_events[pos as usize].get_other_pos();

            unprocessed.remove(&pos);
            contour.push(result_events[pos as usize].as_ref().clone());

            // pos advancement (B)
            let next_pos_opt = get_next_pos_unprocessed(pos, &unprocessed, &iteration_map);
            match next_pos_opt {
                Some(npos) => {
                    pos = npos;
                }
                None => {
                    break;
                }
            }

            // Optional: Terminate contours early (to avoid overly long contours that
            // may mix clockwise and counter-clockwise winding rules, which can be more
            // difficult to handle in some use cases).
            if result_events[pos as usize].point == initial {
                break;
            }
        }

        // This assert should be possible once the first stage of the algorithm is robust.
        // debug_assert_eq!(contour.points.first(), contour.points.last());

        let next_pos = if let Some(last_on_line) = contour
            .iter()
            .rposition(|ev| ev.is_subject && ev.get_other_event().map_or(false, |o| o.is_subject))
        {
            contour.truncate(last_on_line + 1);
            let last_point = contour[last_on_line].point;
            contours.push(contour);

            let sqnorm = |p: Coord<F>| p.x * p.x + p.y * p.y;

            unprocessed
                .iter()
                .copied()
                .filter(|i| result_events[*i as usize].is_subject)
                .min_by(|i, j| {
                    let i_dist = sqnorm(result_events[*i as usize].point - last_point);
                    let j_dist = sqnorm(result_events[*j as usize].point - last_point);
                    i_dist.partial_cmp(&j_dist).unwrap()
                })
        } else {
            unprocessed
                .iter()
                .copied()
                .min_by_key(|i| &result_events[*i as usize])
        };
        pos = match next_pos {
            Some(p) => p,
            None => break,
        };
    }

    contours
}
