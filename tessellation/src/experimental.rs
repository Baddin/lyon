//use std::mem::{replace, swap};
use std::cmp::Ordering;

use {FillOptions, FillRule};
use geom::math::*;
use geom::LineSegment;
use geometry_builder::VertexId;
use std::ops::Range;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct SegmentId(u16);

impl SegmentId {
    fn from_usize(idx: usize) -> Self { SegmentId(idx as u16) }
    fn to_usize(&self) -> usize { self.0 as usize }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct Segment {
    from: VertexId,
    to: VertexId,
    ctrl: VertexId,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct SubPathInfo {
    range: Range<usize>,
    is_closed: bool,
}

#[derive(Clone, Debug)]
pub struct Path {
    points: Vec<Point>,
    segments: Vec<Segment>,
    sub_paths: Vec<SubPathInfo>,
}

impl Path {
    pub fn new() -> Self {
        Path {
            points: Vec::new(),
            segments: Vec::new(),
            sub_paths: Vec::new(),
        }
    }

    fn previous_segment(&self, id: SegmentId) -> SegmentId {
        let idx = id.0 as usize;
        for sp in &self.sub_paths {
            if sp.range.start > idx || sp.range.end <= idx {
                continue;
            }

            return SegmentId::from_usize(
                if idx == sp.range.start { sp.range.end - 1 } else { idx - 1 }
            );
        }

        // invalid id?
        panic!();
    }

    fn next_segment(&self, id: SegmentId) -> SegmentId {
        let idx = id.0 as usize;
        for sp in &self.sub_paths {
            if sp.range.start > idx || sp.range.end <= idx {
                continue;
            }

            return SegmentId::from_usize(
                if idx == sp.range.end - 1 { sp.range.start } else { idx + 1 }
            );
        }

        // invalid id?
        panic!();
    }

    fn segment_from(&self, id: SegmentId) -> VertexId {
        self.segments[id.to_usize()].from
    }

    fn segment_ctrl(&self, id: SegmentId) -> VertexId {
        self.segments[id.to_usize()].ctrl
    }

    fn segment_to(&self, id: SegmentId) -> VertexId {
        let id = self.next_segment(id);
        self.segments[id.to_usize()].from
    }

    fn point(&self, id: VertexId) -> Point {
        self.points[id.0 as usize]
    }


    fn sort(&self, events: &mut Vec<Event>) {
        for sub_path in &self.sub_paths {
            if sub_path.range.end - sub_path.range.start < 2 {
                continue;
            }
            for i in sub_path.range.clone() {
                events.push(Event {
                    vertex: self.segments[i].from,
                    segment: SegmentId::from_usize(i),
                });
            }
        }

        events.sort_by(|a, b| {
            compare_positions(
                self.points[b.vertex.0 as usize],
                self.points[a.vertex.0 as usize],
            )
        });
    }
}

struct Event {
    vertex: VertexId,
    segment: SegmentId,
}

pub struct Builder {
    path: Path,
}

impl Builder {
    pub fn new() -> Self {
        Builder {
            path: Path::new(),
        }
    }

    pub fn quadartic_bezier_to(&mut self, ctrl_pos: Point, to_pos: Point) {
        if self.path.points.is_empty() {
            self.path.points.push(point(0.0, 0.0));
        }

        let from = VertexId((self.path.points.len() - 1) as u16);
        let ctrl = VertexId(from.0 + 1);
        let to = VertexId(from.0 + 2);

        self.path.points.push(ctrl_pos);
        self.path.points.push(to_pos);

        self.path.segments.push(Segment{ from, ctrl, to });
    }

    pub fn line_to(&mut self, to_pos: Point) {
        if self.path.points.is_empty() {
            self.path.points.push(point(0.0, 0.0));
        }

        let from = VertexId((self.path.points.len() - 1) as u16);
        let ctrl = from;
        let to = VertexId(from.0 + 1);

        self.path.points.push(to_pos);

        self.path.segments.push(Segment{ from, ctrl, to });
    }

    pub fn move_to(&mut self, to_pos: Point) {
        self.end_sub_path(false);
        self.path.points.push(to_pos);
    }

    pub fn close(&mut self) {
        self.end_sub_path(true);
    }

    fn end_sub_path(&mut self, is_closed: bool) {
        let mut sp_end = self.path.segments.len();
        let sp_start = self.path.sub_paths.last()
            .map(|sp| sp.range.end)
            .unwrap_or(0);

        if sp_end >= sp_start {
            if is_closed && !self.path.points.is_empty() {
                let first = self.path.sub_paths.last().map(|sp|{
                    self.path.segments[sp.range.start].from.0 as usize
                }).unwrap_or(0);
                let first_point = self.path.points[first];
                self.line_to(first_point);

                sp_end += 1;
            }

            self.path.sub_paths.push(SubPathInfo {
                range: sp_start..sp_end,
                is_closed,
            });
        }
    }

    pub fn build(self) -> Path {
        self.path
    }
}

pub struct FillTessellator {
    active: ActiveEdges,
    pending_edges: Vec<PendingEdge>,
    options: FillOptions,
    events: Vec<Event>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum Transition {
    In,
    Out,
    None,
}

#[derive(Copy, Clone, Debug)]
struct WindingState {
    span_index: usize,
    number: i16,
    transition: Transition,
}

struct ActiveEdge {
    from: Point,
    to: Point,
    ctrl: Point,

    range_start: f32,

    winding: i16,
    is_merge: bool,
}

struct ActiveEdgeAux {
    from_id: VertexId,
    ctrl_id: VertexId,
    to_id: VertexId,
}

struct ActiveEdges {
    edges: Vec<ActiveEdge>,
    aux: Vec<ActiveEdgeAux>,
}

impl ActiveEdges {
    fn insert(&mut self, idx: usize, e: &PendingEdge) {
        self.edges.insert(idx, ActiveEdge {
            from: e.from,
            to: e.to,
            ctrl: e.ctrl,
            range_start: e.range_start,
            winding: e.winding,
            is_merge: false,
        });

        self.aux.insert(idx, ActiveEdgeAux {
            from_id: e.from_id,
            to_id: e.to_id,
            ctrl_id: e.ctrl_id,
        });
    }

    fn remove(&mut self, range: Range<usize>) {
        let count = range.end - range.start;
        for _ in 0..count {
            self.edges.remove(range.start);
            self.aux.remove(range.start);
        }
    }
}

struct PendingEdge {
    from: Point,
    to: Point,
    ctrl: Point,

    range_start: f32,
    angle: f32,

    from_id: VertexId,
    ctrl_id: VertexId,
    to_id: VertexId,

    winding: i16,
}

impl ActiveEdge {
    fn solve_x_for_y(&self, y: f32) -> f32 {
        // TODO: curves.
        LineSegment {
            from: self.from,
            to: self.to,
        }.solve_x_for_y(y)
    }
}

impl FillTessellator {
    pub fn new() -> Self {
        FillTessellator {
            active: ActiveEdges {
                edges: Vec::new(),
                aux: Vec::new(),
            },
            pending_edges: Vec::new(),
            options: FillOptions::default(),
            events: Vec::new(),
        }
    }

    pub fn tessellate_path(
        &mut self,
        path: &Path,
        options: &FillOptions,
    ) {
        self.options = options.clone();

        path.sort(&mut self.events);

        self.tessellator_loop(path);
    }

    fn tessellator_loop(&mut self, path: &Path) {
        while let Some(event) = self.events.pop() {
            let segment_id_a = event.segment;
            let current_vertex_id = path.segment_from(segment_id_a);
            let segment_id_b = path.previous_segment(segment_id_a);
            let endpoint_id_b = path.segment_from(segment_id_b);
            let endpoint_id_a = path.segment_to(segment_id_a);

            let endpoint_pos_a = path.point(endpoint_id_a);
            let endpoint_pos_b = path.point(endpoint_id_b);
            let current_pos = path.point(current_vertex_id);

            let after_a = is_after(current_pos, endpoint_pos_a);
            let after_b = is_after(current_pos, endpoint_pos_b);

            let mut edges_above = 0;

            if after_a {
                edges_above += 1;
            } else {
                let ctrl_id_a = path.segment_ctrl(segment_id_a);
                self.pending_edges.push(PendingEdge {
                    from: current_pos,
                    ctrl: path.point(ctrl_id_a),
                    to: endpoint_pos_a,

                    range_start: 0.0,
                    angle: (endpoint_pos_a - current_pos).angle_from_x_axis().radians,

                    from_id: current_vertex_id,
                    ctrl_id: ctrl_id_a,
                    to_id: endpoint_id_a,

                    winding: 1,
                });
            }

            if after_b {
                edges_above += 1;
            } else {
                let ctrl_id_b = path.segment_ctrl(segment_id_b);
                self.pending_edges.push(PendingEdge {
                    from: current_pos,
                    ctrl: path.point(ctrl_id_b),
                    to: endpoint_pos_b,

                    range_start: 0.0,
                    angle: (endpoint_pos_b - current_pos).angle_from_x_axis().radians,

                    from_id: current_vertex_id,
                    ctrl_id: ctrl_id_b,
                    to_id: endpoint_id_b,

                    winding: -1,
                });
            }

            self.process_events(
                current_pos, current_vertex_id,
                edges_above,
            );
        }
    }

    fn process_events(
        &mut self,
        current_pos: Point,
        _current_vertex_id: VertexId,
        edges_above: u32,
    ) {
        println!("");
        println!(" --- events at [{}, {}]                       {} -> {}",
            current_pos.x, current_pos.y,
            edges_above, self.pending_edges.len(),
        );

        let mut winding = WindingState {
            span_index: 0,
            number: 0,
            transition: Transition::None,
        };
        let mut winding_below = None;
        let mut above = self.active.edges.len()..self.active.edges.len();
        let mut connecting_edges = false;
        let mut first_transition_above = true;
        let mut pending_merge = None;
        let mut pending_right = None;
        let mut prev_transition_in = None;

        // First go through the sweep line until we find an edge that touches
        // the current position.
        for (i, active_edge) in self.active.edges.iter().enumerate() {
            if active_edge.is_merge {
                continue;
            }

            let was_connecting_edges = connecting_edges;

            if self.points_are_equal(current_pos, active_edge.to) {
                if !connecting_edges {
                    debug_assert!(edges_above != 0);
                    connecting_edges = true;
                }
            } else {
                let ex = active_edge.solve_x_for_y(current_pos.y);
                println!("ex: {}", ex);

                if ex == current_pos.x {
                    connecting_edges = true;
                    // TODO
                    unimplemented!();
                }

                if ex > current_pos.x {
                    above.end = i;
                    break;
                }
            }

            if !was_connecting_edges && connecting_edges {
                println!("begin connecting edges");
                winding_below = Some(winding.clone());
                above.start = i;
            }

            self.update_winding(&mut winding, active_edge.winding);

            if !connecting_edges {
                continue;
            }

            println!("{:?}", winding.transition);

            match winding.transition {
                Transition::Out => {
                    if first_transition_above {
                        if self.pending_edges.is_empty() {
                            // Merge event.
                            pending_merge = Some(i);
                        } else {
                            // Right event.
                            pending_right = Some(i);
                        }
                    } else if let Some(in_idx) = prev_transition_in.take() {

                        println!(" ** end ** edges: [{}, {}] span: {}",
                            in_idx, i,
                            winding.span_index
                        );

                    } else {
                        // TODO: ??
                        unimplemented!();
                    }
                }
                Transition::In => {
                    prev_transition_in = Some(i)
                }
                Transition::None => {}
            }

            if winding.transition != Transition::None {
                first_transition_above = false;
            }
        }

        // Fix up above index range in case there was no connecting edges.
        above.start = usize::min(above.start, above.end);

        println!("connecting edges: {}..{}", above.start, above.end);

        let mut winding = winding_below.unwrap_or(winding);
        let mut prev_transition_in = None;

        self.sort_ending_edges();

        if let Some(idx) = pending_right {
            // Right event.
            //
            //   ..../
            //   ...x
            //   ....\
            //

            println!(" ** right ** edge: {} span: {}", idx, winding.span_index);

        } else if let Some(in_idx) = pending_merge {
            // Merge event.
            //
            // ...\   /...
            // ....\ /....
            // .....x.....
            //

            println!(" ** merge ** edges: [{}, {}] span: {}",
                in_idx, above.end - 1,
                winding.span_index
            );

        } else if !self.is_inside(winding.number) && self.pending_edges.len() % 2 == 1 {
            // Left event.
            //
            //     /...
            //    x....
            //     \...
            //

            println!(" ** left ** edge {} span: {}", above.start, winding.span_index);
        }

        // Go through the edges starting at the current point and emmit
        // start events.

        for (i, pending_edge) in self.pending_edges.iter().enumerate() {
            self.update_winding(&mut winding, pending_edge.winding);

            match winding.transition {
                Transition::In => {
                    prev_transition_in = Some(i);
                }
                Transition::Out => {
                    if let Some(in_idx) = prev_transition_in {

                        println!(" ** start ** edges: [{}, {}] span: {}", in_idx, i, winding.span_index);

                    }
                }
                Transition::None => {}
            }
        }

        self.update_active_edges(above);

        println!("sweep line: {}", self.active.edges.len());
        for e in &self.active.edges {
            println!("| {} -> {}", e.from, e.to);
        }
    }

    fn update_active_edges(&mut self, above: Range<usize>) {

        let first_edge_below = above.start;

        self.active.remove(above);

        for (i, pending_edge) in self.pending_edges.drain(..).enumerate() {
            self.active.insert(first_edge_below + i, &pending_edge);
        }
    }

    fn sort_ending_edges(&mut self) {
        self.pending_edges.sort_by(|a, b| {
            b.angle.partial_cmp(&a.angle).unwrap_or(Ordering::Equal)
        });
    }

    fn points_are_equal(&self, a: Point, b: Point) -> bool {
        // TODO: Use the tolerance threshold.
        a == b
    }

    fn is_inside(&self, winding_number: i16) -> bool {
        match self.options.fill_rule {
            FillRule::EvenOdd => { winding_number % 2 != 0 }
            FillRule::NonZero => { winding_number != 0 }
        }
    }

    fn transition(&self, prev_winding: i16, new_winding: i16) -> Transition {
        match (self.is_inside(prev_winding), self.is_inside(new_winding)) {
            (false, true) => Transition::In,
            (true, false) => Transition::Out,
            _ => Transition::None,
        }
    }

    fn update_winding(&self, winding: &mut WindingState, edge_winding: i16) {
        let prev_winding_number = winding.number;
        winding.number += edge_winding;
        winding.transition = self.transition(prev_winding_number, winding.number);
        if winding.transition == Transition::In {
            winding.span_index += 1;
        }
    }
}

fn compare_positions(a: Point, b: Point) -> Ordering {
    if a.y > b.y {
        return Ordering::Greater;
    }
    if a.y < b.y {
        return Ordering::Less;
    }
    if a.x > b.x {
        return Ordering::Greater;
    }
    if a.x < b.x {
        return Ordering::Less;
    }
    return Ordering::Equal;
}

#[inline]
fn is_after(a: Point, b: Point) -> bool {
    a.y > b.y || (a.y == b.y && a.x > b.x)
}

#[test]
fn new_tess() {
    println!("");

    let mut builder = Builder::new();
    builder.move_to(point(0.0, 0.0));
    builder.line_to(point(5.0, -5.0));
    builder.line_to(point(10.0, 0.0));
    builder.line_to(point(9.0, 5.0));
    builder.line_to(point(10.0, 10.0));
    builder.line_to(point(5.0, 6.0));
    builder.line_to(point(0.0, 10.0));
    builder.line_to(point(1.0, 5.0));
    builder.close();

    builder.move_to(point(20.0, -1.0));
    builder.line_to(point(25.0, 1.0));
    builder.line_to(point(25.0, 9.0));
    builder.close();


    let path = builder.build();

    let mut tess = FillTessellator::new();

    tess.tessellate_path(&path, &FillOptions::default());

    panic!();
}
