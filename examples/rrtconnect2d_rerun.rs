use motion_planning::{
    rrt_connect::RRTConnect,
    utils::{CubeSpace, Rect, SimpleTree}, Tree,
};
use nalgebra::Vector2;
use rerun::{Boxes2D, Color, Points2D};

fn main() {
    let rec = rerun::RecordingStreamBuilder::new("rrt-connect").spawn().unwrap();

    let space = CubeSpace {
        width: 10.0,
        height: 10.0,
        obstacles: vec![
            Rect::new(Vector2::new(2.0, 2.0), Vector2::new(5.0, 5.0)),
            Rect::new(Vector2::new(6.0, 6.0), Vector2::new(8.0, 7.0)),
        ],
    };

    rec.log(
        "space",
        &Boxes2D::from_centers_and_sizes(
            [((space.width / 2.0) as f32, (space.height / 2.0) as f32)],
            [(space.width as f32, space.height as f32)],
        ),
    )
    .unwrap();
    rec.log(
        "obstacles",
        &Boxes2D::from_centers_and_sizes(
            space.obstacles.iter().map(|obstacle| {
                (
                    ((obstacle.min.x + obstacle.max.x) / 2.0) as f32,
                    ((obstacle.min.y + obstacle.max.y) / 2.0) as f32,
                )
            }),
            space.obstacles.iter().map(|obstacle| {
                (
                    (obstacle.max.x - obstacle.min.x) as f32,
                    (obstacle.max.y - obstacle.min.y) as f32,
                )
            }),
        ),
    )
    .unwrap();

    let start = Vector2::new(1.0, 1.0);
    rec.log(
        "start",
        &Points2D::new([(start.x as f32, start.y as f32)]).with_radii([0.05]),
    )
    .unwrap();

    let goal = Vector2::new(9.0, 9.0);
    rec.log(
        "goal",
        &Points2D::new([(goal.x as f32, goal.y as f32)])
            .with_radii([0.05]),
    )
    .unwrap();

    let mut start_lines: Vec<[[f32; 2]; 2]> = vec![];
    let mut start_points: Vec<(f32, f32)> = vec![];

    let mut goal_lines: Vec<[[f32; 2]; 2]> = vec![];
    let mut goal_points: Vec<(f32, f32)> = vec![];

    let mut rrt_solver = RRTConnect::<SimpleTree<_>, _>::new(start, goal, space, 0.2);
    let mut path = vec![];
    while rrt_solver.start_tree.size() < 10000 {
        let result = rrt_solver.step();
        if let motion_planning::rrt_connect::StepResult::Success(res) = result {
            if res.is_start {
                let start_parent_node = rrt_solver.start_tree.get(res.nearest_start);
                start_points.push((res.new.x as f32, res.new.y as f32));
                start_lines.push([
                    [start_parent_node.x as f32, start_parent_node.y as f32],
                    [res.new.x as f32, res.new.y as f32],
                ]);
                rec.log("start_tree", &Points2D::new(start_points.iter()).with_radii([0.05]))
                    .unwrap();
                rec.log("start_tree", &rerun::LineStrips2D::new(start_lines.iter()))
                    .unwrap();
            } else {
                let goal_parent_node = rrt_solver.goal_tree.get(res.nearest_goal);
                goal_points.push((res.new.x as f32, res.new.y as f32));
                goal_lines.push([
                    [goal_parent_node.x as f32, goal_parent_node.y as f32],
                    [res.new.x as f32, res.new.y as f32],
                ]);
                rec.log("goal_tree", &Points2D::new(goal_points.iter()).with_radii([0.05]))
                    .unwrap();
                rec.log("goal_tree", &rerun::LineStrips2D::new(goal_lines.iter()))
                    .unwrap();
            }
            if res.is_goal {
                path = rrt_solver.path(res);
                break;
            }
        }
    }
    let mut path_lines: Vec<[[f32; 2]; 2]> = vec![];
    for p in 0..path.len() - 1 {
        path_lines.push([
            [path[p].x as f32, path[p].y as f32],
            [path[p + 1].x as f32, path[p + 1].y as f32],
        ]);
    }
    rec.log(
        "path",
        &rerun::LineStrips2D::new(path_lines.iter()).with_colors([Color::from_rgb(255, 0, 0)]),
    )
    .unwrap();
}