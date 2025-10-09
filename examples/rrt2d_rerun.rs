use motion_planning::{
    rrt::RRT,
    utils::{CubeSpace, Goal, Rect, SimpleTree}, Tree,
};
use nalgebra::Vector2;
use rerun::{Boxes2D, Color, Points2D};

fn main() {
    let rec = rerun::RecordingStreamBuilder::new("rrt").spawn().unwrap();

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

    let goal = Goal {
        position: Vector2::new(9.0, 9.0),
        radius: 0.2,
    };
    rec.log(
        "goal",
        &Points2D::new([(goal.position.x as f32, goal.position.y as f32)])
            .with_radii([goal.radius as f32]),
    )
    .unwrap();

    let mut lines: Vec<[[f32; 2]; 2]> = vec![];
    let mut points: Vec<(f32, f32)> = vec![];

    let mut rrt_solver = RRT::<SimpleTree<_>, _, _>::new(start, goal, space, 0.2);
    let mut path = vec![];
    while rrt_solver.tree_size() < 10000 {
        let result = rrt_solver.step();
        if let motion_planning::rrt::StepResult::Success(res) = result {
            let parent_node = rrt_solver.tree.get(res.nearest);
            points.push((res.new.x as f32, res.new.y as f32));
            lines.push([
                [parent_node.x as f32, parent_node.y as f32],
                [res.new.x as f32, res.new.y as f32],
            ]);
            rec.log("tree", &Points2D::new(points.iter()).with_radii([0.05]))
                .unwrap();
            rec.log("tree", &rerun::LineStrips2D::new(lines.iter()))
                .unwrap();
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
    // for p in path {
    //     println!("{p}");
    // }
    // let mut path = vec![];
    let start_time = std::time::Instant::now();
    while rrt_solver.tree_size() < 10000 {
        let result = rrt_solver.step();
        if let motion_planning::rrt::StepResult::Success(res) = result
            && res.is_goal
        {
            // path = rrt_solver.path(res);
            break;
        }
    }
    println!("Time taken: {:?}", start_time.elapsed());
}
