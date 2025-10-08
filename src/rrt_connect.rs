use rand::rngs::ThreadRng;

use crate::{Point, Space, Tree};

pub struct RRTConnect<T, S> {
    pub start_tree: Tree<T>,
    pub goal_tree: Tree<T>,
    pub space: S,
    pub d: f64,
    rng: ThreadRng,
}

impl<T, S> RRTConnect<T, S>
where
    T: Point,
    S: Space<Point = T>,
{
    pub fn new(start: T, goal: T, space: S, d: f64) -> Self {
        Self {
            start_tree: Tree::new(start),
            goal_tree: Tree::new(goal),
            space,
            rng: rand::rng(),
            d,
        }
    }

    pub fn step(&mut self) -> StepResult<T> {
        let sample = self.space.sample(&mut self.rng);
        let (tree_a, tree_b, start_tree) = if self.start_tree.size() <= self.goal_tree.size() {
            (&mut self.start_tree, &mut self.goal_tree, true)
        } else {
            (&mut self.goal_tree, &mut self.start_tree, false)
        };
        let nearest = tree_a.nearest(&sample);
        let nearest_point = tree_a.get(nearest);
        let new = nearest_point.get_new(&sample, self.d);
        if self.space.is_collision_free(nearest_point, &new) {
            let nearest_in_b = tree_b.nearest(&new);
            let nearest_point_in_b = tree_b.get(nearest_in_b);
            if nearest_point_in_b.distance(&new) < self.d
                && self.space.is_collision_free(nearest_point_in_b, &new)
            {
                return StepResult::Success(SuccessStep {
                    new,
                    nearest_start: if start_tree {nearest} else {nearest_in_b},
                    nearest_goal: if start_tree {nearest_in_b} else {nearest},
                    is_goal: true,
                });
            }
            tree_a.expand(nearest, new);
        }
        StepResult::Failure
    }
}

pub enum StepResult<T> {
    Success(SuccessStep<T>),
    Failure,
}

pub struct SuccessStep<T> {
    pub is_goal: bool,
    pub new: T,
    pub nearest_start: usize,
    pub nearest_goal: usize,
}
