use rand::rngs::ThreadRng;

use crate::{Point, Space, Tree};

pub struct RRTConnect<T, S> {
    pub start_tree: T,
    pub goal_tree: T,
    pub space: S,
    pub d: f64,
    rng: ThreadRng,
}

impl<T, P, S> RRTConnect<T, S>
where
    P: Point,
    T: Tree<P>,
    S: Space<Point = P>,
{
    pub fn new(start: P, goal: P, space: S, d: f64) -> Self {
        Self {
            start_tree: T::new(start),
            goal_tree: T::new(goal),
            space,
            rng: rand::rng(),
            d,
        }
    }

    pub fn step(&mut self) -> StepResult<P> {
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
                    is_start: start_tree,
                    is_goal: true,
                });
            }
            tree_a.expand(nearest, new.clone());
            StepResult::Success(SuccessStep {
                new,
                nearest_start: if start_tree {nearest} else {nearest_in_b},
                nearest_goal: if start_tree {nearest_in_b} else {nearest},
                is_start: start_tree,
                is_goal: false,
            })
        } else {
            StepResult::Failure
        }
    }

    pub fn path(&self, step_result: SuccessStep<P>) -> Vec<P> {
        let mut start_path = self.start_tree.path(step_result.nearest_start);
        let mut goal_path = self.goal_tree.path(step_result.nearest_goal);
        goal_path.reverse();
        start_path.push(step_result.new);
        start_path.extend(goal_path);
        start_path
    }
}

pub enum StepResult<P> {
    Success(SuccessStep<P>),
    Failure,
}

pub struct SuccessStep<P> {
    pub is_goal: bool,
    pub new: P,
    pub is_start: bool,
    pub nearest_start: usize,
    pub nearest_goal: usize,
}
