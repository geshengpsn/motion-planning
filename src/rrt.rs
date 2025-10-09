use rand::rngs::ThreadRng;
use crate::{Point, Set, Space, Tree};

pub struct RRT<T, G, S> {
    pub tree: T,
    pub goal: G,
    pub space: S,
    pub d: f64,
    rng: ThreadRng,
}

impl<T, G, S, P> RRT<T, G, S> 
where 
    P: Point,
    T: Tree<P>,
    G: Set<Point = P>,
    S: Space<Point = P>,
{
    pub fn new(start: P, goal: G, space: S, d: f64) -> Self {
        Self { tree: T::new(start), goal, space, rng: rand::rng(), d }
    }

    pub fn step(&mut self) -> StepResult<P> {
        let sample = self.space.sample(&mut self.rng);
        let nearest = self.tree.nearest(&sample);
        let nearest_point = self.tree.get(nearest);
        
        let new = nearest_point.get_new(&sample, self.d);
        if self.space.is_collision_free(nearest_point, &new) {
            if self.goal.is_in(&new) {
                return StepResult::Success(SuccessStep { new, nearest, is_goal: true });
            }
            self.tree.expand(nearest, new.clone());
            StepResult::Success(SuccessStep { new, nearest, is_goal: false })
        } else {
            StepResult::Failure
        }
    }

    pub fn tree_size(&self) -> usize {
        self.tree.size()
    }

    pub fn path(&self, step_result: SuccessStep<P>) -> Vec<P> {
        let mut path = self.tree.path(step_result.nearest);
        path.push(step_result.new);
        path
    }
}

pub enum StepResult<P> {
    Success(SuccessStep<P>),
    Failure,
}

pub struct SuccessStep<P> {
    pub new: P,
    pub nearest: usize,
    pub is_goal: bool,
}


#[derive(Debug)]
pub enum RRTError {
    Failure,
}
