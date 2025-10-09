use rand::rngs::ThreadRng;

pub mod rrt;
pub mod rrt_connect;
pub mod rrt_star;
pub mod utils;

pub trait Set {
    type Point;
    fn is_in(&self, point: &Self::Point) -> bool;
}

pub trait Space {
    type Point;
    fn sample(&self, rng: &mut ThreadRng) -> Self::Point;
    fn is_collision_free(&self, from: &Self::Point, to: &Self::Point) -> bool;
}

pub trait Point: Clone {
    fn distance(&self, other: &Self) -> f64;
    fn get_new(&self, to: &Self, d: f64) -> Self;
}

pub trait Tree<T> {
    fn new(start: T) -> Self;
    fn size(&self) -> usize;
    fn get(&self, index: usize) -> &T;
    fn expand(&mut self, from: usize, to: T);
    fn path(&self, end: usize) -> Vec<T>;
    fn nearest(&self, point: &T) -> usize;
}
