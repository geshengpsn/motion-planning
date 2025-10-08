use rand::rngs::ThreadRng;

pub mod rrt;
pub mod rrt_connect;
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

pub struct Tree<T> {
    // parent index, point
    nodes: Vec<(usize, T)>,
}

impl<T> Tree<T>
where
    T: Point,
{
    fn new(start: T) -> Self {
        Self {
            nodes: vec![(0, start)],
        }
    }

    pub fn size(&self) -> usize {
        self.nodes.len()
    }

    pub fn get(&self, index: usize) -> &T {
        &self.nodes[index].1
    }

    fn expand(&mut self, from: usize, to: T) {
        self.nodes.push((from, to));
    }

    fn path(&self, end: usize) -> Vec<T> {
        let mut path = vec![];
        let mut current = end;
        while current != 0 {
            path.push(self.get(current).clone());
            current = self.nodes[current].0;
        }
        path.push(self.get(0).clone());
        path.reverse();
        path
    }
}

impl<T> Tree<T>
where
    T: Point,
{
    fn nearest(&self, point: &T) -> usize {
        let mut min_distance = f64::MAX;
        let mut min_index = 0;
        for (index, (_, p)) in self.nodes.iter().enumerate() {
            if p.distance(point) < min_distance {
                min_distance = p.distance(point);
                min_index = index;
            }
        }
        min_index
    }
}
