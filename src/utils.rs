use nalgebra::Vector2;
use rand::{rngs::ThreadRng, Rng};

use crate::{Point, Set, Space};

impl Point for Vector2<f64> {
    fn distance(&self, other: &Self) -> f64 {
        (*self - *other).norm()
    }
    
    fn get_new(&self, to: &Self, d: f64) -> Self {
        self + (to - self).normalize() * d
    }
}


pub struct Goal {
    pub position: Vector2<f64>,
    pub radius: f64,
}

impl Set for Goal {
    type Point = Vector2<f64>;
    fn is_in(&self, point: &Self::Point) -> bool {
        point.distance(&self.position) <= self.radius
    }
}

pub struct CubeSpace {
    pub width: f64,
    pub height: f64,
    pub obstacles: Vec<Rect<f64>>,
}

pub struct Rect<T> {
    pub min: Vector2<T>,
    pub max: Vector2<T>,
}

impl Rect<f64> {
    pub fn new(min: Vector2<f64>, max: Vector2<f64>) -> Self {
        Self { min, max }
    }
}

impl Set for Rect<f64> {
    type Point = Vector2<f64>;
    fn is_in(&self, point: &Self::Point) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }
}

impl Rect<f64> {
    fn intersect_line(&self, from: &Vector2<f64>, to: &Vector2<f64>) -> bool {
        // Check if either endpoint is inside the rectangle
        if self.is_in(from) || self.is_in(to) {
            return true;
        }

        // Check intersection with each of the four edges
        let corners = [
            self.min,
            Vector2::new(self.max.x, self.min.y),
            self.max,
            Vector2::new(self.min.x, self.max.y),
        ];

        // Check all four edges of the rectangle
        for i in 0..4 {
            let edge_start = &corners[i];
            let edge_end = &corners[(i + 1) % 4];
            if Self::segments_intersect(from, to, edge_start, edge_end) {
                return true;
            }
        }

        false
    }

    // Helper function to check if two line segments intersect
    fn segments_intersect(
        p1: &Vector2<f64>,
        p2: &Vector2<f64>,
        p3: &Vector2<f64>,
        p4: &Vector2<f64>,
    ) -> bool {
        let d1 = Self::cross_product_2d(&(p2 - p1), &(p3 - p1));
        let d2 = Self::cross_product_2d(&(p2 - p1), &(p4 - p1));
        let d3 = Self::cross_product_2d(&(p4 - p3), &(p1 - p3));
        let d4 = Self::cross_product_2d(&(p4 - p3), &(p2 - p3));

        if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
            && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
        {
            return true;
        }

        // Check for collinear cases
        if d1.abs() < 1e-10 && Self::on_segment(p1, p3, p2) {
            return true;
        }
        if d2.abs() < 1e-10 && Self::on_segment(p1, p4, p2) {
            return true;
        }
        if d3.abs() < 1e-10 && Self::on_segment(p3, p1, p4) {
            return true;
        }
        if d4.abs() < 1e-10 && Self::on_segment(p3, p2, p4) {
            return true;
        }

        false
    }

    // 2D cross product (returns scalar)
    fn cross_product_2d(v1: &Vector2<f64>, v2: &Vector2<f64>) -> f64 {
        v1.x * v2.y - v1.y * v2.x
    }

    // Check if point q lies on segment pr
    fn on_segment(p: &Vector2<f64>, q: &Vector2<f64>, r: &Vector2<f64>) -> bool {
        q.x <= p.x.max(r.x) && q.x >= p.x.min(r.x) && q.y <= p.y.max(r.y) && q.y >= p.y.min(r.y)
    }
}

impl Space for CubeSpace {
    type Point = Vector2<f64>;
    fn sample(&self, rng: &mut ThreadRng) -> Self::Point {
        Vector2::new(
            self.width * rng.random_range(0.0..1.0),
            self.height * rng.random_range(0.0..1.0),
        )
    }
    fn is_collision_free(&self, from: &Self::Point, to: &Self::Point) -> bool {
        for obstacle in self.obstacles.iter() {
            if obstacle.intersect_line(from, to) {
                return false;
            }
        }
        true
    }
}