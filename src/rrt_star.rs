use rand::rngs::ThreadRng;

use crate::{Tree};

pub struct RRTStar<T, G, S> {
    tree: T,
    goal: G,
    space: S,
    d: f64,
    rng: ThreadRng,
}