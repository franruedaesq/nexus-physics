#![deny(clippy::all)]

pub mod world;

pub use world::{BodyConfig, BodyType, EntityId, PhysicsWorld, ShapeConfig, Snapshot, SnapshotEntry};
