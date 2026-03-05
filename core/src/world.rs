use rapier3d::prelude::*;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// A unique identifier for a body in the physics world.
/// The consumer can pass in any string ID they like (e.g., "player_1").
pub type EntityId = String;

/// Fixed simulation timestep: 60 Hz.
const FIXED_DT: f32 = 1.0 / 60.0;

/// The type of a rigid body — either dynamic (affected by forces), static (immovable),
/// or kinematic position-based (moved programmatically via position targets).
#[derive(Debug, Clone, PartialEq)]
pub enum BodyType {
    Dynamic,
    Static,
    KinematicPositionBased,
}

/// Describes the shape of a collider to be added to the world.
#[derive(Debug, Clone)]
pub enum ShapeConfig {
    /// An axis-aligned box defined by its half-extents `[hx, hy, hz]`.
    Cuboid { hx: f32, hy: f32, hz: f32 },
    /// A sphere defined by its radius.
    Ball { radius: f32 },
    /// A cylinder defined by its radius and half-height along the Y axis.
    Cylinder { radius: f32, half_height: f32 },
}

/// Configuration used to add a body to the physics world.
#[derive(Debug, Clone)]
pub struct BodyConfig {
    pub entity_id: EntityId,
    pub body_type: BodyType,
    pub shape: ShapeConfig,
    /// Initial position `[x, y, z]`.
    pub position: [f32; 3],
}

/// A single entry in a world snapshot representing one body's state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SnapshotEntry {
    pub entity_id: EntityId,
    /// Position `[x, y, z]`.
    pub position: [f32; 3],
    /// Rotation as a quaternion `[qx, qy, qz, qw]`.
    pub rotation: [f32; 4],
}

/// A complete snapshot of all dynamic bodies in the world at a given instant.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Snapshot {
    pub entries: Vec<SnapshotEntry>,
}

impl Snapshot {
    /// Serialize the snapshot into a flat `Float32` buffer:
    /// `[id_len (as u32 bits), id_bytes..., x, y, z, qx, qy, qz, qw, ...]`
    ///
    /// For zero-copy Wasm interop the positions and rotations are packed as
    /// a contiguous `Vec<f32>` where every body occupies exactly 8 floats:
    /// `[x, y, z, qx, qy, qz, qw, <index_as_f32>]`.
    ///
    /// The returned buffer is `num_bodies * 8` f32 values long.  The caller
    /// must track which index corresponds to which `entity_id` via
    /// `Snapshot::entries` (ordered consistently).
    pub fn to_buffer(&self) -> Vec<f32> {
        let mut buf = Vec::with_capacity(self.entries.len() * 8);
        for (idx, entry) in self.entries.iter().enumerate() {
            buf.push(entry.position[0]);
            buf.push(entry.position[1]);
            buf.push(entry.position[2]);
            buf.push(entry.rotation[0]);
            buf.push(entry.rotation[1]);
            buf.push(entry.rotation[2]);
            buf.push(entry.rotation[3]);
            buf.push(idx as f32);
        }
        buf
    }

    /// Serialize to JSON using `serde_json`.
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }
}

/// The central physics world.  It wraps rapier3d and exposes a high-level,
/// ID-based API that is safe to expose over a network boundary.
pub struct PhysicsWorld {
    gravity: Vector<f32>,
    integration_params: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    /// Maps user-supplied string IDs to internal Rapier handles.
    entity_map: HashMap<EntityId, RigidBodyHandle>,
    /// Leftover time from the previous `step()` call for the accumulator.
    time_accumulator: f32,
}

impl PhysicsWorld {
    /// Create a new physics world with the given gravity vector `[gx, gy, gz]`.
    pub fn new(gravity: [f32; 3]) -> Self {
        let mut integration_params = IntegrationParameters::default();
        integration_params.dt = FIXED_DT;

        Self {
            gravity: vector![gravity[0], gravity[1], gravity[2]],
            integration_params,
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            entity_map: HashMap::new(),
            time_accumulator: 0.0,
        }
    }

    /// Return the configured gravity vector.
    pub fn gravity(&self) -> [f32; 3] {
        [self.gravity.x, self.gravity.y, self.gravity.z]
    }

    /// Advance the simulation using a fixed-timestep accumulator.
    ///
    /// `delta_time` is the real elapsed time in seconds since the last call.
    /// The accumulator will run as many fixed-`FIXED_DT` ticks as needed to
    /// consume `delta_time`, ensuring determinism regardless of frame rate.
    pub fn step(&mut self, delta_time: f32) {
        self.time_accumulator += delta_time;

        while self.time_accumulator >= FIXED_DT {
            self.physics_pipeline.step(
                &self.gravity,
                &self.integration_params,
                &mut self.island_manager,
                &mut self.broad_phase,
                &mut self.narrow_phase,
                &mut self.rigid_body_set,
                &mut self.collider_set,
                &mut self.impulse_joint_set,
                &mut self.multibody_joint_set,
                &mut self.ccd_solver,
                None,
                &(),
                &(),
            );
            self.time_accumulator -= FIXED_DT;
        }
    }

    /// Add a body to the world using the provided configuration.
    ///
    /// Returns an error if an entity with the same `entity_id` already exists.
    pub fn add_body(&mut self, config: BodyConfig) -> Result<(), String> {
        if self.entity_map.contains_key(&config.entity_id) {
            return Err(format!(
                "Entity '{}' already exists in the physics world",
                config.entity_id
            ));
        }

        let rigid_body = match config.body_type {
            BodyType::Dynamic => RigidBodyBuilder::dynamic()
                .translation(vector![
                    config.position[0],
                    config.position[1],
                    config.position[2]
                ])
                .build(),
            BodyType::Static => RigidBodyBuilder::fixed()
                .translation(vector![
                    config.position[0],
                    config.position[1],
                    config.position[2]
                ])
                .build(),
            BodyType::KinematicPositionBased => RigidBodyBuilder::kinematic_position_based()
                .translation(vector![
                    config.position[0],
                    config.position[1],
                    config.position[2]
                ])
                .build(),
        };

        let handle = self.rigid_body_set.insert(rigid_body);

        let collider = match config.shape {
            ShapeConfig::Cuboid { hx, hy, hz } => ColliderBuilder::cuboid(hx, hy, hz).build(),
            ShapeConfig::Ball { radius } => ColliderBuilder::ball(radius).build(),
            ShapeConfig::Cylinder { radius, half_height } => {
                ColliderBuilder::cylinder(half_height, radius).build()
            }
        };

        self.collider_set
            .insert_with_parent(collider, handle, &mut self.rigid_body_set);

        self.entity_map.insert(config.entity_id, handle);
        Ok(())
    }

    /// Return the total number of rigid bodies registered in the world.
    pub fn body_count(&self) -> usize {
        self.rigid_body_set.len()
    }

    /// Return the number of entries in the internal entity handle map.
    pub fn entity_count(&self) -> usize {
        self.entity_map.len()
    }

    /// Set the linear and angular velocity of a body identified by `entity_id`.
    pub fn set_velocity(
        &mut self,
        entity_id: &str,
        linear: [f32; 3],
        angular: [f32; 3],
    ) -> Result<(), String> {
        let handle = self
            .entity_map
            .get(entity_id)
            .copied()
            .ok_or_else(|| format!("Entity '{}' not found", entity_id))?;

        let body = self
            .rigid_body_set
            .get_mut(handle)
            .ok_or_else(|| format!("Rapier handle for '{}' is invalid", entity_id))?;

        body.set_linvel(vector![linear[0], linear[1], linear[2]], true);
        body.set_angvel(vector![angular[0], angular[1], angular[2]], true);
        Ok(())
    }

    /// Apply an instantaneous impulse to a body identified by `entity_id`.
    pub fn apply_impulse(&mut self, entity_id: &str, impulse: [f32; 3]) -> Result<(), String> {
        let handle = self
            .entity_map
            .get(entity_id)
            .copied()
            .ok_or_else(|| format!("Entity '{}' not found", entity_id))?;

        let body = self
            .rigid_body_set
            .get_mut(handle)
            .ok_or_else(|| format!("Rapier handle for '{}' is invalid", entity_id))?;

        body.apply_impulse(vector![impulse[0], impulse[1], impulse[2]], true);
        Ok(())
    }

    /// Get the current translation of a body as `[x, y, z]`.
    pub fn get_position(&self, entity_id: &str) -> Result<[f32; 3], String> {
        let handle = self
            .entity_map
            .get(entity_id)
            .copied()
            .ok_or_else(|| format!("Entity '{}' not found", entity_id))?;

        let body = self
            .rigid_body_set
            .get(handle)
            .ok_or_else(|| format!("Rapier handle for '{}' is invalid", entity_id))?;

        let t = body.translation();
        Ok([t.x, t.y, t.z])
    }

    /// Build a snapshot of all registered bodies ordered by entity_id.
    pub fn get_snapshot(&self) -> Snapshot {
        let mut entries: Vec<SnapshotEntry> = self
            .entity_map
            .iter()
            .filter_map(|(id, handle)| {
                self.rigid_body_set.get(*handle).map(|body| {
                    let t = body.translation();
                    let r = body.rotation();
                    SnapshotEntry {
                        entity_id: id.clone(),
                        position: [t.x, t.y, t.z],
                        rotation: [r.i, r.j, r.k, r.w],
                    }
                })
            })
            .collect();

        // Sort by entity_id for a deterministic ordering.
        entries.sort_by(|a, b| a.entity_id.cmp(&b.entity_id));

        Snapshot { entries }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Step 2: PhysicsWorld Initialization ----

    #[test]
    fn test_physics_world_stores_gravity() {
        let world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        let g = world.gravity();
        assert!((g[0] - 0.0).abs() < 1e-6);
        assert!((g[1] - (-9.81)).abs() < 1e-4);
        assert!((g[2] - 0.0).abs() < 1e-6);
    }

    #[test]
    fn test_physics_world_custom_gravity() {
        let world = PhysicsWorld::new([0.0, -1.62, 0.0]); // Moon gravity
        let g = world.gravity();
        assert!((g[1] - (-1.62)).abs() < 1e-4);
    }

    #[test]
    fn test_step_does_not_panic() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        // Stepping with exactly one fixed-timestep delta should not panic.
        world.step(FIXED_DT);
    }

    #[test]
    fn test_step_accumulator_runs_multiple_ticks() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        // A delta of 3 × FIXED_DT should run 3 ticks without panicking.
        world.step(FIXED_DT * 3.0);
    }

    #[test]
    fn test_step_accumulator_partial_tick() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        // A delta smaller than FIXED_DT should not advance the simulation yet.
        world.step(FIXED_DT * 0.5);
        // No panic is the only assertion here.
    }

    // ---- Step 3: Entity Registration ----

    #[test]
    fn test_add_static_floor_and_dynamic_sphere() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);

        world
            .add_body(BodyConfig {
                entity_id: "floor".to_string(),
                body_type: BodyType::Static,
                shape: ShapeConfig::Cuboid {
                    hx: 50.0,
                    hy: 0.1,
                    hz: 50.0,
                },
                position: [0.0, -0.1, 0.0],
            })
            .expect("Adding floor should succeed");

        world
            .add_body(BodyConfig {
                entity_id: "ball".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 0.5 },
                position: [0.0, 5.0, 0.0],
            })
            .expect("Adding ball should succeed");

        assert_eq!(world.body_count(), 2);
    }

    #[test]
    fn test_add_body_duplicate_id_returns_error() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);

        world
            .add_body(BodyConfig {
                entity_id: "player".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 0.5 },
                position: [0.0, 0.0, 0.0],
            })
            .unwrap();

        let result = world.add_body(BodyConfig {
            entity_id: "player".to_string(),
            body_type: BodyType::Dynamic,
            shape: ShapeConfig::Ball { radius: 0.5 },
            position: [1.0, 0.0, 0.0],
        });

        assert!(result.is_err());
    }

    #[test]
    fn test_entity_count_matches_handle_map() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);

        world
            .add_body(BodyConfig {
                entity_id: "floor".to_string(),
                body_type: BodyType::Static,
                shape: ShapeConfig::Cuboid {
                    hx: 50.0,
                    hy: 0.1,
                    hz: 50.0,
                },
                position: [0.0, -0.1, 0.0],
            })
            .expect("Adding floor should succeed");

        world
            .add_body(BodyConfig {
                entity_id: "ball".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 0.5 },
                position: [0.0, 5.0, 0.0],
            })
            .expect("Adding ball should succeed");

        assert_eq!(world.entity_count(), 2, "handle map should contain exactly 2 entries");
        assert_eq!(world.body_count(), 2);
    }

    #[test]
    fn test_get_position_invalid_entity_returns_error() {
        let world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        let result = world.get_position("nonexistent");
        assert!(result.is_err(), "get_position with invalid ID should return Err, not panic");
    }

    #[test]
    fn test_add_kinematic_position_based_body() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);

        world
            .add_body(BodyConfig {
                entity_id: "platform".to_string(),
                body_type: BodyType::KinematicPositionBased,
                shape: ShapeConfig::Cuboid {
                    hx: 2.0,
                    hy: 0.2,
                    hz: 2.0,
                },
                position: [0.0, 2.0, 0.0],
            })
            .expect("Adding kinematic body should succeed");

        assert_eq!(world.entity_count(), 1);
        let pos = world.get_position("platform").unwrap();
        assert!((pos[1] - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_add_cylinder_shape() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);

        world
            .add_body(BodyConfig {
                entity_id: "pillar".to_string(),
                body_type: BodyType::Static,
                shape: ShapeConfig::Cylinder {
                    radius: 0.5,
                    half_height: 2.0,
                },
                position: [0.0, 0.0, 0.0],
            })
            .expect("Adding cylinder-shaped body should succeed");

        assert_eq!(world.entity_count(), 1);
    }

    // ---- Step 4: Input Injection & Forces ----

    #[test]
    fn test_set_velocity_increases_y_position() {
        let mut world = PhysicsWorld::new([0.0, 0.0, 0.0]); // no gravity
        world
            .add_body(BodyConfig {
                entity_id: "cube".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Cuboid {
                    hx: 0.5,
                    hy: 0.5,
                    hz: 0.5,
                },
                position: [0.0, 0.0, 0.0],
            })
            .unwrap();

        world
            .set_velocity("cube", [0.0, 10.0, 0.0], [0.0, 0.0, 0.0])
            .expect("set_velocity should succeed");

        world.step(FIXED_DT);

        let pos = world.get_position("cube").unwrap();
        assert!(pos[1] > 0.0, "Y position should increase after upward velocity");
    }

    #[test]
    fn test_set_velocity_invalid_entity_returns_error() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        let result = world.set_velocity("nonexistent", [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]);
        assert!(result.is_err());
    }

    #[test]
    fn test_apply_impulse_moves_body() {
        let mut world = PhysicsWorld::new([0.0, 0.0, 0.0]); // no gravity
        world
            .add_body(BodyConfig {
                entity_id: "box".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Cuboid {
                    hx: 0.5,
                    hy: 0.5,
                    hz: 0.5,
                },
                position: [0.0, 0.0, 0.0],
            })
            .unwrap();

        world
            .apply_impulse("box", [100.0, 0.0, 0.0])
            .expect("apply_impulse should succeed");

        world.step(FIXED_DT);

        let pos = world.get_position("box").unwrap();
        assert!(pos[0] > 0.0, "X position should increase after impulse in +X");
    }

    #[test]
    fn test_apply_impulse_invalid_entity_returns_error() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        let result = world.apply_impulse("ghost", [0.0, 100.0, 0.0]);
        assert!(result.is_err());
    }

    // ---- Step 5: State Snapshot Generation ----

    #[test]
    fn test_snapshot_contains_correct_body_count() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        world
            .add_body(BodyConfig {
                entity_id: "a".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 1.0 },
                position: [0.0, 10.0, 0.0],
            })
            .unwrap();
        world
            .add_body(BodyConfig {
                entity_id: "b".to_string(),
                body_type: BodyType::Static,
                shape: ShapeConfig::Cuboid {
                    hx: 5.0,
                    hy: 0.1,
                    hz: 5.0,
                },
                position: [0.0, 0.0, 0.0],
            })
            .unwrap();

        let snapshot = world.get_snapshot();
        assert_eq!(snapshot.entries.len(), 2);
    }

    #[test]
    fn test_snapshot_position_updates_after_step() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        world
            .add_body(BodyConfig {
                entity_id: "falling".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 0.5 },
                position: [0.0, 10.0, 0.0],
            })
            .unwrap();

        let before = world.get_snapshot();
        // Step forward by one second (60 ticks).
        world.step(1.0);
        let after = world.get_snapshot();

        let y_before = before.entries[0].position[1];
        let y_after = after.entries[0].position[1];
        assert!(
            y_after < y_before,
            "Body should fall under gravity: y_before={y_before}, y_after={y_after}"
        );
    }

    #[test]
    fn test_snapshot_quaternion_is_unit() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        world
            .add_body(BodyConfig {
                entity_id: "body".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 0.5 },
                position: [0.0, 0.0, 0.0],
            })
            .unwrap();
        world.step(FIXED_DT);

        let snapshot = world.get_snapshot();
        assert!(!snapshot.entries.is_empty(), "Expected at least one entry");
        for entry in &snapshot.entries {
            let r = entry.rotation;
            let norm = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]).sqrt();
            assert!(
                (norm - 1.0).abs() < 1e-4,
                "Quaternion should be normalized: norm={norm}"
            );
        }
    }

    #[test]
    fn test_snapshot_to_buffer_size() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        world
            .add_body(BodyConfig {
                entity_id: "p1".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 0.5 },
                position: [0.0, 0.0, 0.0],
            })
            .unwrap();
        world
            .add_body(BodyConfig {
                entity_id: "p2".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 0.5 },
                position: [1.0, 0.0, 0.0],
            })
            .unwrap();

        let snapshot = world.get_snapshot();
        let buf = snapshot.to_buffer();
        // 2 bodies × 8 floats each = 16
        assert_eq!(buf.len(), 16);
    }

    #[test]
    fn test_snapshot_to_json_is_valid() {
        let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);
        world
            .add_body(BodyConfig {
                entity_id: "obj".to_string(),
                body_type: BodyType::Dynamic,
                shape: ShapeConfig::Ball { radius: 1.0 },
                position: [3.0, 4.0, 5.0],
            })
            .unwrap();

        let snapshot = world.get_snapshot();
        let json = snapshot.to_json();
        assert!(json.contains("obj"));
        assert!(json.contains("position"));
        assert!(json.contains("rotation"));
    }
}
