#![deny(clippy::all)]

use js_sys::Float32Array;
use nexus_physics_core::{BodyConfig, BodyType, PhysicsWorld, ShapeConfig};
use wasm_bindgen::prelude::*;

/// A WebAssembly-compatible wrapper around [`nexus_physics_core::PhysicsWorld`].
///
/// All method signatures use types that are representable across the Wasm-JS
/// boundary (primitives, `JsValue`, `Float32Array`, etc.).
#[wasm_bindgen]
pub struct WasmPhysicsWorld {
    inner: PhysicsWorld,
    /// Backing buffer for zero-copy snapshot views.  Owned by this struct so
    /// it lives in Wasm linear memory and its address stays stable for the
    /// lifetime of a single call.
    snapshot_buf: Vec<f32>,
    /// Backing buffer for zero-copy raycast-batch views.
    raycast_buf: Vec<f32>,
}

#[wasm_bindgen]
impl WasmPhysicsWorld {
    /// Create a new physics world with the given gravity components.
    #[wasm_bindgen(constructor)]
    pub fn new(gx: f32, gy: f32, gz: f32) -> WasmPhysicsWorld {
        WasmPhysicsWorld {
            inner: PhysicsWorld::new([gx, gy, gz]),
            snapshot_buf: Vec::new(),
            raycast_buf: Vec::new(),
        }
    }

    /// Advance the simulation.
    ///
    /// `delta_time` is the elapsed wall-clock time in seconds since the last
    /// call.  Internally an accumulator ensures only fixed-timestep ticks run.
    ///
    /// Returns a `JsError` if `delta_time` is not a finite positive value.
    pub fn step(&mut self, delta_time: f32) -> Result<(), JsError> {
        if !delta_time.is_finite() || delta_time < 0.0 {
            return Err(JsError::new(
                "delta_time must be a finite non-negative number",
            ));
        }
        self.inner.step(delta_time);
        Ok(())
    }

    /// Add a body to the world.
    ///
    /// `body_type_str` must be one of `"dynamic"`, `"static"`, or `"kinematic"`.
    /// `shape_type` must be one of `"cuboid"`, `"ball"`, or `"cylinder"`.
    /// `dims` for a cuboid is `[hx, hy, hz]`; for a ball it is `[radius]`;
    /// for a cylinder it is `[radius, half_height]`.
    ///
    /// Returns `Ok(())` on success, throws a `JsError` on failure.
    #[allow(clippy::too_many_arguments)]
    pub fn add_body(
        &mut self,
        entity_id: &str,
        body_type_str: &str,
        shape_type: &str,
        dims: &[f32],
        px: f32,
        py: f32,
        pz: f32,
    ) -> Result<(), JsError> {
        if entity_id.is_empty() {
            return Err(JsError::new("entity_id must not be empty"));
        }
        for (name, v) in [("px", px), ("py", py), ("pz", pz)] {
            if !v.is_finite() {
                return Err(JsError::new(&format!(
                    "Position component '{name}' must be finite, got {v}"
                )));
            }
        }

        let body_type = match body_type_str {
            "dynamic" => BodyType::Dynamic,
            "static" => BodyType::Static,
            "kinematic" => BodyType::KinematicPositionBased,
            other => return Err(JsError::new(&format!("Unknown body type: '{other}'"))),
        };

        let shape = match shape_type {
            "cuboid" => {
                if dims.len() < 3 {
                    return Err(JsError::new("Cuboid requires 3 dimensions: [hx, hy, hz]"));
                }
                ShapeConfig::Cuboid {
                    hx: dims[0],
                    hy: dims[1],
                    hz: dims[2],
                }
            }
            "ball" => {
                if dims.is_empty() {
                    return Err(JsError::new("Ball requires 1 dimension: [radius]"));
                }
                ShapeConfig::Ball { radius: dims[0] }
            }
            "cylinder" => {
                if dims.len() < 2 {
                    return Err(JsError::new(
                        "Cylinder requires 2 dimensions: [radius, half_height]",
                    ));
                }
                ShapeConfig::Cylinder {
                    radius: dims[0],
                    half_height: dims[1],
                }
            }
            other => return Err(JsError::new(&format!("Unknown shape type: '{other}'"))),
        };

        self.inner
            .add_body(BodyConfig {
                entity_id: entity_id.to_string(),
                body_type,
                shape,
                position: [px, py, pz],
            })
            .map_err(|e| JsError::new(&e))
    }

    /// Remove a body (and its collider) from the world by `entity_id`.
    ///
    /// Returns a `JsError` if the entity is not found.
    pub fn remove_body(&mut self, entity_id: &str) -> Result<(), JsError> {
        self.inner
            .remove_body(entity_id)
            .map_err(|e| JsError::new(&e))
    }

    /// Set the linear and angular velocity of a body.
    #[allow(clippy::too_many_arguments)]
    pub fn set_velocity(
        &mut self,
        entity_id: &str,
        lx: f32,
        ly: f32,
        lz: f32,
        ax: f32,
        ay: f32,
        az: f32,
    ) -> Result<(), JsError> {
        for (name, v) in [
            ("lx", lx),
            ("ly", ly),
            ("lz", lz),
            ("ax", ax),
            ("ay", ay),
            ("az", az),
        ] {
            if !v.is_finite() {
                return Err(JsError::new(&format!(
                    "Velocity component '{name}' must be finite, got {v}"
                )));
            }
        }
        self.inner
            .set_velocity(entity_id, [lx, ly, lz], [ax, ay, az])
            .map_err(|e| JsError::new(&e))
    }

    /// Apply an instantaneous impulse to a body.
    pub fn apply_impulse(
        &mut self,
        entity_id: &str,
        ix: f32,
        iy: f32,
        iz: f32,
    ) -> Result<(), JsError> {
        for (name, v) in [("ix", ix), ("iy", iy), ("iz", iz)] {
            if !v.is_finite() {
                return Err(JsError::new(&format!(
                    "Impulse component '{name}' must be finite, got {v}"
                )));
            }
        }
        self.inner
            .apply_impulse(entity_id, [ix, iy, iz])
            .map_err(|e| JsError::new(&e))
    }

    /// Get the current position of a body as `[x, y, z]` packed into a
    /// `Float32Array` of length 3.
    pub fn get_position(&self, entity_id: &str) -> Result<Float32Array, JsError> {
        let pos = self
            .inner
            .get_position(entity_id)
            .map_err(|e| JsError::new(&e))?;
        let arr = Float32Array::new_with_length(3);
        arr.set_index(0, pos[0]);
        arr.set_index(1, pos[1]);
        arr.set_index(2, pos[2]);
        Ok(arr)
    }

    /// Get the current rotation of a body as `[qx, qy, qz, qw]` packed into a
    /// `Float32Array` of length 4.
    pub fn get_rotation(&self, entity_id: &str) -> Result<Float32Array, JsError> {
        let rot = self
            .inner
            .get_rotation(entity_id)
            .map_err(|e| JsError::new(&e))?;
        let arr = Float32Array::new_with_length(4);
        arr.set_index(0, rot[0]);
        arr.set_index(1, rot[1]);
        arr.set_index(2, rot[2]);
        arr.set_index(3, rot[3]);
        Ok(arr)
    }

    /// Return a zero-copy `Float32Array` view into the snapshot buffer in Wasm
    /// linear memory.
    ///
    /// Layout per body: `[index, x, y, z, qx, qy, qz, qw]` — 8 floats.
    /// Total length is `num_dynamic_bodies * 8`.
    ///
    /// # Safety
    /// The returned view points directly into WebAssembly linear memory.
    /// Do **not** trigger any Wasm heap allocation (e.g., call other Wasm
    /// methods that allocate) while holding the returned `Float32Array` in
    /// JavaScript, as a reallocation of the backing `Vec` would invalidate the
    /// pointer.  Acquire all the values you need from the view before calling
    /// any other method on this object.
    pub fn get_snapshot_view(&mut self) -> Float32Array {
        let snapshot = self.inner.get_snapshot();
        self.snapshot_buf = snapshot.to_buffer();
        // SAFETY: `self.snapshot_buf` is owned by this struct and lives in
        // Wasm linear memory.  No reallocation of this Vec occurs between this
        // point and when JavaScript reads the returned view, provided the
        // caller does not invoke any other allocating Wasm method while the
        // view is live.
        unsafe { Float32Array::view(&self.snapshot_buf) }
    }

    /// Return the number of bodies in the world.
    pub fn body_count(&self) -> usize {
        self.inner.body_count()
    }

    /// Rebuild the internal query pipeline from the current world state.
    ///
    /// This is called automatically at the end of every `step()`. Call this
    /// manually after adding bodies to a static scene that is never stepped.
    pub fn update_query_pipeline(&mut self) {
        self.inner.update_query_pipeline();
    }

    /// Cast a single ray and return the time-of-impact (distance) to the first
    /// hit, or `NaN` if no collider was struck within `max_toi`.
    ///
    /// * `ox, oy, oz` — ray origin.
    /// * `dx, dy, dz` — ray direction (need not be normalised).
    /// * `max_toi` — maximum distance / time-of-impact.
    #[allow(clippy::too_many_arguments)]
    pub fn cast_ray(
        &self,
        ox: f32,
        oy: f32,
        oz: f32,
        dx: f32,
        dy: f32,
        dz: f32,
        max_toi: f32,
    ) -> f32 {
        self.inner
            .cast_ray([ox, oy, oz], [dx, dy, dz], max_toi)
            .unwrap_or(f32::NAN)
    }

    /// Cast multiple rays in a single call and return the results as a
    /// zero-copy `Float32Array` view into Wasm linear memory.
    ///
    /// * `origins` — flat `Float32Array` with layout `[x0, y0, z0, x1, y1, z1, …]`.
    /// * `directions` — flat `Float32Array` with layout `[dx0, dy0, dz0, …]`.
    /// * `max_toi` — maximum distance / time-of-impact for every ray.
    ///
    /// Returns a `Float32Array` of length `n` (one TOI per ray). A miss is
    /// represented as `max_toi`.
    ///
    /// # Safety
    /// The returned view points directly into WebAssembly linear memory.
    /// Do **not** trigger any Wasm heap allocation while holding the returned
    /// `Float32Array` in JavaScript.  Acquire all values before calling any
    /// other allocating method on this object.
    pub fn cast_ray_batch(
        &mut self,
        origins: &[f32],
        directions: &[f32],
        max_toi: f32,
    ) -> Result<Float32Array, JsError> {
        if origins.len() != directions.len() {
            return Err(JsError::new(
                "cast_ray_batch: origins and directions must have the same length",
            ));
        }
        if !origins.len().is_multiple_of(3) {
            return Err(JsError::new(
                "cast_ray_batch: origins length must be a multiple of 3",
            ));
        }

        let n = origins.len() / 3;
        let orig_chunks: Vec<[f32; 3]> = (0..n)
            .map(|i| [origins[i * 3], origins[i * 3 + 1], origins[i * 3 + 2]])
            .collect();
        let dir_chunks: Vec<[f32; 3]> = (0..n)
            .map(|i| {
                [
                    directions[i * 3],
                    directions[i * 3 + 1],
                    directions[i * 3 + 2],
                ]
            })
            .collect();

        self.raycast_buf = self
            .inner
            .cast_ray_batch(&orig_chunks, &dir_chunks, max_toi)
            .map_err(|e| JsError::new(&e))?;
        // SAFETY: `self.raycast_buf` is owned by this struct and lives in Wasm
        // linear memory.  No reallocation of this Vec occurs between this
        // point and when JavaScript reads the returned view, provided the
        // caller does not invoke any other allocating Wasm method while the
        // view is live.
        Ok(unsafe { Float32Array::view(&self.raycast_buf) })
    }
}
