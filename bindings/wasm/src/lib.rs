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
}

#[wasm_bindgen]
impl WasmPhysicsWorld {
    /// Create a new physics world with the given gravity components.
    #[wasm_bindgen(constructor)]
    pub fn new(gx: f32, gy: f32, gz: f32) -> WasmPhysicsWorld {
        WasmPhysicsWorld {
            inner: PhysicsWorld::new([gx, gy, gz]),
        }
    }

    /// Advance the simulation.
    ///
    /// `delta_time` is the elapsed wall-clock time in seconds since the last
    /// call.  Internally an accumulator ensures only fixed-timestep ticks run.
    pub fn step(&mut self, delta_time: f32) {
        self.inner.step(delta_time);
    }

    /// Add a body to the world.
    ///
    /// `shape_type` must be one of `"cuboid"` or `"ball"`.
    /// `dims` for a cuboid is `[hx, hy, hz]`; for a ball it is `[radius]`.
    /// `body_type_str` must be `"dynamic"` or `"static"`.
    ///
    /// Returns `true` on success, throws a `JsError` on failure.
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
        let body_type = match body_type_str {
            "dynamic" => BodyType::Dynamic,
            "static" => BodyType::Static,
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

    /// Set the linear and angular velocity of a body.
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
        self.inner
            .apply_impulse(entity_id, [ix, iy, iz])
            .map_err(|e| JsError::new(&e))
    }

    /// Return a `Float32Array` containing the current snapshot buffer.
    ///
    /// Layout: `[x, y, z, qx, qy, qz, qw, index]` per body, 8 floats each.
    /// The data is copied into JavaScript-owned memory, making it safe to hold
    /// across Wasm heap allocations.
    pub fn get_snapshot_buffer(&self) -> Float32Array {
        let snapshot = self.inner.get_snapshot();
        let buf = snapshot.to_buffer();
        Float32Array::from(buf.as_slice())
    }

    /// Return the number of bodies in the world.
    pub fn body_count(&self) -> usize {
        self.inner.body_count()
    }
}
