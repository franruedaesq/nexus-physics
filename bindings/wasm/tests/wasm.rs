//! Integration tests for the `nexus-physics-wasm` crate.
//!
//! These tests run inside a headless browser using `wasm-bindgen-test`.
//! Execute them with:
//!
//! ```sh
//! wasm-pack test bindings/wasm --headless --firefox
//! ```
//!
//! Each test exercises the zero-copy `Float32Array` memory-view API and
//! verifies that the values read directly from Wasm linear memory match the
//! expected physics calculations.

#![cfg(target_arch = "wasm32")]

use nexus_physics_wasm::WasmPhysicsWorld;
use wasm_bindgen_test::*;

wasm_bindgen_test_configure!(run_in_browser);

// ---------------------------------------------------------------------------
// Snapshot view tests (Step 6)
// ---------------------------------------------------------------------------

/// Verify that `get_snapshot_view` returns a zero-copy `Float32Array` whose
/// layout matches `[index, x, y, z, qx, qy, qz, qw]` and that after one
/// fixed-timestep the body has begun to fall under gravity.
#[wasm_bindgen_test]
fn test_snapshot_view_buffer_layout_and_gravity() {
    let mut world = WasmPhysicsWorld::new(0.0, -9.81, 0.0);

    world
        .add_body("ball", "dynamic", "ball", &[0.5_f32], 0.0, 10.0, 0.0)
        .expect("add_body should succeed");

    // Advance by exactly one fixed timestep (1/60 s).
    world.step(1.0_f32 / 60.0);

    let view = world.get_snapshot_view();

    // One dynamic body → 8 floats in the buffer.
    assert_eq!(view.length(), 8, "buffer length must be 8 for one body");

    // Slot 0: body index (0-based).
    assert_eq!(view.get_index(0), 0.0_f32, "index slot must be 0");

    // Slots 1 (x) and 3 (z): no horizontal force applied, should be ~0.
    assert!(
        (view.get_index(1)).abs() < 1e-5_f32,
        "x must be ~0, got {}",
        view.get_index(1)
    );
    assert!(
        (view.get_index(3)).abs() < 1e-5_f32,
        "z must be ~0, got {}",
        view.get_index(3)
    );

    // Slot 2 (y): gravity must have pulled the body below its starting height.
    let y = view.get_index(2);
    assert!(y < 10.0_f32, "body must have fallen: y = {}", y);
    assert!(y > 9.95_f32, "body should not have fallen too far in one step: y = {}", y);

    // Slots 4-6 (qx, qy, qz): no torque → rotation axes near zero.
    assert!((view.get_index(4)).abs() < 1e-5_f32, "qx must be ~0");
    assert!((view.get_index(5)).abs() < 1e-5_f32, "qy must be ~0");
    assert!((view.get_index(6)).abs() < 1e-5_f32, "qz must be ~0");

    // Slot 7 (qw): identity rotation → ~1.
    assert!(
        (view.get_index(7) - 1.0_f32).abs() < 1e-5_f32,
        "qw must be ~1 (identity rotation), got {}",
        view.get_index(7)
    );
}

/// Verify that a world with no gravity and a body given an explicit velocity
/// produces a snapshot whose position exactly matches one step of kinematic
/// integration: `x_new = x_old + vx * dt`.
#[wasm_bindgen_test]
fn test_snapshot_view_deterministic_no_gravity() {
    // No gravity so the only motion comes from the velocity we set.
    let mut world = WasmPhysicsWorld::new(0.0, 0.0, 0.0);

    world
        .add_body("cube", "dynamic", "cuboid", &[0.5_f32, 0.5, 0.5], 0.0, 0.0, 0.0)
        .expect("add_body should succeed");

    // Velocity: 1 m/s along +X.
    world
        .set_velocity("cube", 1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        .expect("set_velocity should succeed");

    // One tick.
    let dt = 1.0_f32 / 60.0;
    world.step(dt);

    let view = world.get_snapshot_view();

    assert_eq!(view.length(), 8);

    // After one symplectic-Euler tick: x = 0 + vx * dt = 1/60.
    let expected_x = 1.0_f32 / 60.0;
    let x = view.get_index(1);
    assert!(
        (x - expected_x).abs() < 1e-5_f32,
        "x after one step must be ~{:.6}, got {:.6}",
        expected_x,
        x
    );

    // y and z must remain 0.
    assert!((view.get_index(2)).abs() < 1e-5_f32, "y must be ~0");
    assert!((view.get_index(3)).abs() < 1e-5_f32, "z must be ~0");
}

/// Verify that static bodies are excluded from the snapshot (they never move).
#[wasm_bindgen_test]
fn test_snapshot_view_excludes_static_bodies() {
    let mut world = WasmPhysicsWorld::new(0.0, -9.81, 0.0);

    world
        .add_body(
            "floor",
            "static",
            "cuboid",
            &[50.0_f32, 0.1, 50.0],
            0.0,
            0.0,
            0.0,
        )
        .expect("add floor");

    world
        .add_body("ball", "dynamic", "ball", &[0.5_f32], 0.0, 5.0, 0.0)
        .expect("add ball");

    world.step(1.0_f32 / 60.0);

    let view = world.get_snapshot_view();

    // Only the dynamic ball appears; the static floor is excluded.
    assert_eq!(
        view.length(),
        8,
        "snapshot must contain exactly one body (the dynamic ball)"
    );
}

// ---------------------------------------------------------------------------
// Raycast-batch view tests (Step 5)
// ---------------------------------------------------------------------------

/// Verify that `cast_ray_batch` returns a zero-copy `Float32Array` view and
/// that the TOI for a ray aimed directly at a static wall is the expected
/// distance.
#[wasm_bindgen_test]
fn test_raycast_batch_view_hit() {
    // No gravity — we only care about geometry.
    let mut world = WasmPhysicsWorld::new(0.0, 0.0, 0.0);

    // Place a thin wall at x = 5.0; half-extent along X is 0.5, so the near
    // face is at x = 4.5.
    world
        .add_body(
            "wall",
            "static",
            "cuboid",
            &[0.5_f32, 5.0, 5.0],
            5.0,
            0.0,
            0.0,
        )
        .expect("add wall");

    world.update_query_pipeline();

    // Single ray: origin [0,0,0], direction [1,0,0].
    let origins: Vec<f32> = vec![0.0, 0.0, 0.0];
    let directions: Vec<f32> = vec![1.0, 0.0, 0.0];
    let max_toi = 100.0_f32;

    let view = world
        .cast_ray_batch(&origins, &directions, max_toi)
        .expect("cast_ray_batch should succeed");

    assert_eq!(view.length(), 1, "one ray → one TOI result");

    let toi = view.get_index(0);
    assert!(
        toi.is_finite(),
        "ray aimed at the wall must produce a finite TOI, got {}",
        toi
    );
    // The near face of the wall is at x = 5.0 − 0.5 = 4.5.
    assert!(
        (toi - 4.5_f32).abs() < 0.1_f32,
        "TOI must be ~4.5, got {}",
        toi
    );
}

/// Verify that a ray that misses all geometry returns `max_toi`.
#[wasm_bindgen_test]
fn test_raycast_batch_view_miss() {
    let mut world = WasmPhysicsWorld::new(0.0, 0.0, 0.0);

    world
        .add_body(
            "wall",
            "static",
            "cuboid",
            &[0.5_f32, 5.0, 5.0],
            5.0,
            0.0,
            0.0,
        )
        .expect("add wall");

    world.update_query_pipeline();

    // Fire ray in the opposite direction — away from the wall.
    let origins: Vec<f32> = vec![0.0, 0.0, 0.0];
    let directions: Vec<f32> = vec![-1.0, 0.0, 0.0];
    let max_toi = 100.0_f32;

    let view = world
        .cast_ray_batch(&origins, &directions, max_toi)
        .expect("cast_ray_batch should succeed");

    assert_eq!(view.length(), 1);

    let toi = view.get_index(0);
    assert_eq!(
        toi, max_toi,
        "a ray that misses must return max_toi, got {}",
        toi
    );
}

/// Verify that `cast_ray_batch` correctly handles multiple rays at once.
#[wasm_bindgen_test]
fn test_raycast_batch_view_multiple_rays() {
    let mut world = WasmPhysicsWorld::new(0.0, 0.0, 0.0);

    world
        .add_body(
            "wall",
            "static",
            "cuboid",
            &[0.5_f32, 5.0, 5.0],
            5.0,
            0.0,
            0.0,
        )
        .expect("add wall");

    world.update_query_pipeline();

    // Two rays: first hits the wall, second misses.
    let origins: Vec<f32> = vec![
        0.0, 0.0, 0.0, // ray 0 origin
        0.0, 0.0, 0.0, // ray 1 origin
    ];
    let directions: Vec<f32> = vec![
        1.0, 0.0, 0.0,  // ray 0: toward wall
        -1.0, 0.0, 0.0, // ray 1: away from wall
    ];
    let max_toi = 100.0_f32;

    let view = world
        .cast_ray_batch(&origins, &directions, max_toi)
        .expect("cast_ray_batch should succeed");

    assert_eq!(view.length(), 2, "two rays → two TOI results");

    let toi_hit = view.get_index(0);
    assert!(
        toi_hit.is_finite() && (toi_hit - 4.5_f32).abs() < 0.1_f32,
        "ray 0 (hit) must be ~4.5, got {}",
        toi_hit
    );

    let toi_miss = view.get_index(1);
    assert_eq!(
        toi_miss, max_toi,
        "ray 1 (miss) must be max_toi, got {}",
        toi_miss
    );
}
