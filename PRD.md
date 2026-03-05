# Nexus Physics: Master Development Plan

## 1. Product Requirements Document (PRD)

**Project Name:** `nexus-physics`
**Description:** A headless, domain-agnostic, server-authoritative 3D physics engine written in Rust. It utilizes `rapier3d` internally and compiles to both a native Rust crate (for backend servers) and a WebAssembly (Wasm) module with TypeScript bindings (for frontend/Node.js usage).

**Core Objectives:**

* **Headless by Design:** No rendering logic. It strictly calculates positions, rotations, and collisions.
* **Domain-Agnostic:** Works with abstract concepts (RigidBodies, Colliders) rather than domain-specific entities (like robots or cars).
* **Transport-Decoupled:** Returns deterministic binary snapshots (or JSON) of the world state. The consumer dictates the network layer (WebSockets, WebRTC, UDP).
* **Three.js Interoperability:** The TypeScript API must be easily consumable by rendering libraries like Three.js (e.g., extracting quaternions and vectors directly into Three.js objects).
* **TDD Methodology:** All features must be driven by tests. Write tests first, watch them fail, implement the logic, and refactor.

**Architecture:**

* **Core Crate (`nexus-physics-core`):** Wraps `rapier3d`. Manages the world, entity registration, physics stepping, and state snapshots.
* **Wasm Bindings (`nexus-physics-wasm`):** Uses `wasm-bindgen` to expose the core crate to JavaScript/TypeScript.
* **NPM Package (`@nexus-physics/core`):** The final bundled Wasm + auto-generated `.d.ts` definitions.

## 2. Execution Steps

Execute the following steps sequentially. For every step involving code, strictly follow the TDD cycle: Write the test, verify it fails, implement the code, verify the test passes, and refactor.

### Step 1: Repository & Workspace Scaffolding

Initialize the monorepo structure using Cargo Workspaces.

1. Create a root directory `nexus-physics`.
2. Create a root `Cargo.toml` configuring a workspace with two members: `core` and `bindings/wasm`.
3. Initialize the `core` library crate. Add `rapier3d` and `serde` as dependencies.
4. Initialize the `bindings/wasm` library crate. Add `core` (path dependency) and `wasm-bindgen` as dependencies. Set the crate type to `cdylib`.
5. Create a root `package.json` with scripts for testing (`cargo test`) and Wasm building (using `wasm-pack`).

### Step 2: Core Physics World Initialization (TDD)

Implement the foundational Physics World wrapper.

1. Write a Rust test in `core/src/world.rs` that instantiates a `PhysicsWorld` with a specific gravity vector and verifies the configuration is stored.
2. Implement the `PhysicsWorld` struct wrapping Rapier's `RigidBodySet`, `ColliderSet`, `PhysicsPipeline`, `IntegrationParameters`, and `IslandManager`.
3. Write a test to advance the simulation by one step (`world.step()`) and ensure it executes without panicking.
4. Implement the `step()` method using Rapier's pipeline.

### Step 3: Entity Registration (RigidBodies & Colliders)

**Objective:** Establish the API for populating the physics world with abstract geometric shapes and physical properties, completely decoupled from any domain-specific concepts (like "wheels" or "chassis").

* **Architecture & DX:** * We must map user-friendly IDs (e.g., `u32` or `String`) to Rapier's internal `RigidBodyHandle` and `ColliderHandle`. The frontend user should never touch a complex internal memory handle.
* Expose a builder pattern or configuration object (`RigidBodyConfig`) that accepts body types (`Dynamic`, `Static`, `KinematicPositionBased`) and collider shapes (`Cuboid`, `Sphere`, `Cylinder`).


* **TDD Implementation:**
* *Test:* Instantiate the `PhysicsWorld`. Call `add_body(Static, Cuboid)` to act as a floor, and `add_body(Dynamic, Sphere)` suspended in the air.
* *Assertion:* Verify that the engine's internal handle map contains exactly 2 entities and that attempting to retrieve an entity with an invalid ID returns a handled `Error` or `None`, rather than panicking.


### Step 4: Input Injection & Forces

**Objective:** Provide the interface for manipulating the simulation from the outside world (e.g., applying movement commands from a network payload, ROS 2 `/cmd_vel`, or UI inputs).

* **Architecture & DX:**
* The API must allow applying continuous forces (Linear/Angular Velocity) for motorized movement, and instant forces (Impulses) for sudden impacts or jumps.
* All methods must target the user-friendly IDs created in Phase 3.


* **TDD Implementation:**
* *Test:* Create a dynamic body at origin `[0, 0, 0]`. Apply a linear velocity of `[0.0, 10.0, 0.0]` (moving up the Y-axis). Call `world.step()`.
* *Assertion:* Read the body's position and assert that `Y > 0.0`. Test invalid inputs (applying forces to a `Static` body) to ensure the engine gracefully ignores them or logs a warning without crashing.

### Step 5: Spatial Queries (The "LiDAR" Engine)

**Objective:** Expose a headless raycasting pipeline to simulate distance sensors, line-of-sight checks, and object detection without requiring a rendering engine.

* **Architecture & DX:**
* Wrap Rapier's `QueryPipeline`.
* Implement two methods: `cast_ray(origin, direction, max_toi)` for single queries, and a highly optimized `cast_ray_batch(origins, directions, max_toi)` specifically designed to handle dense sensor arrays (like a 360-degree LiDAR) in a single CPU tick.
* Return an array of distances (Time of Impact - TOI). If a ray hits nothing, return a designated maximum distance value.


* **TDD Implementation:**
* *Test:* Place a static `Cuboid` wall at `X = 5.0`. Fire a ray from `X = 0.0` pointing directly along the positive X-axis (`[1.0, 0.0, 0.0]`).
* *Assertion:* Assert that the engine returns a hit at exactly distance `5.0`. Fire a second ray in the opposite direction (`[-1.0, 0.0, 0.0]`) and assert it returns `None` or the max range.

### Step 6: State Snapshot Generation

**Objective:** Extract the exact positional and rotational data of all moving entities in the world into a transport-agnostic format, optimized for network broadcasting.

* **Architecture & DX:**
* **Crucial Rule:** Avoid nested JSON objects for the tick loop. The engine must compile the data into a flat, contiguous array to prevent garbage collection spikes in JS.
* Structure the data conceptually as: `[ID, PosX, PosY, PosZ, RotX, RotY, RotZ, RotW]`.
* Only extract data for `Dynamic` and `Kinematic` bodies; `Static` bodies do not need to be broadcasted every frame.


* **TDD Implementation:**
* *Test:* Add one static floor and two dynamic falling cubes. Step the simulation. Call `world.get_snapshot()`.
* *Assertion:* Verify the output is a flat vector/array of floats. Assert the length of the array is exactly `2 * 8` (2 bodies, 8 floats per body: 1 ID, 3 Pos, 4 Quat).

### Step 7: WebAssembly Bridge & Memory Views

**Objective:** Safely export the Rust architecture to JavaScript/TypeScript environments (Browser and Node.js) with zero-copy memory access for maximum performance.

* **Architecture & DX:**
* Use `wasm-bindgen` to wrap the `PhysicsWorld`.
* Expose the Phase 6 snapshot and Phase 5 raycast results not by copying data into JavaScript arrays, but by returning a `js_sys::Float32Array::view` pointing directly to WebAssembly's linear memory.
* Auto-generate TypeScript `.d.ts` definitions so frontend developers get strict autocomplete for the methods.

* **TDD Implementation:**
* *Test:* Write a `#[wasm_bindgen_test]` that runs in a headless browser environment. Instantiate the Wasm class, add a body, step the physics, and call the memory view function.
* *Assertion:* Read the values directly from the JS `Float32Array` memory buffer and assert they perfectly match the expected physics calculations.

