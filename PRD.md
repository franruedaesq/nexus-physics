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

### Step 3: Entity Registration API (TDD)

Allow users to add shapes to the world.

1. Write a test that adds a static floor (Cuboid) and a dynamic sphere to the `PhysicsWorld` and asserts that the internal body count is 2.
2. Implement an `add_body` method. It should accept an agnostic configuration struct (e.g., `RigidBodyConfig` containing shape type, dimensions, position, and dynamic/static status).
3. Return a unique entity ID (e.g., a simple `u32` or Rapier's `RigidBodyHandle` wrapped in a custom type) to the caller.

### Step 4: Input Injection & Forces (TDD)

Implement the ability to move objects programmatically.

1. Write a test that creates a dynamic body, applies a linear velocity of `[0.0, 10.0, 0.0]`, steps the world once, and asserts the body's Y position has increased.
2. Implement `set_velocity(entity_id, linear, angular)` and `apply_impulse(entity_id, impulse)` methods.
3. Ensure error handling exists if an invalid `entity_id` is provided.

### Step 5: State Snapshot Generation (TDD)

Extract the world state into a transport-ready format.

1. Write a test that adds bodies, steps the physics, and calls `world.get_snapshot()`. Assert that the snapshot contains the exact new coordinates and quaternions of the dynamic bodies.
2. Implement a `Snapshot` struct that maps entity IDs to their current `[x, y, z]` and `[qx, qy, qz, qw]`.
3. Implement a method to serialize this snapshot into a flat binary array (`Vec<u8>`) or lightweight JSON format using `serde`.

### Step 6: WebAssembly Bridge Implementation (TDD)

Expose the Rust API to TypeScript.

1. In `bindings/wasm/src/lib.rs`, write a `#[wasm_bindgen]` struct called `WasmPhysicsWorld` that internally holds the `core::PhysicsWorld`.
2. Expose Wasm-compatible methods: `new()`, `add_body()`, `step()`, `set_velocity()`, and `get_snapshot_buffer()`.
3. Write Wasm headless tests (using `wasm-bindgen-test`) to instantiate the world, step it, and read the buffer entirely in a simulated JS environment.

### Step 7: NPM Packaging & Three.js Example

Configure the final build pipeline and prove interoperability.

1. Configure `wasm-pack build --target web` in the `package.json` to output to a `pkg` directory.
2. Create an `examples/threejs-client` directory.
3. Initialize a basic Vite + TypeScript + Three.js project.
4. Import the local Wasm package. Map the snapshot's positional and quaternion data directly to a `THREE.Mesh().position` and `THREE.Mesh().quaternion` in the render loop to prove seamless integration.
