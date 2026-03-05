### 🚨 Addendum: Technical Constraints & AI Guardrails

To ensure the `nexus-physics` engine is truly production-ready, performant, and reliable, you must adhere to the following strict constraints while building:

#### 1. The Wasm-JS Memory Boundary (Zero-Copy is Mandatory)

**Do not serialize the physics state to JSON every frame.** JSON serialization/deserialization at 60Hz will destroy the garbage collector in the browser.

* **Requirement:** The `get_snapshot_buffer()` method must return a direct view into WebAssembly's linear memory.
* **Implementation:** Structure the snapshot data in Rust as a contiguous flat array (e.g., `Vec<f32>` where data is packed as `[id, x, y, z, qx, qy, qz, qw, id, x, ...]`). Expose this to TypeScript as a `Float32Array` view using `js_sys::Float32Array::view`.

#### 2. Determinism and the Fixed Timestep

For a server-authoritative model to work (especially with client-side prediction), the physics engine must be deterministic.

* **Requirement:** Do not tie the physics `step()` directly to the render loop's delta time.
* **Implementation:** Implement an "accumulator" pattern inside the `PhysicsWorld::step(delta_time: f32)` method. The physics engine should only advance in fixed intervals (e.g., exactly `1.0 / 60.0` seconds). If the delta time is larger, it should run multiple simulated ticks to catch up.

#### 3. Entity ID Mapping (Rust Borrow Checker Safety)

Rapier3D uses internal `RigidBodyHandle` and `ColliderHandle` types, which are essentially generational indices.

* **Requirement:** Do not expose Rapier's internal handles directly to the TypeScript/Node.js user. The user should be able to pass in their own string IDs (e.g., `"player_1"`) or receive a simple `u32` ID.
* **Implementation:** Maintain a `HashMap<String, RigidBodyHandle>` (or similar bidirectional map) inside `PhysicsWorld`. When the frontend calls `set_velocity("player_1", ...)`, Rust will look up the correct Rapier handle and apply the force.

#### 4. Strict TDD and AI Execution Rules

As an AI building this, you must follow these rules to avoid context-window collapse:

* **Do not write the whole app at once:** When I ask you to execute a step, *only* execute that step. Do not jump ahead to Wasm bindings if we are working on core Rust logic.
* **Tests must be behavioral:** Do not mock `rapier3d` in the tests. Instantiate a real Rapier physics world in the test environment, run the math, and assert that the floating-point numbers change as expected.
* **Wasm Targets:** When compiling `wasm-pack`, we will eventually need it to work in both Node.js (for the authoritative server) and the browser (for client prediction). Keep the Rust codebase agnostic so it compiles cleanly to both `--target nodejs` and `--target web`.

---
