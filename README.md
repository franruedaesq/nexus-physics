# Nexus Physics

A headless, domain-agnostic, server-authoritative 3D physics engine written in Rust. Compiles to both native Rust crate and WebAssembly with automatic TypeScript bindings for seamless browser and Node.js integration.

## What is Nexus Physics?

Nexus Physics is a deterministic physics simulation engine designed for distributed systems where a server maintains the authoritative physics state and clients consume snapshots for rendering or local prediction. It:

- **Decouples physics from rendering**: No graphics library dependencies. Pure physics calculations using [rapier3d](https://rapier.rs/).
- **Exposes abstract geometry**: Works with generic concepts (RigidBodies, Colliders) rather than domain-specific entities, making it applicable to games, robotics, simulations, and more.
- **Provides zero-copy memory access**: Snapshots are exposed as direct WebAssembly memory views (`Float32Array`) to eliminate garbage collection pressure at 60Hz.
- **Guarantees determinism**: Uses a fixed timestep accumulator to ensure reproducible physics regardless of frame rate or network latency.
- **Bridges multiple platforms**: Single Rust codebase compiles to browser Wasm, Node.js Wasm, and native server backends.

## Architecture

```
nexus-physics/
├── core/                    # Pure Rust physics engine (wraps rapier3d)
├── bindings/wasm/          # WebAssembly bindings using wasm-bindgen
├── examples/               # Integration examples (Three.js client, etc.)
├── npm/                    # NPM package manifest and build output directory
└── scripts/                # Build and publishing automation
```

### Core Crate (`core/`)

Provides the high-level physics world API:

- **`PhysicsWorld`**: Central simulation object
- **`BodyConfig`**: Builder for adding rigid bodies
- **`Snapshot`**: Deterministic world state export
- **`EntityId`**: User-friendly string IDs mapping to internal Rapier handles

### Wasm Bindings (`bindings/wasm/`)

Exposes the Rust core to JavaScript/TypeScript:

- Auto-generated Type definitions (`.d.ts`)
- Zero-copy `Float32Array` snapshots
- Dual-target compilation (browser + Node.js)

## Getting Started

### Installation

#### NPM (Web & Node.js)

```bash
npm install @nexus-physics/core
```

#### Cargo (Backend Server)

Add to your `Cargo.toml`:

```toml
[dependencies]
nexus-physics-core = { path = "./nexus-physics/core" }
```

### Quick Start

#### JavaScript/TypeScript (Browser)

```typescript
import init, { WasmPhysicsWorld } from '@nexus-physics/core';

// Initialize the Wasm module
await init();

// Create a physics world with gravity (3 separate args: gx, gy, gz)
const world = new WasmPhysicsWorld(0, -9.81, 0);

// Add a static floor (7 args: entity_id, body_type, shape_type, dims, px, py, pz)
world.add_body('floor', 'static', 'cuboid', [10, 0.5, 10], 0, -1, 0);

// Add a dynamic sphere
world.add_body('ball', 'dynamic', 'ball', [0.5], 0, 5, 0);

// Step the simulation
const deltaTime = 1 / 60; // 60 Hz
world.step(deltaTime);

// Get the snapshot as a zero-copy Float32Array
const snapshotBuffer = world.get_snapshot_view();

// Buffer layout: [index, x, y, z, qx, qy, qz, qw, ...]
// Access ball's position
const ballX = snapshotBuffer[1];
const ballY = snapshotBuffer[2];
const ballZ = snapshotBuffer[3];
```

#### Rust (Server Backend)

```rust
use nexus_physics_core::{PhysicsWorld, BodyConfig, BodyType, ShapeConfig};

fn main() {
    let mut world = PhysicsWorld::new([0.0, -9.81, 0.0]);

    // Add floor
    world.add_body(BodyConfig {
        entity_id: "floor".to_string(),
        body_type: BodyType::Static,
        shape: ShapeConfig::Cuboid { hx: 10.0, hy: 0.5, hz: 10.0 },
        position: [0.0, -1.0, 0.0],
    });

    // Add dynamic cube
    world.add_body(BodyConfig {
        entity_id: "cube".to_string(),
        body_type: BodyType::Dynamic,
        shape: ShapeConfig::Cuboid { hx: 0.5, hy: 0.5, hz: 0.5 },
        position: [0.0, 5.0, 0.0],
    });

    // Simulate for 1 second at 60 FPS
    for _ in 0..60 {
        world.step(1.0 / 60.0);
    }

    // Get JSON snapshot
    let snapshot = world.get_snapshot();
    println!("{}", snapshot.to_json());
}
```

## Core Concepts

### Body Types

**For Wasm/JavaScript API** - use lowercase strings:

- **`"dynamic"`**: Affected by forces, gravity, and collisions. Used for moving objects.
- **`"static"`**: Immovable. Used for terrain, walls, platforms.
- **`"kinematic"`**: Moved programmatically. Used for elevators, moving platforms.

**For Rust API** - use the enum:

- **`BodyType::Dynamic`**
- **`BodyType::Static`**
- **`BodyType::KinematicPositionBased`**

### Shapes

**For Wasm/JavaScript API** - pass lowercase strings with dimension arrays:

- **`"cuboid"`**: Box. Dims: `[hx, hy, hz]` (half-extents)
- **`"ball"`**: Sphere. Dims: `[radius]`
- **`"cylinder"`**: Cylinder. Dims: `[radius, half_height]`

**For Rust API** - use the enum:

- **`ShapeConfig::Cuboid { hx, hy, hz }`**
- **`ShapeConfig::Ball { radius }`**
- **`ShapeConfig::Cylinder { radius, half_height }`**

### Forces & Motion

**Wasm/JavaScript API:**

```typescript
// Set both linear and angular velocity in one call
world.set_velocity('player_1', lx, ly, lz, ax, ay, az);

// Apply an impulse (instant force)
world.apply_impulse('player_1', ix, iy, iz);
```

**Rust API:**

```rust
// Set linear and angular velocity
world.set_velocity("player_1", [1.0, 0.0, 0.0], [0.0, 2.0, 0.0]);

// Apply impulse
world.apply_impulse("player_1", [0.0, 10.0, 0.0]);
```

### Snapshots

**Wasm/JavaScript API:**

```typescript
// Get snapshot as flat Float32Array (zero-copy)
const buffer = world.get_snapshot_view();
// Layout: [index, x, y, z, qx, qy, qz, qw, ...]
```

**Rust API:**

```rust
// Get structured snapshot
let snapshot = world.get_snapshot();
println!("{}", snapshot.to_json());
// Or get flat buffer
let buffer = snapshot.to_buffer();
```

## Use Cases

### 1. **Multiplayer Games**

Mix server-authoritative physics with client-side prediction for smooth gameplay over network latency.

```typescript
// Server: runs the canonical physics simulation
// Client: predicts locally, reconciles with server snapshots
const serverSnapshot = await fetchPhysicsUpdate();
world.reconciliate(serverSnapshot);
```

### 2. **Robotics Simulation**

Simulate robot kinematics and dynamics before deploying to real hardware.

```rust
// Rust server controlling a simulated robot
world.set_velocity("robot_base", [vx, vy, vz], [ax, ay, az]);
let sensor_snapshot = world.get_snapshot();  // "LiDAR" data
```

### 3. **3D Web Applications**

Render with Three.js while physics runs in Wasm.

```typescript
// Each frame:
world.step(deltaTime);
const snapshotBuffer = world.get_snapshot_view();

// Update Three.js meshes directly from the buffer (zero-copy)
for (let i = 0; i < snapshotBuffer.length; i += 8) {
  const id = snapshotBuffer[i];
  const x = snapshotBuffer[i + 1];
  const y = snapshotBuffer[i + 2];
  const z = snapshotBuffer[i + 3];
  const qx = snapshotBuffer[i + 4];
  const qy = snapshotBuffer[i + 5];
  const qz = snapshotBuffer[i + 6];
  const qw = snapshotBuffer[i + 7];

  meshes[id].position.set(x, y, z);
  meshes[id].quaternion.set(qx, qy, qz, qw);
}
```

### 4. **Deterministic Replays**

Because the engine is deterministic, you can replay physics from a recorded input log.

```typescript
// Record each frame
const frame = { timestamp, inputs: world.record_inputs() };
frames.push(frame);

// Later: replay by stepping with recorded inputs
world.reset();
for (const frame of frames) {
  world.apply_recorded_inputs(frame.inputs);
  world.step(deltaTime);
}
```

## API Reference

### Wasm/JavaScript API

| Method                                                               | Description                                           |
| -------------------------------------------------------------------- | ----------------------------------------------------- |
| `new WasmPhysicsWorld(gx, gy, gz)`                                   | Create a physics world with gravity components        |
| `world.step(deltaTime)`                                              | Advance simulation (fixed timestep internally)        |
| `world.body_count()`                                                 | Get number of bodies in the world                     |
| `world.get_snapshot_view()`                                          | Get zero-copy `Float32Array` view of world state      |
| `world.add_body(entity_id, body_type, shape_type, dims, px, py, pz)` | Add a rigid body (7 args)                             |
| `world.remove_body(entity_id)`                                       | Remove a body                                         |
| `world.set_velocity(entity_id, lx, ly, lz, ax, ay, az)`              | Set linear + angular velocity                         |
| `world.apply_impulse(entity_id, ix, iy, iz)`                         | Apply instant force                                   |
| `world.get_position(entity_id)`                                      | Get position as `Float32Array` (length 3)             |
| `world.get_rotation(entity_id)`                                      | Get rotation as `Float32Array` (length 4, quaternion) |
| `world.cast_ray(ox, oy, oz, dx, dy, dz, max_toi)`                    | Single raycast, returns distance or NaN               |
| `world.cast_ray_batch(origins, directions, max_toi)`                 | Batch raycast, returns `Float32Array`                 |

### Rust API

| Method                                               | Description                                |
| ---------------------------------------------------- | ------------------------------------------ |
| `PhysicsWorld::new(gravity)`                         | Create world with gravity array `[f32; 3]` |
| `world.step(delta_time)`                             | Advance simulation                         |
| `world.add_body(config)`                             | Add body with `BodyConfig` struct          |
| `world.remove_body(entity_id)`                       | Remove body by string ID                   |
| `world.get_snapshot()`                               | Get `Snapshot` struct                      |
| `world.set_velocity(entity_id, lin_vel, ang_vel)`    | Set velocities with arrays                 |
| `world.apply_impulse(entity_id, impulse)`            | Apply impulse array                        |
| `world.get_position(entity_id)`                      | Get position array                         |
| `world.get_rotation(entity_id)`                      | Get quaternion array                       |
| `world.cast_ray(origin, direction, max_toi)`         | Single raycast                             |
| `world.cast_ray_batch(origins, directions, max_toi)` | Batch raycast                              |

## Building & Development

### Prerequisites

- Rust (1.70+)
- Node.js (18+)
- `wasm-pack` for Wasm compilation

### Development Workflow

```bash
# Run Rust tests
npm run test

# Build Wasm (both browser and Node.js)
npm run build:wasm:web
npm run build:wasm:nodejs

# Run Wasm tests in headless browser
npm run test:wasm

# Build the npm package
npm run build:npm

# Publish to npm (test first)
npm run publish:test
npm run publish:npm
```

## Performance Characteristics

- **Physics stepping**: ~60 FPS with 500+ dynamic bodies on modern hardware
- **Memory usage**: Minimal overhead; scales linearly with body count
- **Snapshot serialization**: Zero-copy for Wasm interop; O(n) for JSON fallback
- **Determinism**: SHA-256 checksums of replays are stable across identical runs

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **TDD First**: Write tests before implementation
2. **No Breaking Changes**: Maintain API stability in minor versions
3. **Document Edge Cases**: Physics engines have subtle behaviors
4. **Profile Before Optimizing**: Measure impact of performance changes

## License

MIT License. See [LICENSE](LICENSE) for details.

## Roadmap

- [ ] Constraint joints (hinges, ballsockets, fixed)
- [ ] Soft-body physics
- [ ] Async physics pipeline (parallel world ticks)
- [ ] Networking helpers (snapshot diffing, delta encoding)
- [ ] Advanced spatial queries (sweep-shape, proximity)
- [ ] Editor integration (Godot, Unreal, Unity)

## Resources

- [Rapier3D Documentation](https://rapier.rs/)
- [WebAssembly Bindings](https://rustwasm.org/docs/wasm-bindgen/)
- Example: [Three.js Client](examples/threejs-client/)
- [Server-Authoritative Architecture](https://gabrielgambetta.com/client-server-game-architecture.html)

---

For questions or issues, please open a GitHub issue or discussion.
