# @nexus-physics/core

High-performance, headless 3D physics engine for JavaScript/TypeScript. Powered by Rust + Rapier3D, compiled to WebAssembly with zero-copy memory access and automatic TypeScript definitions.

## 🚀 Quick Start

```bash
npm install @nexus-physics/core
```

```typescript
import init, { WasmPhysicsWorld } from '@nexus-physics/core';

// Initialize the Wasm module
await init();

// Create world with gravity
const world = new WasmPhysicsWorld([0, -9.81, 0]);

// Add static floor
world.add_body({
  entity_id: 'floor',
  body_type: 'Static',
  shape: { type: 'Cuboid', hx: 10, hy: 0.5, hz: 10 },
  position: [0, -1, 0],
});

// Add dynamic ball
world.add_body({
  entity_id: 'ball',
  body_type: 'Dynamic',
  shape: { type: 'Ball', radius: 0.5 },
  position: [0, 5, 0],
});

// Step the simulation
world.step(1 / 60);

// Get zero-copy snapshot (Float32Array)
const buffer = world.get_snapshot_view();
console.log(`Ball position: (${buffer[1]}, ${buffer[2]}, ${buffer[3]})`);
```

## 📦 What You Get

- **Deterministic Physics**: Fixed timestep accumulator ensures reproducible behavior
- **Zero-Copy Snapshots**: Direct `Float32Array` views into Wasm memory—no garbage collection pressure
- **Type-Safe API**: Full TypeScript definitions auto-generated from Rust
- **Isomorphic**: Works in browser and Node.js from the same package
- **Domain-Agnostic**: Generic geometry and forces, not tied to games or robotics
- **Server-Authoritative Ready**: Perfect for multiplayer simulations with client prediction

## 📚 Core Concepts

### Body Types

```typescript
type BodyType = 'Dynamic' | 'Static' | 'KinematicPositionBased';
```

- **Dynamic**: Affected by forces and gravity. For moving objects.
- **Static**: Immovable terrain, walls, platforms.
- **KinematicPositionBased**: Programmatically positioned via `set_position()`. For elevators, moving platforms.

### Shape Types

```typescript
type Shape =
  | { type: 'Cuboid'; hx: number; hy: number; hz: number }
  | { type: 'Ball'; radius: number }
  | { type: 'Cylinder'; radius: number; half_height: number };
```

### Snapshot Buffer Layout

Each body in a snapshot occupies exactly **8 floats**:

```
[index, x, y, z, qx, qy, qz, qw, index, x, y, z, ...]
```

Access directly without parsing:

```typescript
const buffer = world.get_snapshot_view();
for (let i = 0; i < buffer.length; i += 8) {
  const id = buffer[i + 0]; // Snapshot entry index
  const x = buffer[i + 1]; // Position X
  const y = buffer[i + 2]; // Position Y
  const z = buffer[i + 3]; // Position Z
  const qx = buffer[i + 4]; // Quaternion X
  const qy = buffer[i + 5]; // Quaternion Y
  const qz = buffer[i + 6]; // Quaternion Z
  const qw = buffer[i + 7]; // Quaternion W
}
```

## 🎮 Common Patterns

### Multiplayer Game with Client Prediction

```typescript
const world = new WasmPhysicsWorld([0, -9.81, 0]);

// Game loop
function gameLoop(deltaTime) {
  // Apply local player input
  world.set_linear_velocity('player', inputVelocity);

  // Predict locally
  world.step(deltaTime);

  // Render from prediction
  const buffer = world.get_snapshot_view();
  updatePlayerMesh(buffer);

  // Periodically reconciliate with server
  if (shouldReconciliate()) {
    const serverSnapshot = await fetchServerSnapshot();
    world.set_position('other_player', serverSnapshot.position);
    world.set_rotation('other_player', serverSnapshot.rotation);
  }
}
```

### Three.js Integration

```typescript
import * as THREE from 'three';
import init, { WasmPhysicsWorld } from '@nexus-physics/core';

const world = new WasmPhysicsWorld([0, -9.81, 0]);
const meshes = new Map<string, THREE.Mesh>();

function render() {
  world.step(1 / 60);

  const buffer = world.get_snapshot_view();

  // Update all meshes directly from physics buffer
  for (let i = 0; i < buffer.length; i += 8) {
    const id = buffer[i];
    const mesh = meshes.get(String(id));
    if (mesh) {
      mesh.position.set(buffer[i + 1], buffer[i + 2], buffer[i + 3]);
      mesh.quaternion.set(
        buffer[i + 4],
        buffer[i + 5],
        buffer[i + 6],
        buffer[i + 7],
      );
    }
  }

  renderer.render(scene, camera);
  requestAnimationFrame(render);
}
```

### Robotics Simulation

```typescript
import { WasmPhysicsWorld } from '@nexus-physics/core';

const world = new WasmPhysicsWorld([0, -9.81, 0]);

// Add robot base and wheels
world.add_body({
  entity_id: 'robot_base',
  body_type: 'Dynamic',
  shape: { type: 'Cuboid', hx: 0.3, hy: 0.1, hz: 0.3 },
  position: [0, 0.5, 0],
});

// Control via velocity commands
function applyMotorCommand(linear_velocity, angular_velocity) {
  world.set_linear_velocity('robot_base', linear_velocity);
  world.set_angular_velocity('robot_base', angular_velocity);
}

// Simulate sensor reading (e.g., LiDAR)
function readSensors() {
  world.step(1 / 60);
  const snapshot = world.get_snapshot_view();
  // Process snapshot as needed
  return snapshot;
}
```

### Deterministic Replay

```typescript
const world = new WasmPhysicsWorld([0, -9.81, 0]);
const timeline: Array<{
  position: [number, number, number];
  velocity: [number, number, number];
}> = [];

// Record phase
function recordFrame() {
  world.step(1 / 60);
  timeline.push({
    position: world.get_position('player'),
    velocity: world.get_linear_velocity('player'),
  });
}

// Replay phase
function replay() {
  const newWorld = new WasmPhysicsWorld([0, -9.81, 0]);
  // Recreate bodies
  newWorld.add_body({
    /* ... */
  });

  for (const frame of timeline) {
    newWorld.set_position('player', frame.position);
    newWorld.set_linear_velocity('player', frame.velocity);
    newWorld.step(1 / 60);
  }
}
```

## 📖 API Reference

### World

```typescript
class WasmPhysicsWorld {
  constructor(gravity: [number, number, number]);

  step(deltaTime: number): void;
  gravity(): [number, number, number];

  add_body(config: {
    entity_id: string;
    body_type: 'Dynamic' | 'Static' | 'KinematicPositionBased';
    shape: Shape;
    position: [number, number, number];
  }): void;

  remove_body(entity_id: string): void;
  body_exists(entity_id: string): boolean;

  get_snapshot_view(): Float32Array;
  get_snapshot_json(): {
    entries: Array<{
      entity_id: string;
      position: [number, number, number];
      rotation: [number, number, number, number];
    }>;
  };

  set_linear_velocity(
    entity_id: string,
    velocity: [number, number, number],
  ): void;
  set_angular_velocity(
    entity_id: string,
    velocity: [number, number, number],
  ): void;
  apply_impulse(entity_id: string, impulse: [number, number, number]): void;

  set_position(entity_id: string, position: [number, number, number]): void;
  set_rotation(
    entity_id: string,
    rotation: [number, number, number, number],
  ): void;

  get_position(entity_id: string): [number, number, number];
  get_rotation(entity_id: string): [number, number, number, number];
  get_linear_velocity(entity_id: string): [number, number, number];
  get_angular_velocity(entity_id: string): [number, number, number];

  cast_ray(
    origin: [number, number, number],
    direction: [number, number, number],
    max_toi: number,
  ): number; // Distance to hit, or max_toi if no hit

  cast_ray_batch(
    origins: Float32Array, // [x, y, z, x, y, z, ...]
    directions: Float32Array, // [x, y, z, x, y, z, ...]
    max_toi: number,
  ): Float32Array; // [distance, distance, ...]
}
```

## 🎯 When to Use

✅ **Ideal for:**

- Multiplayer games with server-authoritative physics
- 3D web applications (Three.js, Babylon.js)
- Robotics and physics simulations
- Deterministic replay systems
- Real-time collision detection
- Networked motion synthesis

❌ **Not ideal for:**

- Softbody physics (cloth, hair)
- Deformable terrain
- Particle systems (use GPU instead)
- Passive rigid body stacking (CPU-bound)

## ⚡ Performance Tips

1. **Reuse Float32Array views**: Don't create new buffers every frame
2. **Batch updates**: Apply multiple forces before stepping
3. **Limit body count**: 500-1000 bodies per world on modern hardware
4. **Use appropriate shapes**: Spheres are faster than boxes
5. **Profile first**: Use Chrome DevTools to measure bottlenecks

## 🔧 Advanced: Node.js Usage

The package exports separate distributions for browser and Node.js. Import detection is automatic:

```javascript
// Node.js automatically gets the Node.js Wasm build
import init, { WasmPhysicsWorld } from '@nexus-physics/core';

// Or explicitly:
import initNode from '@nexus-physics/core/dist/node/nexus_physics_wasm.js';

// Browser sees the browser build via conditional exports
```

## 🐛 Troubleshooting

**Q: "Module not found" errors in bundler**

A: Ensure your bundler is configured for Wasm. Vite, Webpack 5+, and esbuild handle this automatically.

**Q: Physics feels jittery between frames**

A: You're likely stepping with variable delta times. The engine uses a fixed 60Hz timestep internally—pass the actual elapsed time and let the accumulator handle it.

**Q: Why is my snapshot empty?**

A: Only `Dynamic` and `Kinematic` bodies appear in snapshots. `Static` bodies are implicit in the world geometry.

## 📚 Further Reading

- [Full Project README](https://github.com/franruedaesq/nexus-physics)
- [Rapier3D Physics Engine](https://rapier.rs/)
- [Server-Authoritative Game Architecture](https://gabrielgambetta.com/client-server-game-architecture.html)
- [WebAssembly in Rust](https://rustwasm.org/)

## 📄 License

MIT License. See project repository for full license text.

## 🤝 Contributing

Found a bug? Have a feature request? Open an issue on the [GitHub repository](https://github.com/franruedaesq/nexus-physics).

---

Built with ❤️ in Rust & WebAssembly
