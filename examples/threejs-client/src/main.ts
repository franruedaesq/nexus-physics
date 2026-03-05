/**
 * main.ts – Nexus Physics × Three.js proof-of-concept
 *
 * Demonstrates the zero-copy bridge between the Wasm physics engine and
 * Three.js:
 *
 *   1. Initialise the Wasm module and create a WasmPhysicsWorld.
 *   2. Spawn a static floor + 100 dynamic cubes.
 *   3. Each frame: call world.step(dt), obtain the Float32Array via
 *      world.get_snapshot_view() (zero-copy – no GC pressure), and directly
 *      write position/quaternion data into a Three.js InstancedMesh matrix.
 *
 * Snapshot buffer layout per body (8 floats):
 *   [index, x, y, z, qx, qy, qz, qw]
 */

import * as THREE from "three";
// The path alias in vite.config.ts points this at the wasm-pack "web" output
// when running from the monorepo.  In a published npm install it resolves via
// the "browser" condition in @nexus-physics/core's package.json.
import init, { WasmPhysicsWorld } from "@nexus-physics/core";

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
const NUM_CUBES = 100;
/** Half-extent of each dynamic cube, in metres. */
const CUBE_HALF = 0.5;
/** Floor dimensions: a thin, wide slab. */
const FLOOR_HALF_X = 30;
const FLOOR_HALF_Y = 0.2;
const FLOOR_HALF_Z = 30;
/** Height from which cubes are dropped. */
const DROP_HEIGHT = 20;
/** Floats per body in the snapshot buffer. */
const FLOATS_PER_BODY = 8;

// ---------------------------------------------------------------------------
// Three.js scene setup
// ---------------------------------------------------------------------------
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0a0f);
scene.fog = new THREE.Fog(0x0a0a0f, 40, 100);

const camera = new THREE.PerspectiveCamera(
  60,
  window.innerWidth / window.innerHeight,
  0.1,
  200
);
camera.position.set(0, 18, 45);
camera.lookAt(0, 5, 0);

// Lighting
const ambient = new THREE.AmbientLight(0x223344, 2);
scene.add(ambient);

const sun = new THREE.DirectionalLight(0xffffff, 3);
sun.position.set(15, 30, 20);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
sun.shadow.camera.near = 0.5;
sun.shadow.camera.far = 120;
sun.shadow.camera.left = -40;
sun.shadow.camera.right = 40;
sun.shadow.camera.top = 40;
sun.shadow.camera.bottom = -40;
scene.add(sun);

// Static floor mesh
const floorGeo = new THREE.BoxGeometry(
  FLOOR_HALF_X * 2,
  FLOOR_HALF_Y * 2,
  FLOOR_HALF_Z * 2
);
const floorMat = new THREE.MeshStandardMaterial({ color: 0x1a2233 });
const floorMesh = new THREE.Mesh(floorGeo, floorMat);
floorMesh.position.y = 0;
floorMesh.receiveShadow = true;
scene.add(floorMesh);

// InstancedMesh for the dynamic cubes (one draw call for all 100)
const cubeGeo = new THREE.BoxGeometry(
  CUBE_HALF * 2,
  CUBE_HALF * 2,
  CUBE_HALF * 2
);
const cubeMat = new THREE.MeshStandardMaterial({
  color: 0x00e5ff,
  metalness: 0.3,
  roughness: 0.4,
});
const instancedCubes = new THREE.InstancedMesh(cubeGeo, cubeMat, NUM_CUBES);
instancedCubes.castShadow = true;
instancedCubes.receiveShadow = true;
scene.add(instancedCubes);

// Pre-allocate reusable Three.js helpers (no per-frame GC allocations)
const _pos = new THREE.Vector3();
const _quat = new THREE.Quaternion();
const _scale = new THREE.Vector3(1, 1, 1);
const _matrix = new THREE.Matrix4();

// ---------------------------------------------------------------------------
// Resize handler
// ---------------------------------------------------------------------------
window.addEventListener("resize", () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------
const elFps = document.getElementById("fps")!;
const elBodies = document.getElementById("bodies")!;

let frameCount = 0;
let lastHudUpdate = 0;

// ---------------------------------------------------------------------------
// Physics world bootstrap (async – wasm init)
// ---------------------------------------------------------------------------
async function bootstrap(): Promise<void> {
  // Initialise the Wasm module.  For the wasm-pack "web" target this fetches
  // the .wasm binary; wasm-pack "nodejs" does a synchronous require() instead.
  await init();

  // Create the physics world with Earth-like gravity.
  const world = new WasmPhysicsWorld(0, -9.81, 0);

  // Static floor: a large flat cuboid sitting at y = 0.
  world.add_body(
    "floor",
    "static",
    "cuboid",
    new Float32Array([FLOOR_HALF_X, FLOOR_HALF_Y, FLOOR_HALF_Z]),
    0,
    0,
    0
  );

  // Spawn 100 dynamic cubes in a 10×10 grid dropped from DROP_HEIGHT.
  for (let i = 0; i < NUM_CUBES; i++) {
    const col = i % 10;
    const row = Math.floor(i / 10);
    const x = (col - 4.5) * 2.5;
    const z = (row - 4.5) * 2.5;
    // Stagger drop heights so cubes don't all collide at once.
    const y = DROP_HEIGHT + row * 1.5;
    world.add_body(
      `cube_${i}`,
      "dynamic",
      "cuboid",
      new Float32Array([CUBE_HALF, CUBE_HALF, CUBE_HALF]),
      x,
      y,
      z
    );
  }

  elBodies.textContent = `Bodies: ${world.body_count()}`;

  // -------------------------------------------------------------------------
  // Render / physics loop
  // -------------------------------------------------------------------------
  let previousTime = performance.now();

  function loop(nowMs: number): void {
    requestAnimationFrame(loop);

    const deltaTime = Math.min((nowMs - previousTime) / 1000, 0.1);
    previousTime = nowMs;

    // --- Physics step -------------------------------------------------------
    // Pass the real wall-clock delta.  The Wasm engine's accumulator ensures
    // that only fixed-timestep (1/60 s) ticks are actually simulated,
    // guaranteeing determinism regardless of frame rate.
    world.step(deltaTime);

    // --- Zero-copy snapshot bridge ------------------------------------------
    // get_snapshot_view() returns a Float32Array VIEW into Wasm linear memory.
    // No data is copied; no GC objects are created per frame.
    const snapshot: Float32Array = world.get_snapshot_view();

    const numBodies = snapshot.length / FLOATS_PER_BODY;

    for (let b = 0; b < numBodies; b++) {
      const offset = b * FLOATS_PER_BODY;
      // Slot 0: body index (used as instanced mesh index).
      const idx = snapshot[offset] | 0;
      // Slots 1-3: position
      _pos.set(snapshot[offset + 1], snapshot[offset + 2], snapshot[offset + 3]);
      // Slots 4-7: quaternion
      _quat.set(
        snapshot[offset + 4],
        snapshot[offset + 5],
        snapshot[offset + 6],
        snapshot[offset + 7]
      );
      // Compose and upload matrix (no allocation – reuses pre-allocated objects)
      _matrix.compose(_pos, _quat, _scale);
      instancedCubes.setMatrixAt(idx, _matrix);
    }

    instancedCubes.instanceMatrix.needsUpdate = true;

    // --- Render -------------------------------------------------------------
    renderer.render(scene, camera);

    // --- HUD ----------------------------------------------------------------
    frameCount++;
    if (nowMs - lastHudUpdate >= 500) {
      const fps = Math.round((frameCount * 1000) / (nowMs - lastHudUpdate));
      elFps.textContent = `FPS: ${fps}`;
      frameCount = 0;
      lastHudUpdate = nowMs;
    }
  }

  requestAnimationFrame(loop);
}

bootstrap().catch((err) => {
  console.error("Failed to initialise nexus-physics:", err);
});
