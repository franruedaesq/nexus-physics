#!/usr/bin/env node
/**
 * build-npm.mjs
 *
 * Orchestrates a dual-target wasm-pack build and assembles the outputs into a
 * single, isomorphic NPM package inside the `npm/` directory.
 *
 * Usage:
 *   node scripts/build-npm.mjs
 *
 * Outputs:
 *   npm/dist/web/   – --target web build (Vite / browser bundlers)
 *   npm/dist/node/  – --target nodejs build (Node.js / authoritative servers)
 *   npm/package.json – Universal package manifest with conditional exports
 */

import { execSync } from "child_process";
import {
  cpSync,
  mkdirSync,
  readFileSync,
  writeFileSync,
  rmSync,
} from "fs";
import { join, dirname } from "path";
import { fileURLToPath } from "url";

const __dirname = dirname(fileURLToPath(import.meta.url));
const ROOT = join(__dirname, "..");
const NPM_DIR = join(ROOT, "npm");
const DIST_WEB = join(NPM_DIR, "dist", "web");
const DIST_NODE = join(NPM_DIR, "dist", "node");

/** Run a shell command, streaming stdout/stderr to the parent process. */
function run(cmd, cwd = ROOT) {
  console.log(`\n▶  ${cmd}`);
  execSync(cmd, { cwd, stdio: "inherit" });
}

/** Remove a directory if it exists, then recreate it. */
function cleanDir(dir) {
  rmSync(dir, { recursive: true, force: true });
  mkdirSync(dir, { recursive: true });
}

// ---------------------------------------------------------------------------
// 1. Build both wasm-pack targets into temporary staging directories.
// ---------------------------------------------------------------------------
const TMP_WEB = join(ROOT, "pkg");
const TMP_NODE = join(ROOT, "pkg-node");

console.log("=== Building web target ===");
run(
  `wasm-pack build bindings/wasm --target web --out-dir ../../pkg --out-name nexus_physics_wasm`
);

console.log("\n=== Building Node.js target ===");
run(
  `wasm-pack build bindings/wasm --target nodejs --out-dir ../../pkg-node --out-name nexus_physics_wasm`
);

// ---------------------------------------------------------------------------
// 2. Copy outputs into the unified npm/dist/{web,node} trees.
// ---------------------------------------------------------------------------
console.log("\n=== Assembling npm/ distribution ===");

cleanDir(DIST_WEB);
cleanDir(DIST_NODE);

// Copy every file produced by wasm-pack (JS glue, .wasm binary, .d.ts, etc.)
// Skip the generated package.json – we supply our own.
function copyWasmPackOutput(src, dest) {
  cpSync(src, dest, {
    recursive: true,
    filter: (srcPath) => !srcPath.endsWith("package.json"),
  });
}

copyWasmPackOutput(TMP_WEB, DIST_WEB);
copyWasmPackOutput(TMP_NODE, DIST_NODE);

// ---------------------------------------------------------------------------
// 3. Read the version from the wasm-pack-generated package to keep them in sync.
// ---------------------------------------------------------------------------
const generatedPkg = JSON.parse(
  readFileSync(join(TMP_WEB, "package.json"), "utf8")
);
const version = generatedPkg.version ?? "0.1.0";

// ---------------------------------------------------------------------------
// 4. Write the universal package.json with Node.js conditional exports.
//
//    Bundlers (Vite/Webpack) that understand the "browser" condition will pick
//    up the --target web build.  A plain Node.js require() / import will fall
//    through to the "node" condition and get the --target nodejs build.
// ---------------------------------------------------------------------------
const universalPackage = {
  name: "@nexus-physics/core",
  version,
  description:
    "A headless, domain-agnostic, server-authoritative 3D physics engine (isomorphic Wasm)",
  keywords: ["physics", "wasm", "rapier3d", "3d", "isomorphic"],
  license: "MIT",
  repository: {
    type: "git",
    url: "https://github.com/franruedaesq/nexus-physics.git",
  },
  // Conditional exports: Node.js → node target, bundlers → web target.
  exports: {
    ".": {
      browser: "./dist/web/nexus_physics_wasm.js",
      node: "./dist/node/nexus_physics_wasm.js",
      import: "./dist/web/nexus_physics_wasm.js",
      require: "./dist/node/nexus_physics_wasm.js",
      default: "./dist/web/nexus_physics_wasm.js",
    },
  },
  // Legacy entry-points for older tooling that ignores "exports".
  main: "./dist/node/nexus_physics_wasm.js",
  module: "./dist/web/nexus_physics_wasm.js",
  browser: "./dist/web/nexus_physics_wasm.js",
  // TypeScript definitions are identical between targets.
  types: "./dist/web/nexus_physics_wasm.d.ts",
  // Ship the .wasm files alongside the JS glue.
  files: ["dist/"],
  sideEffects: false,
};

writeFileSync(
  join(NPM_DIR, "package.json"),
  JSON.stringify(universalPackage, null, 2) + "\n"
);

console.log("\n✅  npm/ package assembled successfully.");
console.log(`    Version : ${version}`);
console.log(`    Web     : ${DIST_WEB}`);
console.log(`    Node    : ${DIST_NODE}`);
console.log(`    Manifest: ${join(NPM_DIR, "package.json")}`);
