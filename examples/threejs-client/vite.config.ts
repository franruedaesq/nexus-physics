import { defineConfig } from "vite";
import { resolve } from "path";
import { fileURLToPath } from "url";

const __dirname = fileURLToPath(new URL(".", import.meta.url));

export default defineConfig({
  // Ensure Vite handles the .wasm files from the npm package correctly.
  // The @nexus-physics/core package exposes its WASM through the wasm-pack
  // "web" target, which uses `fetch()` to load the .wasm binary.  Vite needs
  // to be told to resolve ?url imports and copy those assets.
  optimizeDeps: {
    exclude: ["@nexus-physics/core"],
  },
  server: {
    headers: {
      // Required for SharedArrayBuffer / high-resolution timers (optional, but
      // good practice for performance-sensitive WebAssembly workloads).
      "Cross-Origin-Opener-Policy": "same-origin",
      "Cross-Origin-Embedder-Policy": "require-corp",
    },
  },
  resolve: {
    alias: {
      // When running the example directly from the monorepo (without a
      // published npm package) we explicitly point at the wasm-pack "web"
      // target JS entry point.  This is intentional: the example is a browser
      // application so we always want the fetch()-based WASM initialisation
      // path.  In a published install the "browser" condition in
      // @nexus-physics/core's package.json handles this automatically.
      "@nexus-physics/core": resolve(
        __dirname,
        "../../pkg/nexus_physics_wasm.js"
      ),
    },
  },
});
