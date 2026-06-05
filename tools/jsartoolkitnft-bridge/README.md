# `jsartoolkitnft-bridge` — cross-stack parity reference for the Rust/C++ matchers

A small Node.js script that drives [jsartoolkitNFT-Node][1] over the
same NFT fixtures the Rust corner-error gate consumes and writes a
**`expected-js.json` sidecar** with the JS-stack matched_id +
3×4 transformation pose.

The Rust integration test
[`crates/core/tests/cross_stack_parity.rs`](../../crates/core/tests/cross_stack_parity.rs)
consumes that sidecar to gate `RustFreakMatcher` + `CppFreakMatcher`
against the JS-stack reference.

[1]: https://github.com/webarkit/jsartoolkitNFT

Filed under [jsartoolkitNFT#584][2] Track 2 / [WebARKitLib-rs#166][3] Track B / [#170][4].

[2]: https://github.com/webarkit/jsartoolkitNFT/issues/584
[3]: https://github.com/webarkit/WebARKitLib-rs/issues/166
[4]: https://github.com/webarkit/WebARKitLib-rs/issues/170

## Why JSON sidecar instead of live Node-in-CI?

CI on this repo is Rust-only. Adding a Node toolchain to every CI run
would add ~1-2 minutes per shard and a non-trivial maintenance
surface. By pre-generating the sidecar and committing it, the Rust
test stays Rust-only at run time. The trade-off: the sidecar can go
stale relative to the npm package or our submodule. The test surfaces
that explicitly (the failure message points contributors here to
regenerate).

## Regenerating `expected-js.json`

```sh
cd tools/jsartoolkitnft-bridge
npm install
npm run regen
```

That writes a fresh `expected-js.json` in this directory. Inspect the
diff, then commit it together with whatever change prompted the regen.

When to regenerate:
- After the `@webarkit/jsartoolkit-nft` dependency in
  [`package.json`](./package.json) is bumped.
- After jsartoolkitNFT itself bumps its WebARKitLib submodule
  (matters once [WebARKitLib#39][5] lands and the post-fix C++
  matcher ships in a new jsartoolkitNFT npm release).
- Whenever you add a new fixture to the `FIXTURES` array in
  [`run.js`](./run.js).

[5]: https://github.com/webarkit/WebARKitLib/pull/39

## What the sidecar contains

```json
{
  "schema": 1,
  "generated_with": "@webarkit/jsartoolkit-nft@1.9.0 via tools/jsartoolkitnft-bridge/run.js",
  "generated_at": "2026-06-01T21:57:58.358Z",
  "notes": "...",
  "per_frame": {
    "pinball-demo.jpg": {
      "loaded_marker_id": 0,
      "nft_data": {"id": 0, "width": 893, "height": 1117, "dpi": 120},
      "camera_matrix": [/* 16-element flattened 4×4 */],
      "first_match": {
        "id": 0,
        "error": 0.9179872274398804,
        "found": 1,
        "pose": [/* 12-element flattened 3×4, row-major */]
      }
    }
  }
}
```

The Rust gate reads only `loaded_marker_id` and `first_match.{id, pose}`
today; the other fields are informational.

## Current state (pre-#39 npm)

The published `@webarkit/jsartoolkit-nft@1.9.0` dist was built against
the **pre-#39** C++ matcher (`std::unordered_map` typedefs, with
libc++ iteration order baked into the WASM bytes). Its output matches
the "Linux pre-fix C++" baseline branch — `pose[0][2] ≈ 0.00159` on
`pinball-demo.jpg` rather than the post-fix canonical `≈ 0.064`.

Once [WebARKitLib#39][5] merges, jsartoolkitNFT bumps its WebARKitLib
submodule, rebuilds, and publishes a new npm release, the bridge's
`@webarkit/jsartoolkit-nft` dep will be bumped here and the sidecar
regenerated. At that point the JS reference, Rust, and C++ FFI should
all converge on the canonical values across all platforms.

## Why the script chdirs into `crates/core/examples/Data/`

jsartoolkitNFT-Node mounts the working directory into its Emscripten
NODEFS, then asks for marker assets at relative paths
(`camera_para.dat`, `pinball.fset3`, `pinball.fset`, `pinball.iset`).
We already have those files in
`crates/core/examples/Data/`, so the script does `process.chdir()` to
that directory rather than duplicating ~890 KB of marker data here.

The output `expected-js.json` is written back to this directory using
an absolute path, so it lands where the Rust test expects it.

## Out of scope

- Multi-frame regression baselines (matched_id should be stable, but
  drift in second / third / Nth-frame tracking poses is not gated
  here — that's a separate concern).
- Browser-side WASM (vs Node WASM) parity. They share the same WASM
  bytes, so the algorithmic output is identical; only the host I/O
  differs.

## References

- Rust test: [`crates/core/tests/cross_stack_parity.rs`](../../crates/core/tests/cross_stack_parity.rs)
- Issue tracking the broader cross-stack work: [WebARKitLib-rs#170][4]
- jsartoolkitNFT-side tracking: [jsartoolkitNFT#584][2]
