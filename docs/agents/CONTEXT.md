# Agent Context Map & Repository Rules

This directory (`docs/agents/`) contains domain-specific knowledge, architecture guidelines, and specification files for AI coding agents operating on **WebARKitLib-rs**.

## 1. Project Core Mandate
Porting WebARKitLib (C/C++) to high-performance, side-effect-free, safe Rust (`WebARKitLib-rs`) targeting:
- `crates/core`: Pure Rust Computer Vision & AR tracking algorithms.
- `crates/wasm`: WebAssembly bindings exposing side-effect-free functions (accepting pixel buffers e.g. `&[u8]`, returning tracking matrices/coordinates).

## 2. Inviolable Scope Exclusions
Agents **MUST NOT** import, port, or generate code for:
1. **Video Capture / I/O**: (V4L2, DirectShow, QuickTime, `video.h`, `arVideo.h`). Video buffering is passed into WASM from JS or host environments.
2. **Rendering / OpenGL**: (`gsub.h`, `gsub_lite.h`, `arGL.h`, GLUT). Rendering is external.
3. **Multi-marker tracking**: (`arMulti.h`, `arMulti*.c`).

## 3. Key Documentation References
- [ARCHITECTURE.md](../../ARCHITECTURE.md): Architectural decisions and crate boundaries.
- [CLAUDE.md](../../CLAUDE.md): Idiomatic Rust conventions, logging macros (`arlog_*`), conventional commits format.
- [docs/wasm-js-integration.md](../wasm-js-integration.md): WASM memory management & JavaScript binding contracts.
- [docs/miri.md](../miri.md): Guidelines for memory safety validation with Miri.
