# Maintainers Guide for webarkitlib-rs

This document is intended for the core maintainers of the `webarkit/webarkitlib-rs` repository. It outlines the responsibilities, architectural mandates to enforce during code reviews, and the exact steps required to publish a new release.

## Current Maintainers

* **Walter Perdan** ([@kalwalt](https://github.com/kalwalt)) - Creator & Lead Maintainer

## 1. Code Review & Architectural Mandates

When reviewing Pull Requests, maintainers must ensure that the following core principles of the `webarkitlib-rs` project are strictly upheld:

* **Zero-FFI Policy:** Absolutely no C++ linking, `bindgen`, or `cc` is allowed. Every algorithm must be written in pure, idiomatic Rust.
* **Memory Safety:** Ensure Rust's ownership model is respected. Verify that buffers rely on `Vec<T>` or `Box<[T]>` and that there are no unnecessary memory allocations inside hot loops.
* **Feature Gating:** * Concurrency (`parallel` feature via Rayon) must have a sequential fallback.
    * Vectorization (`simd` feature via Pulp) must be optional and gracefully degrade to standard iterators or auto-vectorization when disabled.
    * WebAssembly (`wasm` feature) compatibility must be preserved.
* **Language:** All comments, docstrings, and commit messages **must** be in English.
* **Conventional Commits:** PR titles and commit messages must follow the Conventional Commits specification. PRs should be strictly squashed and merged.

## 2. Release Process

Publishing a new version requires a mix of manual changelog curation and automated CI/CD deployment. Follow these steps sequentially:

### Step 1: Pre-Release Checks
1. Ensure you are on the `dev` branch and it is up to date.
2. Verify that all CI checks (Formatting, Clippy, Tests for `parallel` and `simd`) are passing on the latest commit.

### Step 2: Bump the Version
Update the version number in the `Cargo.toml` file of the workspace and in package.json, then run the following command:

```bash
npm run build:wasm
```

to update the version in the generated wasm package. The version should follow semantic versioning (MAJOR.MINOR.PATCH) and reflect the nature of the changes since the last release.

### Step 3: Generate the Local Changelog
We use `git-cliff` to parse the conventional commits and update the historical changelog. Run the following command in the root directory:

```bash
npx git-cliff -u --prepend CHANGELOG.md
```

### Step 4: Commit everything

Commit the version bump, the changelog update and the updated wasm package. Following the Conventional Commits format, read the [CONTRIBUTING.md](CONTRIBUTING.md) file for more details on how to write a good commit message.

### Step 5: Push to Remote

Push the changes to the `dev` branch:

```bash
git push origin dev
```
then checkout the `main` branch and merge the `dev` branch into it:

```bash
git checkout main
git merge dev   
git push origin main
``` 
### Step 6: Create a GitHub tag

Create a new tag for the release using the version number X.Y.Z you just set in the `Cargo.toml` and `package.json` files:

```bash     
git tag -a vX.Y.Z -m "Release version X.Y.Z"
git push origin vX.Y.Z
``` 

This will trigger the CI/CD pipeline to publish the new version to crates.io and npm.
