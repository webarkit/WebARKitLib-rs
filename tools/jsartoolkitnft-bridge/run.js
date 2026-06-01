/*
 * run.js
 * WebARKitLib-rs — jsartoolkitNFT-Node cross-stack parity bridge
 *
 * Refs: jsartoolkitNFT#584 Track 2, WebARKitLib-rs#170, #166 Track B.
 *
 * Drives the Node entry point of @webarkit/jsartoolkit-nft over the
 * same fixtures used by the Rust corner-error gate, captures the
 * marker IDs + transformation matrices it reports, and writes a
 * paste-friendly `expected-js.json` sidecar.
 *
 * The Rust integration test `crates/core/tests/cross_stack_parity.rs`
 * consumes this sidecar and compares it against what `RustFreakMatcher`
 * + `CppFreakMatcher` produce on the same frames.
 *
 * Workflow:
 *   1. `npm install` inside this directory.
 *   2. `npm run regen` (or `node run.js`).
 *   3. Inspect `expected-js.json`. Commit it together with any
 *      Rust-side changes that depend on it.
 *
 * Run from this directory; the script changes cwd to
 * `crates/core/examples/Data/` so jsartoolkitNFT-Node's Emscripten
 * NODEFS sees the marker assets at the relative paths it expects
 * (`camera_para.dat`, `pinball.fset3`, `pinball.fset`, `pinball.iset`).
 */

'use strict';

const path = require('path');
const fs = require('fs');
const sharp = require('sharp');
const jsartoolkitNFT = require('@webarkit/jsartoolkit-nft/node');

const SCRIPT_DIR = __dirname;
const REPO_ROOT = path.resolve(SCRIPT_DIR, '..', '..');
const DATA_DIR = path.join(
    REPO_ROOT, 'crates', 'core', 'examples', 'Data'
);

// Fixtures to drive. Add more here as they are added to the corner-error
// gate; the Rust test discovers them by matching against this list's
// `name` field.
//
// Width / height are the post-resize dimensions used by the Rust gate
// (camera_para.dat is rescaled to match in both stacks).
const FIXTURES = [
    { name: 'pinball-demo.jpg', width: 2000, height: 1500 },
    // Additional fixtures (e.g. screen-capture seq* shots) can be added
    // here when they're known to be cleanly matched by jsartoolkitNFT.
];

const MARKER_BASE = 'pinball';  // ↔ pinball.{fset, fset3, iset}
const CAMERA_FILE = 'camera_para.dat';
// Number of process() iterations before recording the pose. NFT
// matching detects on the first frame; subsequent frames track the
// pose. The bridge uses the FIRST successful match (KPM detection)
// since that's what the Rust gate exercises.
const PROCESS_ITERATIONS = 10;

function nowIso() {
    return new Date().toISOString();
}

/**
 * Run one fixture through jsartoolkitNFT-Node, return the matched
 * marker ID + transformation matrix (as observed via getNFTMarker).
 */
async function processFixture(fixture) {
    const arControllerNFT = await new jsartoolkitNFT.ARControllerNFT(
        fixture.width, fixture.height, '/' + CAMERA_FILE);
    const ar = await arControllerNFT._initialize();

    const rgbaBuf = await sharp(fixture.name)
        .ensureAlpha()
        .raw()
        .toBuffer();
    const imageData = new Uint8Array(rgbaBuf.buffer);

    // Record the FIRST successful match — the KPM detection result —
    // which is what the Rust integration test compares against.
    let firstMarker = null;
    ar.on('getNFTMarker', (e) => {
        if (firstMarker === null) {
            firstMarker = JSON.parse(JSON.stringify(e.data.marker));
        }
    });

    return new Promise((resolve, reject) => {
        ar.loadNFTMarker(MARKER_BASE, (id) => {
            try {
                ar.trackNFTMarkerId(id);
                const nftData = ar.getNFTData(ar.id, 0);
                const cameraMatrix = ar.getCameraMatrix();
                for (let i = 0; i < PROCESS_ITERATIONS; i++) {
                    ar.process(imageData);
                }
                // Give event listeners a tick to flush.
                setTimeout(() => {
                    resolve({
                        loaded_marker_id: id,
                        nft_data: nftData,
                        camera_matrix: Array.from(cameraMatrix),
                        first_match: firstMarker,
                    });
                }, 50);
            } catch (e) {
                reject(e);
            }
        });
    });
}

async function main() {
    // Ensure the Emscripten NODEFS sees the marker assets at the
    // expected relative paths.
    process.chdir(DATA_DIR);
    console.error(`[bridge] cwd = ${process.cwd()}`);

    const pkgVersion = require(
        '@webarkit/jsartoolkit-nft/package.json'
    ).version;
    console.error(
        `[bridge] using @webarkit/jsartoolkit-nft@${pkgVersion}`
    );

    const results = {};
    for (const fixture of FIXTURES) {
        console.error(`[bridge] processing ${fixture.name}...`);
        try {
            results[fixture.name] = await processFixture(fixture);
        } catch (e) {
            console.error(
                `[bridge] ERROR on ${fixture.name}: ${e.message || e}`
            );
            results[fixture.name] = { error: String(e) };
        }
    }

    const sidecar = {
        schema: 1,
        generated_with:
            `@webarkit/jsartoolkit-nft@${pkgVersion} via ` +
            `tools/jsartoolkitnft-bridge/run.js`,
        generated_at: nowIso(),
        notes:
            'JS-stack reference output for the cross-stack parity ' +
            'gate. Pre-rebuild WebARKitLib#39 status: the npm-published ' +
            'dist still uses the pre-fix C++ matcher, so these numbers ' +
            'reflect that. Regenerate after jsartoolkitNFT republishes ' +
            'post-#39 to update the comparison reference.',
        per_frame: results,
    };

    const outPath = path.join(SCRIPT_DIR, 'expected-js.json');
    fs.writeFileSync(outPath, JSON.stringify(sidecar, null, 2) + '\n');
    console.error(`[bridge] wrote ${outPath}`);
}

main().catch((e) => {
    console.error('[bridge] FATAL:', e);
    process.exit(1);
});
