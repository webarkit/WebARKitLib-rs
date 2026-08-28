/*
 *  nft.worker.js
 *  WebARKitLib-rs WebWorker
 *
 *  Dedicated WebWorker for background WASM NFT (KPM detection & AR2 tracking).
 *  Offloads CPU-intensive feature extraction and template matching from the main UI thread.
 */

const VALID_VARIANTS = ['dist-std', 'dist-simd'];

let wasmModule = null;
let kpmHandle = null;
let nftHandle = null;

let isInitialized = false;
let isProcessing = false;
let pipelineState = 'SEARCHING';
let markerWidth = 0;
let markerHeight = 0;
let markerDpi = 0;

self.onmessage = async function (e) {
    const msg = e.data;

    switch (msg.type) {
        case 'init': {
            try {
                const { variant, paramData, isetData, fsetData, fset3Data, width, height } = msg;

                // Only the two builds produced by `npm run build:wasm` are importable.
                // `variant` arrives over postMessage, so never interpolate it unchecked.
                if (!VALID_VARIANTS.includes(variant)) {
                    throw new Error(`Unknown WASM variant '${variant}' (expected one of ${VALID_VARIANTS.join(', ')})`);
                }

                // Dynamically import WASM module in WebWorker
                const modulePath = `../pkg/${variant}/webarkitlib_wasm.js`;
                const module = await import(modulePath);
                wasmModule = module;

                await module.default();
                module.init_wasm();

                // Instantiate WasmKpmHandle and WasmNFTHandle inside worker thread
                kpmHandle = new wasmModule.WasmKpmHandle(paramData, width, height);
                nftHandle = new wasmModule.WasmNFTHandle(paramData, width, height);

                kpmHandle.load_ref_data(fset3Data);
                nftHandle.load_nft_marker(isetData, fsetData);

                markerWidth = nftHandle.get_marker_width();
                markerHeight = nftHandle.get_marker_height();
                markerDpi = nftHandle.get_marker_dpi();

                const intrinsics = Array.from(nftHandle.get_camera_intrinsics());
                const projMat = Array.from(nftHandle.get_projection_matrix(0.1, 1000.0));

                isInitialized = true;
                pipelineState = 'SEARCHING';
                isProcessing = false;

                self.postMessage({
                    type: 'loaded',
                    width,
                    height,
                    markerWidth,
                    markerHeight,
                    markerDpi,
                    intrinsics,
                    projMat
                });
            } catch (err) {
                self.postMessage({ type: 'error', error: err.toString() });
            }
            break;
        }

        case 'process': {
            // The main thread clears its `workerBusy` flag only when a message
            // comes back, so a silent return here would stall the pipeline for
            // good. Always acknowledge, even when the frame is dropped.
            if (!isInitialized || isProcessing) {
                self.postMessage({
                    type: 'skipped',
                    reason: isInitialized ? 'busy' : 'notInitialized',
                    pipelineState
                });
                return;
            }
            isProcessing = true;

            const { rgba, width, height } = msg;
            const t0 = performance.now();

            try {
                if (pipelineState === 'SEARCHING') {
                    // Execute KPM FREAK feature detection
                    const det = kpmHandle.detect(rgba);
                    const elapsed = performance.now() - t0;

                    if (det && det.pose) {
                        const poseArray = new Float32Array(det.pose);
                        nftHandle.set_initial_pose(poseArray);
                        pipelineState = 'TRACKING';

                        self.postMessage({
                            type: 'found',
                            pose: Array.from(det.pose),
                            page: det.page,
                            error: det.error,
                            contNum: 1,
                            pipelineState: 'TRACKING',
                            elapsed: elapsed,
                            stage: 'KPM'
                        });
                    } else {
                        self.postMessage({
                            type: 'notFound',
                            pipelineState: 'SEARCHING',
                            elapsed: elapsed,
                            stage: 'KPM'
                        });
                    }
                } else if (pipelineState === 'TRACKING') {
                    // Execute per-frame AR2 template tracking
                    const res = nftHandle.track(rgba, width, height);
                    const elapsed = performance.now() - t0;

                    if (res && res.found) {
                        self.postMessage({
                            type: 'found',
                            pose: Array.from(res.matrix),
                            error: res.error,
                            contNum: res.cont_num,
                            pipelineState: 'TRACKING',
                            elapsed: elapsed,
                            stage: 'AR2'
                        });
                    } else {
                        // Tracking lost: reset AR2 state and revert to SEARCHING
                        nftHandle.reset_tracking();
                        pipelineState = 'SEARCHING';

                        self.postMessage({
                            type: 'lost',
                            pipelineState: 'SEARCHING',
                            elapsed: elapsed,
                            stage: 'AR2'
                        });
                    }
                }
            } catch (err) {
                self.postMessage({ type: 'error', error: err.toString() });
            } finally {
                isProcessing = false;
            }
            break;
        }

        case 'reset': {
            if (nftHandle) {
                nftHandle.reset_tracking();
            }
            pipelineState = 'SEARCHING';
            isProcessing = false;
            self.postMessage({ type: 'resetDone' });
            break;
        }

        case 'forceDetect': {
            if (nftHandle) {
                nftHandle.reset_tracking();
            }
            pipelineState = 'SEARCHING';
            break;
        }
    }
};
