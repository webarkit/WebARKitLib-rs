// Engine state container
let currentEngine = {
    WasmARHandle: null,
    version: null
};

const statusEl = document.getElementById('status');
const btnInit = document.getElementById('btn-init');
const btnDetect = document.getElementById('btn-detect');
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const wasmVersionSelect = document.getElementById('wasm-version');
const resultsEl = document.getElementById('results');
const refreshNotice = document.getElementById('refresh-notice');

let arHandle = null;
let currentLoadedVersion = null;
let testImage = new Image();

async function run() {
    try {
        statusEl.innerText = 'Status: Selecting WASM Engine...';
        
        // Setup initial UI state
        btnInit.disabled = true;
        btnDetect.disabled = true;

        // Load test image early
        testImage.src = 'assets/img.jpg';
        await new Promise(resolve => testImage.onload = resolve);

        // Setup canvas
        canvas.width = testImage.width;
        canvas.height = testImage.height;
        ctx.drawImage(testImage, 0, 0);

        statusEl.innerText = 'Status: Assets loaded. Ready to load WASM.';
        btnInit.disabled = false;

    } catch (err) {
        statusEl.innerText = `Status Error: ${err}`;
        console.error(err);
    }
}

wasmVersionSelect.onchange = () => {
    if (arHandle) {
        console.log("Freeing previous AR Handle memory...");
        arHandle.free();
        arHandle = null;
    }
    btnInit.disabled = false;
    btnDetect.disabled = true;
    refreshNotice.style.display = 'block';
    statusEl.innerText = 'Status: Engine changed. Click Initialize to load.';
    console.log(`Engine changed to ${wasmVersionSelect.value}. Ready for re-initialization.`);
};

async function loadWasm() {
    const version = wasmVersionSelect.value;
    
    if (currentEngine.version === version) {
        console.log(`[WebARKit] Engine ${version} is already the active one.`);
        return;
    }

    statusEl.innerText = `Status: Loading ${version} WASM Engine...`;
    console.log(`[WebARKit] Switching to Engine: ${version === 'dist-simd' ? 'SIMD (High Performance)' : 'Standard (Portable)'}`);
    
    // Add cache-breaker to ensure fresh module evaluation on switch
    const cacheBreaker = `?t=${Date.now()}`;
    const modulePath = `../pkg/${version}/webarkitlib_wasm.js${cacheBreaker}`;
    
    try {
        const module = await import(modulePath);
        const init = module.default;
        
        // Initialize the wasm instance
        await init();
        module.init_wasm();
        
        // Store the class constructor and other exports
        currentEngine.WasmARHandle = module.WasmARHandle;
        currentEngine.version = version;

        try {
            module.init_panic_hook();
        } catch (e) {
            // Already initialized or not available
        }
        
        console.log(`[WebARKit] Engine ${version} initialized. Memory isolated.`);
    } catch (e) {
        console.error(`[WebARKit] CRITICAL: Failed to load WASM module from ${modulePath}:`, e);
        statusEl.innerText = `Status Error: Failed to load ${version}`;
        throw e;
    }
}

btnInit.onclick = async () => {
    try {
        // Load WASM if not already loaded or if version changed
        await loadWasm();

        statusEl.innerText = 'Status: Initializing AR Handle...';

        // Fetch Camera Parameters
        const paramRes = await fetch('assets/camera_para.dat');
        const paramData = new Uint8Array(await paramRes.arrayBuffer());

        // Initialize Handle
        if (arHandle) {
            console.log("[WebARKit] Freeing existing AR Handle...");
            arHandle.free();
        }
        
        arHandle = new currentEngine.WasmARHandle(paramData);
        console.log(`[WebARKit] New WasmARHandle created using ${currentEngine.version} engine constructor.`);

        // Enable AutoOtsu thresholding (mode 2) and Debug mode
        arHandle.set_threshold_mode(2);
        arHandle.set_debug_mode(true);

        // Load Pattern (Hiro)
        console.log("[WebARKit] Loading Hiro pattern...");
        const pattRes = await fetch('assets/patt.hiro');
        const pattContent = await pattRes.text();
        const pattId = arHandle.load_pattern(pattContent);
        console.log(`[WebARKit] Hiro pattern loaded. Assigned ID: ${pattId}`);

        statusEl.innerText = `Status: AR Initialized. Engine: ${currentEngine.version === 'dist-simd' ? 'SIMD' : 'Standard'}, Pattern ID: ${pattId}`;
        btnDetect.disabled = false;
        btnInit.disabled = true;
    } catch (err) {
        statusEl.innerText = `Status Error: ${err}`;
        console.error(err);
    }
};

btnDetect.onclick = () => {
    if (!arHandle) return;

    // Get image data from canvas (RGBA)
    const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
    const rgba = new Uint8Array(imageData.data.buffer);

    const startTime = performance.now();
    const markers = arHandle.detect_markers(rgba, canvas.width, canvas.height);
    const threshold = arHandle.get_threshold();
    const endTime = performance.now();

    console.log(`[WebARKit] Detection complete. Engine: ${currentEngine.version}, Markers: ${markers.length}, Threshold: ${threshold}, Time: ${(endTime - startTime).toFixed(2)}ms`);

    resultsEl.innerText = `Detection took ${(endTime - startTime).toFixed(2)}ms\n`;
    resultsEl.innerText += `Markers found: ${markers.length} | Threshold: ${threshold}\n\n`;

    // Draw detection results
    ctx.drawImage(testImage, 0, 0);
    ctx.strokeStyle = '#00ff00';
    ctx.lineWidth = 4;
    ctx.font = '20px Arial';
    ctx.fillStyle = '#00ff00';

    markers.forEach((m, index) => {
        resultsEl.innerText += `Marker ID: ${m.id} (Hiro is usually 0)\n`;
        resultsEl.innerText += `Confidence: ${m.cf.toFixed(4)}\n`;
        resultsEl.innerText += `Position: [${m.pos[0].toFixed(1)}, ${m.pos[1].toFixed(1)}]\n`;

        // Get 3x4 Transformation Matrix (Assume marker width 80.0 units)
        const pose = arHandle.get_trans_mat(index, 80.0);
        const mat = pose.matrix;
        resultsEl.innerText += `ICP Error: ${pose.icp_error.toFixed(4)}\n`;
        resultsEl.innerText += `Matrix 3x4:\n${formatMatrix(mat)}\n\n`;

        // Draw crosshair at pos
        ctx.beginPath();
        ctx.moveTo(m.pos[0] - 20, m.pos[1]);
        ctx.lineTo(m.pos[0] + 20, m.pos[1]);
        ctx.moveTo(m.pos[0], m.pos[1] - 20);
        ctx.lineTo(m.pos[0], m.pos[1] + 20);
        ctx.stroke();
        ctx.fillText(`ID: ${m.id}`, m.pos[0] + 25, m.pos[1] - 10);
    });
};

function formatMatrix(mat) {
    let s = '';
    for (let i = 0; i < 3; i++) {
        s += `[ ${mat[i * 4].toFixed(4)}, ${mat[i * 4 + 1].toFixed(4)}, ${mat[i * 4 + 2].toFixed(4)}, ${mat[i * 4 + 3].toFixed(4)} ]\n`;
    }
    return s;
}

run();
