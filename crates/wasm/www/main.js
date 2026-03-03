import init, { WasmARHandle, init_panic_hook } from '../pkg/wasm.js';

const statusEl = document.getElementById('status');
const btnInit = document.getElementById('btn-init');
const btnDetect = document.getElementById('btn-detect');
const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');
const resultsEl = document.getElementById('results');

let arHandle = null;
let testImage = new Image();

async function run() {
    try {
        // Initialize WASM module
        await init();
        init_panic_hook();

        statusEl.innerText = 'Status: WASM Loaded. Downloading assets...';

        // Load test image
        testImage.src = 'assets/img.jpg';
        await new Promise(resolve => testImage.onload = resolve);

        // Setup canvas
        canvas.width = testImage.width;
        canvas.height = testImage.height;
        ctx.drawImage(testImage, 0, 0);

        statusEl.innerText = 'Status: Ready to Initialize AR';
        btnInit.disabled = false;

    } catch (err) {
        statusEl.innerText = `Status Error: ${err}`;
        console.error(err);
    }
}

btnInit.onclick = async () => {
    try {
        statusEl.innerText = 'Status: Initializing AR Handle...';

        // Fetch Camera Parameters
        const paramRes = await fetch('assets/camera_para.dat');
        const paramData = new Uint8Array(await paramRes.arrayBuffer());

        // Initialize Handle
        arHandle = new WasmARHandle(paramData);

        // Enable AutoOtsu thresholding (mode 2) and Debug mode
        arHandle.set_threshold_mode(2);
        arHandle.set_debug_mode(true);

        // Load Pattern (Hiro)
        const pattRes = await fetch('assets/patt.hiro');
        const pattContent = await pattRes.text();
        const pattId = arHandle.load_pattern(pattContent);

        statusEl.innerText = `Status: AR Initialized. Pattern ID Hiro: ${pattId}`;
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
    const endTime = performance.now();

    resultsEl.innerText = `Detection took ${(endTime - startTime).toFixed(2)}ms\n`;
    resultsEl.innerText += `Markers found: ${markers.length}\n\n`;

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
