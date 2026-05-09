// ============================================================================
// app.js — HIL Control Panel Logic + Three.js 3D Visualization
// Cursr-V Antenna Tracker
// ============================================================================
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ============================================================================
//  Serial Connection
// ============================================================================
let port = null, reader = null, writer = null, readableStreamClosed = null;
let isConnected = false;
let partialLine = '';

window.toggleConnection = async function() {
    if (isConnected) { await disconnect(); return; }
    try {
        port = await navigator.serial.requestPort();
        await port.open({ baudRate: 115200 });
        writer = port.writable.getWriter();
        isConnected = true;
        updateConnectionUI(true);
        readLoop();
        logSerial('--- Connected ---', '#34d399');
    } catch (e) { logSerial('Connection failed: ' + e.message, '#f87171'); }
};

async function disconnect() {
    isConnected = false;
    try {
        if (reader) { await reader.cancel(); reader.releaseLock(); }
        if (readableStreamClosed) await readableStreamClosed.catch(()=>{});
        if (writer) { writer.releaseLock(); }
        if (port) await port.close();
    } catch(e) {}
    reader = null; writer = null; port = null;
    updateConnectionUI(false);
    logSerial('--- Disconnected ---', '#f87171');
}

async function readLoop() {
    const decoder = new TextDecoderStream();
    readableStreamClosed = port.readable.pipeTo(decoder.writable);
    reader = decoder.readable.getReader();
    try {
        while (true) {
            const { value, done } = await reader.read();
            if (done) break;
            partialLine += value;
            const lines = partialLine.split('\n');
            partialLine = lines.pop();
            for (const line of lines) {
                const trimmed = line.trim();
                if (!trimmed) continue;
                processIncoming(trimmed);
            }
        }
    } catch (e) { if (isConnected) logSerial('Read error: '+e.message, '#f87171'); }
}

function updateConnectionUI(connected) {
    const row = document.getElementById('statusRow');
    const txt = document.getElementById('statusText');
    const btn = document.getElementById('btnConnect');
    row.className = 'status-row ' + (connected ? 'connected' : 'disconnected');
    txt.textContent = connected ? 'Connected' : 'Disconnected';
    btn.textContent = connected ? 'Disconnect' : 'Connect';
    btn.className = connected ? 'btn btn-red' : 'btn btn-green';
}

async function sendString(s) {
    if (!writer) return;
    const enc = new TextEncoder();
    try { await writer.write(enc.encode(s + '\n')); } catch(e) { logSerial('Write error: '+e.message,'#f87171'); }
}

window.sendCmd = function(obj) { sendString(JSON.stringify(obj)); logSerial('> '+JSON.stringify(obj),'#6c8cff'); };

// ============================================================================
//  Process Incoming JSON
// ============================================================================
const chartData = { azT:[], azC:[], elT:[], elC:[], ts:[] };
const MAX_CHART = 300;

function processIncoming(line) {
    logSerial('< '+line, '#94a3b8');
    try {
        const d = JSON.parse(line);
        if (d.t === 'tel') {
            document.getElementById('telAzT').textContent = d.az_t+'°';
            document.getElementById('telAzC').textContent = d.az_c+'°';
            document.getElementById('telElT').textContent = d.el_t+'°';
            document.getElementById('telElC').textContent = d.el_c+'°';
            document.getElementById('telPid').textContent = d.pid;
            document.getElementById('telSpd').textContent = d.spd;
            document.getElementById('telSweep').textContent = d.sweep||'none';
            document.getElementById('telUp').textContent = (d.up/1000).toFixed(1)+'s';
            // Update 3D tracker orientation
            updateTrackerModel(parseFloat(d.az_c), parseFloat(d.el_c));
            // Chart
            const now = performance.now()/1000;
            chartData.azT.push(parseFloat(d.az_t)); chartData.azC.push(parseFloat(d.az_c));
            chartData.elT.push(parseFloat(d.el_t)); chartData.elC.push(parseFloat(d.el_c));
            chartData.ts.push(now);
            if (chartData.ts.length > MAX_CHART) { chartData.azT.shift(); chartData.azC.shift(); chartData.elT.shift(); chartData.elC.shift(); chartData.ts.shift(); }
            drawChart();
        }
    } catch(e) {}
}

// ============================================================================
//  Serial Log
// ============================================================================
function logSerial(msg, color='#94a3b8') {
    const el = document.getElementById('serialLog');
    const line = document.createElement('div');
    line.style.color = color;
    line.textContent = msg;
    el.appendChild(line);
    if (el.children.length > 200) el.removeChild(el.firstChild);
    el.scrollTop = el.scrollHeight;
}

// ============================================================================
//  UI Handlers
// ============================================================================
window.sendSensorConfig = function() {
    sendCmd({cmd:'set_sensors', enc:document.getElementById('chkEncoder').checked, mag:document.getElementById('chkMag').checked, gps:document.getElementById('chkGPS').checked});
};
window.onElSlider = function(v) {
    document.getElementById('elVal').textContent = parseFloat(v).toFixed(1)+'°';
    sendCmd({cmd:'direct', el:parseFloat(v)});
    targetEl = parseFloat(v);
    update3DTarget(targetAz, targetEl);
};
window.onAzSlider = function(v) {
    document.getElementById('azVal').textContent = parseFloat(v).toFixed(1)+'°';
    sendCmd({cmd:'direct', az:parseFloat(v)});
    targetAz = parseFloat(v);
    update3DTarget(targetAz, targetEl);
};
window.directEl = function(v) { document.getElementById('elSlider').value=v; window.onElSlider(v); };
window.directAz = function(v) { document.getElementById('azSlider').value=v; window.onAzSlider(v); };
window.onPidChange = function() {
    const kp=parseFloat(document.getElementById('kpSlider').value);
    const ki=parseFloat(document.getElementById('kiSlider').value);
    const kd=parseFloat(document.getElementById('kdSlider').value);
    document.getElementById('kpVal').textContent=kp.toFixed(2);
    document.getElementById('kiVal').textContent=ki.toFixed(3);
    document.getElementById('kdVal').textContent=kd.toFixed(2);
    sendCmd({cmd:'set_pid',kp,ki,kd});
};
window.sendBase = function() {
    sendCmd({cmd:'set_base', lat:parseFloat(document.getElementById('baseLat').value), lon:parseFloat(document.getElementById('baseLon').value), alt:parseFloat(document.getElementById('baseAlt').value)});
};
window.injectCoords = function() {
    sendCmd({cmd:'inject', lat:parseFloat(document.getElementById('tgtLat').value), lon:parseFloat(document.getElementById('tgtLon').value), alt:parseFloat(document.getElementById('tgtAlt').value)});
};
window.preset = function(p) {
    const bLat=parseFloat(document.getElementById('baseLat').value);
    const bLon=parseFloat(document.getElementById('baseLon').value);
    const presets={n1k:{lat:bLat+0.009,lon:bLon,alt:100},e1k:{lat:bLat,lon:bLon+0.0105,alt:100},s1k:{lat:bLat-0.009,lon:bLon,alt:100},w1k:{lat:bLat,lon:bLon-0.0105,alt:100},overhead:{lat:bLat,lon:bLon,alt:1000}};
    const t=presets[p]; if(!t)return;
    document.getElementById('tgtLat').value=t.lat.toFixed(4);
    document.getElementById('tgtLon').value=t.lon.toFixed(4);
    document.getElementById('tgtAlt').value=t.alt;
    injectCoords();
};

// ============================================================================
//  Chart (Canvas 2D)
// ============================================================================
function drawChart() {
    const canvas = document.getElementById('chartCanvas');
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * devicePixelRatio;
    canvas.height = rect.height * devicePixelRatio;
    const ctx = canvas.getContext('2d');
    ctx.scale(devicePixelRatio, devicePixelRatio);
    const W = rect.width, H = rect.height;
    ctx.clearRect(0,0,W,H);
    if (chartData.ts.length < 2) return;
    // Grid
    ctx.strokeStyle = 'rgba(100,120,200,0.1)';
    ctx.lineWidth = 1;
    for (let i=0;i<5;i++) { const y=H*i/4; ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke(); }
    // Labels
    ctx.fillStyle='#64748b'; ctx.font='10px Inter';
    ctx.fillText('360°',2,12); ctx.fillText('180°',2,H/2+4); ctx.fillText('0°',2,H-2);
    // Draw lines
    const tMin=chartData.ts[0], tMax=chartData.ts[chartData.ts.length-1];
    const tRange=Math.max(tMax-tMin,1);
    function drawLine(data,color,maxV=360) {
        ctx.beginPath(); ctx.strokeStyle=color; ctx.lineWidth=1.5;
        for (let i=0;i<data.length;i++) {
            const x=(chartData.ts[i]-tMin)/tRange*W;
            const y=H-(data[i]/maxV)*H;
            i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);
        }
        ctx.stroke();
    }
    drawLine(chartData.azT,'rgba(108,140,255,0.5)');
    drawLine(chartData.azC,'#6c8cff');
    drawLine(chartData.elT,'rgba(167,139,250,0.5)',135);
    drawLine(chartData.elC,'#a78bfa',135);
    // Legend
    ctx.font='10px Inter';
    const labels=[{c:'#6c8cff',l:'Az Current'},{c:'rgba(108,140,255,0.5)',l:'Az Target'},{c:'#a78bfa',l:'El Current'},{c:'rgba(167,139,250,0.5)',l:'El Target'}];
    let lx=W-220;
    labels.forEach(lb=>{ ctx.fillStyle=lb.c; ctx.fillRect(lx,4,12,8); ctx.fillStyle='#94a3b8'; ctx.fillText(lb.l,lx+16,12); lx+=55; });
}

// ============================================================================
//  Three.js 3D Scene
// ============================================================================
let scene, camera, renderer, controls;
let trackerGroup, targetSphere, pointingLine, azArc, elArc;
let targetAz = 0, targetEl = 0;
const SPHERE_R = 5; // hemisphere radius for target
let isDragging = false;
const raycaster = new THREE.Raycaster();
const mouse = new THREE.Vector2();
const dragPlane = new THREE.Plane();
const intersection = new THREE.Vector3();

function init3D() {
    const container = document.getElementById('scene3d');
    const W = container.clientWidth, H = container.clientHeight;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x070b15);
    scene.fog = new THREE.FogExp2(0x070b15, 0.015);

    camera = new THREE.PerspectiveCamera(50, W/H, 0.1, 100);
    camera.position.set(6, 5, 8);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(W, H);
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    container.appendChild(renderer.domElement);

    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.maxDistance = 20;
    controls.minDistance = 3;

    // Ambient + directional light
    scene.add(new THREE.AmbientLight(0x6c8cff, 0.3));
    const dLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dLight.position.set(5, 10, 5);
    scene.add(dLight);

    // Ground grid
    const gridHelper = new THREE.GridHelper(12, 24, 0x1e293b, 0x141a2e);
    scene.add(gridHelper);

    // Distance rings
    [2, 4, SPHERE_R].forEach((r, i) => {
        const ringGeo = new THREE.RingGeometry(r-0.02, r+0.02, 64);
        const ringMat = new THREE.MeshBasicMaterial({ color: 0x334155, side: THREE.DoubleSide, transparent: true, opacity: 0.3 });
        const ring = new THREE.Mesh(ringGeo, ringMat);
        ring.rotation.x = -Math.PI/2;
        ring.position.y = 0.01;
        scene.add(ring);
    });

    // Compass labels (N/E/S/W)
    const labelData = [
        { text: 'N', pos: [0, 0.05, -SPHERE_R-0.5], color: 0xf87171 },
        { text: 'E', pos: [SPHERE_R+0.5, 0.05, 0], color: 0x94a3b8 },
        { text: 'S', pos: [0, 0.05, SPHERE_R+0.5], color: 0x94a3b8 },
        { text: 'W', pos: [-SPHERE_R-0.5, 0.05, 0], color: 0x94a3b8 }
    ];
    labelData.forEach(lb => {
        const canvas = document.createElement('canvas');
        canvas.width = 64; canvas.height = 64;
        const ctx = canvas.getContext('2d');
        ctx.font = 'bold 40px Inter'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
        ctx.fillStyle = '#'+lb.color.toString(16).padStart(6,'0');
        ctx.fillText(lb.text, 32, 32);
        const tex = new THREE.CanvasTexture(canvas);
        const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true }));
        sprite.position.set(...lb.pos);
        sprite.scale.set(0.6, 0.6, 1);
        scene.add(sprite);
    });

    // Hemisphere wireframe (semi-transparent)
    const hemiGeo = new THREE.SphereGeometry(SPHERE_R, 32, 16, 0, Math.PI*2, 0, Math.PI/2);
    const hemiMat = new THREE.MeshBasicMaterial({ color: 0x6c8cff, wireframe: true, transparent: true, opacity: 0.05 });
    scene.add(new THREE.Mesh(hemiGeo, hemiMat));

    // Tracker model (cone + base)
    trackerGroup = new THREE.Group();
    const baseCyl = new THREE.Mesh(
        new THREE.CylinderGeometry(0.25, 0.3, 0.15, 16),
        new THREE.MeshPhongMaterial({ color: 0x334155 })
    );
    baseCyl.position.y = 0.075;
    trackerGroup.add(baseCyl);

    const dish = new THREE.Mesh(
        new THREE.ConeGeometry(0.3, 0.5, 16),
        new THREE.MeshPhongMaterial({ color: 0x6c8cff, emissive: 0x1a2a5e })
    );
    dish.position.y = 0.4;
    dish.name = 'dish';
    trackerGroup.add(dish);
    scene.add(trackerGroup);

    // Target sphere (draggable)
    const tgtGeo = new THREE.SphereGeometry(0.2, 16, 16);
    const tgtMat = new THREE.MeshPhongMaterial({ color: 0xf87171, emissive: 0x7f1d1d, transparent: true, opacity: 0.9 });
    targetSphere = new THREE.Mesh(tgtGeo, tgtMat);
    targetSphere.position.set(0, 0, -SPHERE_R);
    scene.add(targetSphere);

    // Glow around target
    const glowGeo = new THREE.SphereGeometry(0.35, 16, 16);
    const glowMat = new THREE.MeshBasicMaterial({ color: 0xf87171, transparent: true, opacity: 0.15 });
    const glow = new THREE.Mesh(glowGeo, glowMat);
    targetSphere.add(glow);

    // Pointing line
    const lineGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0,0,-SPHERE_R)]);
    const lineMat = new THREE.LineBasicMaterial({ color: 0x34d399, linewidth: 2 });
    pointingLine = new THREE.Line(lineGeo, lineMat);
    scene.add(pointingLine);

    // Azimuth arc (on ground plane)
    azArc = createArc(0x6c8cff);
    scene.add(azArc);

    // Elevation arc (vertical)
    elArc = createArc(0xa78bfa);
    scene.add(elArc);

    // Mouse interaction for target dragging
    const domEl = renderer.domElement;
    domEl.addEventListener('pointerdown', onPointerDown);
    domEl.addEventListener('pointermove', onPointerMove);
    domEl.addEventListener('pointerup', onPointerUp);

    // Resize
    window.addEventListener('resize', () => {
        const w = container.clientWidth, h = container.clientHeight;
        camera.aspect = w/h; camera.updateProjectionMatrix();
        renderer.setSize(w, h);
    });

    update3DTarget(0, 0);
    animate();
}

function createArc(color) {
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(129 * 3); // max 128 segments + 1
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    const mat = new THREE.LineBasicMaterial({ color, transparent: true, opacity: 0.6 });
    return new THREE.Line(geo, mat);
}

function updateAzArc(azDeg) {
    const positions = azArc.geometry.attributes.position.array;
    const segs = 64;
    const azRad = azDeg * Math.PI / 180;
    const r = 2;
    for (let i = 0; i <= segs; i++) {
        const a = (i / segs) * azRad;
        // Az 0 = North (-Z), clockwise
        positions[i*3]   = r * Math.sin(a);
        positions[i*3+1] = 0.02;
        positions[i*3+2] = -r * Math.cos(a);
    }
    azArc.geometry.setDrawRange(0, segs + 1);
    azArc.geometry.attributes.position.needsUpdate = true;
}

function updateElArc(azDeg, elDeg) {
    const positions = elArc.geometry.attributes.position.array;
    const segs = 32;
    const azRad = azDeg * Math.PI / 180;
    const r = 2;
    const elRad = elDeg * Math.PI / 180;
    for (let i = 0; i <= segs; i++) {
        const a = (i / segs) * elRad;
        const cosA = Math.cos(a), sinA = Math.sin(a);
        // Rotate in the vertical plane oriented at azimuth
        positions[i*3]   = r * cosA * Math.sin(azRad);
        positions[i*3+1] = r * sinA;
        positions[i*3+2] = -r * cosA * Math.cos(azRad);
    }
    elArc.geometry.setDrawRange(0, segs + 1);
    elArc.geometry.attributes.position.needsUpdate = true;
}

function update3DTarget(az, el) {
    targetAz = az; targetEl = el;
    const azRad = az * Math.PI / 180;
    const elRad = el * Math.PI / 180;
    // Position on hemisphere
    const x = SPHERE_R * Math.cos(elRad) * Math.sin(azRad);
    const y = SPHERE_R * Math.sin(elRad);
    const z = -SPHERE_R * Math.cos(elRad) * Math.cos(azRad);
    targetSphere.position.set(x, y, z);
    // Update pointing line
    const pts = pointingLine.geometry.attributes.position.array;
    pts[3] = x; pts[4] = y; pts[5] = z;
    pointingLine.geometry.attributes.position.needsUpdate = true;
    // Update arcs
    updateAzArc(az);
    updateElArc(az, el);
    // Update info display
    document.getElementById('sceneAz').textContent = az.toFixed(1) + '°';
    document.getElementById('sceneEl').textContent = el.toFixed(1) + '°';
}

function updateTrackerModel(az, el) {
    const dish = trackerGroup.getObjectByName('dish');
    if (!dish) return;
    // Reset rotation then apply
    dish.rotation.set(0, 0, 0);
    dish.rotation.y = -az * Math.PI / 180;
    dish.rotation.x = -(90 - el) * Math.PI / 180;
}

// ============================================================================
//  Target Dragging
// ============================================================================
let dragTarget = null;

function onPointerDown(e) {
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouse, camera);
    const hits = raycaster.intersectObject(targetSphere, true);
    if (hits.length > 0) {
        isDragging = true;
        dragTarget = targetSphere;
        controls.enabled = false;
        renderer.domElement.style.cursor = 'grabbing';
    }
}

function onPointerMove(e) {
    if (!isDragging) {
        // Hover check
        const rect = renderer.domElement.getBoundingClientRect();
        mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
        mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
        raycaster.setFromCamera(mouse, camera);
        const hits = raycaster.intersectObject(targetSphere, true);
        renderer.domElement.style.cursor = hits.length > 0 ? 'grab' : 'default';
        return;
    }
    const rect = renderer.domElement.getBoundingClientRect();
    mouse.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(mouse, camera);

    // Intersect with hemisphere: cast ray to sphere centered at origin
    const origin = raycaster.ray.origin.clone();
    const dir = raycaster.ray.direction.clone();
    // Solve |origin + t*dir|^2 = R^2
    const a = dir.dot(dir);
    const b = 2 * origin.dot(dir);
    const c = origin.dot(origin) - SPHERE_R * SPHERE_R;
    const disc = b*b - 4*a*c;
    if (disc < 0) return;
    const t1 = (-b - Math.sqrt(disc)) / (2*a);
    const t2 = (-b + Math.sqrt(disc)) / (2*a);
    let t = t1 > 0.01 ? t1 : t2;
    if (t < 0.01) return;
    const hit = origin.clone().add(dir.clone().multiplyScalar(t));
    // Constrain to upper hemisphere
    if (hit.y < 0) hit.y = 0;
    hit.normalize().multiplyScalar(SPHERE_R);

    // Compute az/el from position
    const az = Math.atan2(hit.x, -hit.z) * 180 / Math.PI;
    const el = Math.asin(Math.max(0, hit.y / SPHERE_R)) * 180 / Math.PI;
    const azNorm = az < 0 ? az + 360 : az;
    const elClamped = Math.min(135, Math.max(0, el));

    update3DTarget(azNorm, elClamped);
    // Update sliders
    document.getElementById('azSlider').value = azNorm;
    document.getElementById('azVal').textContent = azNorm.toFixed(1) + '°';
    document.getElementById('elSlider').value = elClamped;
    document.getElementById('elVal').textContent = elClamped.toFixed(1) + '°';
    // Send to ESP32 (throttled)
    throttledSendDirect(azNorm, elClamped);
}

function onPointerUp() {
    if (isDragging) {
        isDragging = false;
        controls.enabled = true;
        renderer.domElement.style.cursor = 'default';
        // Final send
        sendCmd({ cmd: 'direct', az: Math.round(targetAz*10)/10, el: Math.round(targetEl*10)/10 });
    }
}

// Throttle drag sends to ~30fps
let lastDragSend = 0;
function throttledSendDirect(az, el) {
    const now = performance.now();
    if (now - lastDragSend < 33) return;
    lastDragSend = now;
    sendCmd({ cmd: 'direct', az: Math.round(az*10)/10, el: Math.round(el*10)/10 });
}

// ============================================================================
//  Animation Loop
// ============================================================================
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    // Pulse target glow
    const t = performance.now() * 0.003;
    if (targetSphere.children[0]) {
        targetSphere.children[0].material.opacity = 0.1 + 0.08 * Math.sin(t);
    }
    // Color pointing line based on error
    if (pointingLine) {
        const errThreshold = 5;
        // Simple heuristic: if we have telemetry, color based on az error
        pointingLine.material.color.setHex(isDragging ? 0xfbbf24 : 0x34d399);
    }
    renderer.render(scene, camera);
}

// ============================================================================
//  Init
// ============================================================================
init3D();
