import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import config as C
from drone.flight import (
    set_selected_point,
    clear_selected_point,
    request_launch,
    request_land,
    request_kill,
)

_STATE_LOCK = threading.Lock()
_LATEST_PACKET = {
    "t": 0.0,
    "pos": [0.0, -4.5, 2.2],
    "vel": [4.5, 0.0, 0.0],
    "att": [0.0, 0.0, 0.0],
    "target": [0.0, 0.0, 2.2],
    "loiter_center": [0.0, 0.0, 2.2],
    "loiter_radius": 4.5,
    "airspeed": 4.5,
    "alpha_deg": 2.5,
    "beta_deg": 0.0,
    "surfaces": [0.5, 0.0, 0.0, 0.0],
    "wind": [0.0, 0.0, 0.0],
    "phase": "READY",
    "battery": 1.0,
    "selected_mode": "auto",
}


def update_web_telemetry(pkt):
    from drone import flight as F
    with _STATE_LOCK:
        _LATEST_PACKET.update(pkt)
        _LATEST_PACKET["selected_mode"] = (
            F._INTERACTIVE_MODE if F._INTERACTIVE_POINT is not None else "auto"
        )


HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0" />
<title>UAV Control</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  :root {
    --bg: #0c0c0b;
    --panel: #131311;
    --border: #2a2824;
    --border-strong: #45413a;
    --white: #f4f1ea;
    --beige: #d8ccb8;
    --beige-dim: #8c8273;
    --muted: #5e584f;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--white);
    font-family: ui-monospace, 'SFMono-Regular', Menlo, Consolas, 'Liberation Mono', monospace;
    font-size: 12px;
    line-height: 1.4;
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }
  main {
    flex: 1;
    display: grid;
    grid-template-columns: 1fr 340px;
    min-height: 0;
  }
  @media (max-width: 900px) {
    body { height: auto; overflow: auto; }
    main { grid-template-columns: 1fr; }
    .map-pane { height: 500px; }
  }
  .map-pane {
    position: relative;
    background: var(--bg);
    border-right: 1px solid var(--border);
    min-height: 0;
  }
  #map {
    width: 100%;
    height: 100%;
    background: #0c0c0b;
    cursor: crosshair;
  }
  #map.dark-tiles .leaflet-tile-pane {
    filter: invert(100%) hue-rotate(180deg) grayscale(65%) brightness(88%) contrast(95%);
  }
  .leaflet-control-attribution {
    background: rgba(12, 12, 11, 0.75) !important;
    color: var(--beige-dim) !important;
    font-size: 9px !important;
  }
  .leaflet-control-attribution a { color: var(--beige) !important; }
  .leaflet-bar a {
    background-color: var(--panel) !important;
    color: var(--beige) !important;
    border-color: var(--border-strong) !important;
  }
  .leaflet-bar a:hover {
    background-color: var(--bg) !important;
    color: var(--white) !important;
  }
  .sidebar {
    background: var(--panel);
    display: flex;
    flex-direction: column;
    overflow-y: auto;
  }
  .section {
    border-bottom: 1px solid var(--border);
    padding: 12px;
  }
  .btn-row {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 6px;
    margin-bottom: 6px;
  }
  .btn-grid-3 {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 6px;
    margin-bottom: 6px;
  }
  .btn-grid-4 {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 4px;
  }
  button {
    background: var(--bg);
    color: var(--beige);
    border: 1px solid var(--border-strong);
    padding: 6px 8px;
    font: inherit;
    font-size: 11px;
    cursor: pointer;
    text-transform: uppercase;
    letter-spacing: 0.04em;
    transition: background 0.1s, color 0.1s, border-color 0.1s;
  }
  button:hover {
    border-color: var(--beige);
    color: var(--white);
  }
  button.active {
    background: var(--beige);
    color: var(--bg);
    border-color: var(--beige);
    font-weight: 600;
  }
  button.launch-btn {
    border-color: var(--white);
    color: var(--white);
    font-weight: 600;
  }
  button.launch-btn:hover {
    background: var(--white);
    color: var(--bg);
  }
  .input-grid {
    display: grid;
    grid-template-columns: repeat(4, 1fr);
    gap: 6px;
    margin-bottom: 6px;
  }
  .field {
    display: flex;
    flex-direction: column;
    gap: 3px;
  }
  .field label {
    font-size: 10px;
    color: var(--beige-dim);
  }
  .field input {
    width: 100%;
    background: var(--bg);
    border: 1px solid var(--border);
    color: var(--white);
    padding: 4px 6px;
    font: inherit;
    font-size: 11px;
    font-variant-numeric: tabular-nums;
  }
  .field input:focus {
    outline: none;
    border-color: var(--beige);
  }
  table.kv {
    width: 100%;
    border-collapse: collapse;
    font-variant-numeric: tabular-nums;
  }
  table.kv td {
    padding: 4px 0;
    border-bottom: 1px solid #1c1b18;
  }
  table.kv tr:last-child td {
    border-bottom: none;
  }
  table.kv td.k {
    color: var(--beige-dim);
    font-size: 11px;
  }
  table.kv td.v {
    text-align: right;
    color: var(--white);
    font-weight: 500;
  }
  .att-wrap {
    display: grid;
    grid-template-columns: 96px 1fr;
    gap: 12px;
    align-items: center;
  }
  #pfdCanvas {
    width: 96px;
    height: 96px;
    border: 1px solid var(--border-strong);
    background: var(--bg);
    display: block;
  }
  .bars {
    display: flex;
    flex-direction: column;
    gap: 6px;
  }
  .bar-row {
    display: grid;
    grid-template-columns: 32px 1fr 40px;
    align-items: center;
    gap: 8px;
    font-size: 11px;
    font-variant-numeric: tabular-nums;
  }
  .bar-row span.lbl {
    color: var(--beige-dim);
  }
  .bar-track {
    height: 6px;
    background: var(--bg);
    border: 1px solid var(--border);
    position: relative;
  }
  .bar-fill {
    position: absolute;
    top: 0;
    bottom: 0;
    background: var(--beige);
  }
  .bar-val {
    text-align: right;
    color: var(--white);
  }
</style>
</head>
<body>

<main>
  <section class="map-pane">
    <div id="map"></div>
  </section>

  <aside class="sidebar">
    <div class="section">
      <div class="btn-grid-3">
        <button class="launch-btn" onclick="startFlight()">START FLIGHT</button>
        <button onclick="landFlight()">LAND NOW</button>
        <button onclick="killFlight()">KILL</button>
      </div>

      <div class="btn-row">
        <button id="btnLoiter" class="active" onclick="setMode('loiter')">LOITER</button>
        <button id="btnFlyTo" onclick="setMode('fly_to')">FLY TO</button>
      </div>

      <div class="input-grid">
        <div class="field">
          <label>X (m)</label>
          <input id="inX" type="number" step="0.5" value="0.0" />
        </div>
        <div class="field">
          <label>Y (m)</label>
          <input id="inY" type="number" step="0.5" value="0.0" />
        </div>
        <div class="field">
          <label>ALT (m)</label>
          <input id="inZ" type="number" step="0.2" min="1.0" max="50.0" value="2.2" onchange="applyInputs()" />
        </div>
        <div class="field">
          <label>RAD (m)</label>
          <input id="inR" type="number" step="0.5" min="2.0" max="50.0" value="4.5" onchange="applyInputs()" />
        </div>
      </div>

      <div class="btn-grid-3">
        <button onclick="applyInputs()">SEND</button>
        <button onclick="setPoint(0.0, 0.0)">HOME (0,0)</button>
        <button onclick="clearTrail()">CLEAR TRAIL</button>
      </div>

      <div class="btn-grid-4" style="margin-bottom:6px">
        <button id="btnDark" class="active" onclick="setMapLayer('dark')">DARK</button>
        <button id="btnSat" onclick="setMapLayer('sat')">SAT</button>
        <button id="btnOsm" onclick="setMapLayer('osm')">OSM</button>
        <button onclick="locateUser()" title="High-Accuracy GPS">MY GPS</button>
      </div>

      <div style="display:grid;grid-template-columns:1fr auto auto;gap:4px">
        <input id="inSearch" type="text" placeholder="Search place or lat,lon..." onkeydown="if(event.key==='Enter')searchPlace()" style="background:var(--bg);border:1px solid var(--border);color:var(--white);padding:4px 6px;font:inherit;font-size:11px;width:100%" />
        <button onclick="searchPlace()">GO</button>
        <button id="btnSetHome" onclick="toggleSetHome()" title="Click map to pin exact Home (0,0)">SET HOME</button>
      </div>
    </div>

    <div class="section">
      <table class="kv">
        <tr><td class="k">STATUS</td><td class="v" id="tPhase">READY</td></tr>
        <tr><td class="k">BATTERY</td><td class="v" id="tBatt">100%</td></tr>
        <tr><td class="k">GPS (LAT, LON)</td><td class="v" id="tGps">0.00000, 0.00000</td></tr>
        <tr><td class="k">AIRSPEED</td><td class="v" id="tAirspeed">0.00 m/s</td></tr>
        <tr><td class="k">ALTITUDE (Z)</td><td class="v" id="tAlt">0.00 m</td></tr>
        <tr><td class="k">LOCAL (X, Y)</td><td class="v" id="tPos">+0.00, +0.00 m</td></tr>
        <tr><td class="k">DIST TO TARGET</td><td class="v" id="tDist">0.00 m</td></tr>
        <tr><td class="k">ROLL / BANK</td><td class="v" id="tRoll">+0.0 deg</td></tr>
        <tr><td class="k">PITCH / NOSE</td><td class="v" id="tPitch">+0.0 deg</td></tr>
        <tr><td class="k">HEADING (YAW)</td><td class="v" id="tYaw">000 deg</td></tr>
        <tr><td class="k">AOA / SIDESLIP</td><td class="v" id="tAero">+0.0 / +0.0 deg</td></tr>
      </table>
    </div>

    <div class="section" style="border-bottom:none">
      <div class="att-wrap">
        <canvas id="pfdCanvas" width="192" height="192"></canvas>
        <div class="bars">
          <div class="bar-row">
            <span class="lbl">THR</span>
            <div class="bar-track"><div id="bThr" class="bar-fill" style="left:0;width:50%;background:var(--white)"></div></div>
            <span class="bar-val" id="vThr">50%</span>
          </div>
          <div class="bar-row">
            <span class="lbl">AIL</span>
            <div class="bar-track"><div id="bAil" class="bar-fill" style="left:50%;width:0%"></div></div>
            <span class="bar-val" id="vAil">+0.00</span>
          </div>
          <div class="bar-row">
            <span class="lbl">ELV</span>
            <div class="bar-track"><div id="bElv" class="bar-fill" style="left:50%;width:0%"></div></div>
            <span class="bar-val" id="vElv">+0.00</span>
          </div>
          <div class="bar-row">
            <span class="lbl">RUD</span>
            <div class="bar-track"><div id="bRud" class="bar-fill" style="left:50%;width:0%"></div></div>
            <span class="bar-val" id="vRud">+0.00</span>
          </div>
        </div>
      </div>
    </div>
  </aside>
</main>

<script>
  let mode = 'loiter';
  let state = null;
  let lastRun = null;
  let pickingHome = false;
  let currentMapTheme = 'dark';

  let homeLat = parseFloat(localStorage.getItem('uav_home_lat')) || 52.0116;
  let homeLon = parseFloat(localStorage.getItem('uav_home_lon')) || 4.3571;
  const METERS_PER_DEG_LAT = 111320.0;

  function setHomeOrigin(lat, lon, recenter = false) {
    homeLat = lat;
    homeLon = lon;
    localStorage.setItem('uav_home_lat', String(homeLat));
    localStorage.setItem('uav_home_lon', String(homeLon));
    clearTrail();
    if (recenter) map.setView([homeLat, homeLon], 19);
  }

  function xyToLatLon(xEast, yNorth) {
    const dLat = yNorth / METERS_PER_DEG_LAT;
    const dLon = xEast / (METERS_PER_DEG_LAT * Math.cos(homeLat * Math.PI / 180.0));
    return [homeLat + dLat, homeLon + dLon];
  }

  function latLonToXY(lat, lon) {
    const yNorth = (lat - homeLat) * METERS_PER_DEG_LAT;
    const xEast = (lon - homeLon) * (METERS_PER_DEG_LAT * Math.cos(homeLat * Math.PI / 180.0));
    return [xEast, yNorth];
  }

  const mapEl = document.getElementById('map');
  mapEl.classList.add('dark-tiles');

  const map = L.map('map', {
    center: [homeLat, homeLon],
    zoom: 19,
    maxZoom: 22,
    zoomControl: true
  });

  const layers = {
    dark: L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 22,
      maxNativeZoom: 19,
      attribution: '&copy; OpenStreetMap'
    }),
    sat: L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
      maxZoom: 22,
      maxNativeZoom: 19,
      attribution: '&copy; Esri Satellite'
    }),
    osm: L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 22,
      maxNativeZoom: 19,
      attribution: '&copy; OpenStreetMap'
    })
  };
  let activeLayer = layers.dark.addTo(map);

  const loiterCircle = L.circle([homeLat, homeLon], {
    radius: 4.5,
    color: '#d8ccb8',
    weight: 2.0,
    fillColor: '#d8ccb8',
    fillOpacity: 0.05,
    dashArray: '5, 5'
  }).addTo(map);

  const centerMarker = L.circleMarker([homeLat, homeLon], {
    radius: 4.5,
    color: '#f4f1ea',
    weight: 1.5,
    fillColor: '#d8ccb8',
    fillOpacity: 1.0
  }).addTo(map);

  const carrotLine = L.polyline([], {
    color: '#8c8273',
    weight: 1.4,
    dashArray: '3, 3'
  }).addTo(map);

  const carrotMarker = L.circleMarker([homeLat, homeLon], {
    radius: 3.5,
    color: '#d8ccb8',
    weight: 1.2,
    fillColor: '#d8ccb8',
    fillOpacity: 1.0
  }).addTo(map);

  const trailLine = L.polyline([], {
    color: '#d8ccb8',
    weight: 2.2,
    opacity: 0.9
  }).addTo(map);

  const uavDivIcon = L.divIcon({
    className: '',
    iconSize: [44, 44],
    iconAnchor: [22, 22],
    html: `<svg id="uavSvg" width="44" height="44" viewBox="-22 -22 44 44" style="overflow:visible">
      <g id="uavRot">
        <polygon id="uavWing" points="3,-20 -3,-20 -5,0 -3,20 3,20 4,0" fill="#f4f1ea" stroke="#0c0c0b" stroke-width="1.2"/>
        <polygon id="uavTail" points="-10,0 -15,-8 -17,-8 -14,0 -17,8 -15,8" fill="#d8ccb8" stroke="#0c0c0b" stroke-width="1.0"/>
        <rect id="uavFuse" x="-15" y="-2.2" width="28" height="4.4" rx="2" fill="#f4f1ea" stroke="#0c0c0b" stroke-width="1.0"/>
        <line id="uavLead" x1="13" y1="0" x2="22" y2="0" stroke="#f4f1ea" stroke-width="1.5"/>
      </g>
    </svg>`
  });
  const uavMarker = L.marker([homeLat, homeLon], { icon: uavDivIcon, interactive: false }).addTo(map);

  function applyOverlayColors(name) {
    currentMapTheme = name;
    const fg = (name === 'osm') ? '#0c0c0b' : (name === 'sat' ? '#ffffff' : '#d8ccb8');
    const stroke = (name === 'osm') ? '#ffffff' : '#0c0c0b';

    loiterCircle.setStyle({ color: fg, fillColor: fg });
    centerMarker.setStyle({ color: fg, fillColor: fg });
    carrotLine.setStyle({ color: fg });
    carrotMarker.setStyle({ color: fg, fillColor: fg });
    trailLine.setStyle({ color: fg });

    const wing = document.getElementById('uavWing');
    const tail = document.getElementById('uavTail');
    const fuse = document.getElementById('uavFuse');
    const lead = document.getElementById('uavLead');
    if (wing && tail && fuse && lead) {
      wing.setAttribute('fill', fg);
      wing.setAttribute('stroke', stroke);
      tail.setAttribute('fill', fg);
      tail.setAttribute('stroke', stroke);
      fuse.setAttribute('fill', fg);
      fuse.setAttribute('stroke', stroke);
      lead.setAttribute('stroke', fg);
    }
  }

  function setMapLayer(name) {
    map.removeLayer(activeLayer);
    activeLayer = layers[name];
    activeLayer.addTo(map);
    if (name === 'dark') mapEl.classList.add('dark-tiles');
    else mapEl.classList.remove('dark-tiles');
    document.getElementById('btnDark').className = (name === 'dark') ? 'active' : '';
    document.getElementById('btnSat').className = (name === 'sat') ? 'active' : '';
    document.getElementById('btnOsm').className = (name === 'osm') ? 'active' : '';
    applyOverlayColors(name);
  }

  map.on('click', (e) => {
    if (pickingHome) {
      setHomeOrigin(e.latlng.lat, e.latlng.lng, false);
      pickingHome = false;
      document.getElementById('btnSetHome').className = '';
      return;
    }
    const [wx, wy] = latLonToXY(e.latlng.lat, e.latlng.lng);
    setPoint(wx, wy);
  });

  function toggleSetHome() {
    pickingHome = !pickingHome;
    document.getElementById('btnSetHome').className = pickingHome ? 'active' : '';
  }

  async function searchPlace() {
    const q = document.getElementById('inSearch').value.trim();
    if (!q) return;
    const parts = q.split(',').map(s => parseFloat(s.trim()));
    if (parts.length === 2 && !isNaN(parts[0]) && !isNaN(parts[1])) {
      setHomeOrigin(parts[0], parts[1], true);
      return;
    }
    try {
      const res = await fetch(`https://nominatim.openstreetmap.org/search?format=json&limit=1&q=${encodeURIComponent(q)}`);
      const items = await res.json();
      if (items && items.length > 0) {
        setHomeOrigin(parseFloat(items[0].lat), parseFloat(items[0].lon), true);
      }
    } catch (e) {}
  }

  function locateUser() {
    if (!navigator.geolocation) return;
    navigator.geolocation.getCurrentPosition(
      (pos) => {
        setHomeOrigin(pos.coords.latitude, pos.coords.longitude, true);
      },
      () => {},
      { enableHighAccuracy: true, maximumAge: 0, timeout: 10000 }
    );
  }

  function clearTrail() {
    trailLine.setLatLngs([]);
  }

  function setMode(m) {
    mode = m;
    document.getElementById('btnLoiter').className = (m === 'loiter') ? 'active' : '';
    document.getElementById('btnFlyTo').className = (m === 'fly_to') ? 'active' : '';
    if (state && state.selected_mode !== 'auto') {
      applyInputs();
    }
  }

  function setPoint(x, y) {
    document.getElementById('inX').value = x.toFixed(1);
    document.getElementById('inY').value = y.toFixed(1);
    applyInputs();
  }

  async function applyInputs() {
    const x = parseFloat(document.getElementById('inX').value) || 0.0;
    const y = parseFloat(document.getElementById('inY').value) || 0.0;
    const z = parseFloat(document.getElementById('inZ').value) || 2.2;
    const r = parseFloat(document.getElementById('inR').value) || 4.5;
    await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: 'select_point', pos: [x, y, z], radius: r, mode: mode })
    });
  }

  async function startFlight() {
    clearTrail();
    await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: 'start_flight' })
    });
  }

  async function landFlight() {
    await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: 'land' })
    });
  }

  async function killFlight() {
    await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ cmd: 'kill' })
    });
  }

  const pfdCanvas = document.getElementById('pfdCanvas');
  const pfd = pfdCanvas.getContext('2d');

  function drawPFD() {
    if (!state || !state.att) return;
    const w = pfdCanvas.width, h = pfdCanvas.height;
    const r = w * 0.5;
    pfd.fillStyle = '#0c0c0b';
    pfd.fillRect(0, 0, w, h);

    const roll = state.att[0];
    const noseUp = -state.att[1];

    pfd.save();
    pfd.translate(r, r);
    pfd.rotate(roll);

    const py = noseUp * (r * 1.4);
    pfd.fillStyle = '#191815';
    pfd.fillRect(-w, py, w * 2, h * 2);

    pfd.strokeStyle = '#d8ccb8';
    pfd.lineWidth = 1.5;
    pfd.beginPath();
    pfd.moveTo(-w, py);
    pfd.lineTo(w, py);
    pfd.stroke();

    pfd.strokeStyle = '#5e584f';
    pfd.lineWidth = 1;
    [-20, -10, 10, 20].forEach(deg => {
      const y = py - (deg * Math.PI / 180) * (r * 1.4);
      pfd.beginPath();
      pfd.moveTo(-16, y);
      pfd.lineTo(16, y);
      pfd.stroke();
    });
    pfd.restore();

    pfd.strokeStyle = '#f4f1ea';
    pfd.lineWidth = 2;
    pfd.beginPath();
    pfd.moveTo(r - 30, r); pfd.lineTo(r - 10, r);
    pfd.moveTo(r + 10, r); pfd.lineTo(r + 30, r);
    pfd.stroke();
    pfd.fillStyle = '#f4f1ea';
    pfd.fillRect(r - 2, r - 2, 4, 4);
  }

  function setBiBar(idBar, idVal, val) {
    const el = document.getElementById(idBar);
    const pct = Math.min(1, Math.max(-1, val)) * 50;
    if (pct >= 0) {
      el.style.left = '50%';
      el.style.width = `${pct}%`;
    } else {
      el.style.left = `${50 + pct}%`;
      el.style.width = `${-pct}%`;
    }
    document.getElementById(idVal).textContent = (val >= 0 ? '+' : '') + val.toFixed(2);
  }

  async function poll() {
    try {
      const res = await fetch('/api/state');
      const d = await res.json();
      state = d;

      if (d.run !== lastRun) {
        clearTrail();
        lastRun = d.run;
      }

      if (d.home_gps && d.home_gps.length === 2) {
        const [hLat, hLon] = d.home_gps;
        if (Math.hypot(hLat - homeLat, hLon - homeLon) > 1e-5) {
          setHomeOrigin(hLat, hLon, true);
        }
      }

      const p = d.pos || [0, 0, 0];
      const lc = d.loiter_center || [0, 0, 2.2];
      const rad = d.loiter_radius || 4.5;
      const tgt = d.target || lc;
      const att = d.att || [0, 0, 0];

      const uavLL = xyToLatLon(p[0], p[1]);
      const lcLL = xyToLatLon(lc[0], lc[1]);
      const tgtLL = xyToLatLon(tgt[0], tgt[1]);

      uavMarker.setLatLng(uavLL);
      const rotEl = document.getElementById('uavRot');
      if (rotEl) {
        const yawDegSvg = -(att[2] * 180 / Math.PI);
        rotEl.setAttribute('transform', `rotate(${yawDegSvg.toFixed(1)})`);
      }

      loiterCircle.setLatLng(lcLL);
      loiterCircle.setRadius(rad);
      centerMarker.setLatLng(lcLL);
      carrotMarker.setLatLng(tgtLL);
      carrotLine.setLatLngs([uavLL, tgtLL]);

      const pts = trailLine.getLatLngs();
      if (pts.length === 0 || uavMarker.getLatLng().distanceTo(pts[pts.length - 1]) > 0.12) {
        pts.push(L.latLng(uavLL[0], uavLL[1]));
        if (pts.length > 450) pts.shift();
        trailLine.setLatLngs(pts);
      }

      const bankDeg = att[0] * 180 / Math.PI;
      const pitchDeg = -att[1] * 180 / Math.PI;
      const yawDeg = ((att[2] * 180 / Math.PI) % 360 + 360) % 360;
      const dist = Math.hypot(p[0] - lc[0], p[1] - lc[1]);
      const battPct = Math.round((d.battery ?? 1.0) * 100);

      document.getElementById('tPhase').textContent = d.phase || 'READY';
      document.getElementById('tBatt').textContent = battPct <= 20 ? `${battPct}% (LOW - RTL)` : `${battPct}%`;
      document.getElementById('tGps').textContent = `${uavLL[0].toFixed(5)}, ${uavLL[1].toFixed(5)}`;
      document.getElementById('tAirspeed').textContent = `${(d.airspeed || 0).toFixed(2)} m/s`;
      document.getElementById('tAlt').textContent = `${p[2].toFixed(2)} m`;
      document.getElementById('tPos').textContent = `${p[0]>=0?'+':''}${p[0].toFixed(2)}, ${p[1]>=0?'+':''}${p[1].toFixed(2)} m`;
      document.getElementById('tDist').textContent = `${dist.toFixed(2)} m`;
      document.getElementById('tRoll').textContent = `${bankDeg>=0?'+':''}${bankDeg.toFixed(1)} deg`;
      document.getElementById('tPitch').textContent = `${pitchDeg>=0?'+':''}${pitchDeg.toFixed(1)} deg`;
      document.getElementById('tYaw').textContent = `${String(Math.round(yawDeg)).padStart(3, '0')} deg`;
      document.getElementById('tAero').textContent =
        `${(d.alpha_deg||0)>=0?'+':''}${(d.alpha_deg||0).toFixed(1)} / ${(d.beta_deg||0)>=0?'+':''}${(d.beta_deg||0).toFixed(1)} deg`;

      const s = d.surfaces || [0.5, 0, 0, 0];
      const thr = Math.round(Math.min(1, Math.max(0, s[0])) * 100);
      document.getElementById('bThr').style.width = `${thr}%`;
      document.getElementById('vThr').textContent = `${thr}%`;
      setBiBar('bAil', 'vAil', s[1] || 0);
      setBiBar('bElv', 'vElv', s[2] || 0);
      setBiBar('bRud', 'vRud', s[3] || 0);

      drawPFD();
    } catch (e) {}
  }

  setInterval(poll, 40);
  poll();
</script>
</body>
</html>
"""


class _GCSRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            body = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif parsed.path == "/api/state":
            with _STATE_LOCK:
                body = json.dumps(_LATEST_PACKET).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/command":
            length = int(self.headers.get("Content-Length", "0"))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                msg = json.loads(raw.decode("utf-8"))
            except Exception:
                msg = {}
            cmd = msg.get("cmd", "")
            if cmd == "select_point" and "pos" in msg:
                p = msg["pos"]
                rad = msg.get("radius", None)
                mode = msg.get("mode", "loiter")
                set_selected_point(
                    p[0], p[1], p[2] if len(p) > 2 else None, radius=rad, mode=mode
                )
                print(
                    f"  [Web GCS Point Selected] ({p[0]:+.2f}, {p[1]:+.2f}, "
                    f"{p[2] if len(p)>2 else C.HOVER_ALT:.2f}) m  mode={mode}"
                )
            elif cmd == "clear_point":
                clear_selected_point()
                print("  [Web GCS Point Cleared] Returning to Home orbit.")
            elif cmd == "start_flight":
                request_launch()
                print("  [Web GCS Start Flight] Hand-throw launch triggered!")
            elif cmd == "land":
                request_land()
                print("  [Web GCS Land] Auto-landing triggered!")
            elif cmd == "kill":
                request_kill()
                print("  [Web GCS Kill] Emergency disarm triggered!")

            resp = json.dumps({"ok": True}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp)))
            self.end_headers()
            self.wfile.write(resp)
        else:
            self.send_error(404)


def start_web_gcs_server(host="0.0.0.0", port=8080):
    try:
        server = ThreadingHTTPServer((host, port), _GCSRequestHandler)
    except OSError:
        return None
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    return server


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()

    from main_godot import simulate_once
    from godot_bridge import GodotBridge

    bridge = GodotBridge(web_port=args.port)
    print("=" * 60)
    print(f"  UAV Control Web UI : http://127.0.0.1:{args.port}")
    print(f"  Godot UDP stream   : udp://{bridge.addr[0]}:{bridge.addr[1]}")
    print("=" * 60)

    try:
        while True:
            simulate_once(bridge, speed=1.0)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\n  Stopped.")
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
