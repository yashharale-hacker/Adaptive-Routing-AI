// =============================================
// ADAPTIVE ROUTING AI — CORE ENGINE
// Predictive D* Lite + A* Hybrid with Gas Spread Forecasting
// =============================================

const canvas = document.getElementById('cityCanvas');
const ctx = canvas.getContext('2d');

let GRID = 16;
let CELL = 0;
let state = {};
let initialState = null; // Store the initial city state for reset
let simInterval = null;
let mode = 'view';
let animFrame = null;
let tick = 0;
let simRunning = false;

// Colors
const C = {
  bg: '#060a0f',
  road: '#0d1e36',
  roadLine: '#1a3a5c',
  vehicle: '#00e5ff',
  hospitalOn: '#00ff88',
  hospitalOff: '#ff2d55',
  gas: '#39ff14',
  gasAlpha: 'rgba(57,255,20,',
  predicted: 'rgba(57,255,20,',
  substation: '#ffd60a',
  substationOff: '#4a3000',
  powerPlant: '#bf5af2',
  path: 'rgba(0,229,255,',
  blocked: '#1a0a0a',
  target: '#00e5ff',
};

function initState() {
  GRID = parseInt(document.getElementById('gridSize').value);

  const grid = [];
  for (let r = 0; r < GRID; r++) {
    grid[r] = [];
    for (let c = 0; c < GRID; c++) {
      grid[r][c] = {
        blocked: false,
        gas: false,
        gasPredicted: false,
        cost: 1,
        gasAge: 0,
      };
    }
  }

  // Place random obstacles (~10%)
  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      if (Math.random() < 0.08) grid[r][c].blocked = true;
    }
  }

  // Place hospitals (4-6)
  const numH = GRID >= 20 ? 6 : GRID >= 16 ? 5 : 4;
  const hospitals = [];
  const used = new Set();
  while (hospitals.length < numH) {
    const r = Math.floor(Math.random() * GRID);
    const c = Math.floor(Math.random() * GRID);
    const key = `${r},${c}`;
    if (!used.has(key) && !grid[r][c].blocked) {
      used.add(key);
      grid[r][c].blocked = false;
      hospitals.push({ r, c, powered: true, id: hospitals.length + 1 });
    }
  }

  // Place substations (power grid)
  const numS = Math.ceil(numH * 1.2);
  const substations = [];
  while (substations.length < numS) {
    const r = Math.floor(Math.random() * GRID);
    const c = Math.floor(Math.random() * GRID);
    const key = `${r},${c}`;
    if (!used.has(key) && !grid[r][c].blocked) {
      used.add(key);
      substations.push({ r, c, active: true, id: substations.length + 1 });
    }
  }

  // Power plant
  let pr, pc;
  do {
    pr = Math.floor(Math.random() * GRID);
    pc = Math.floor(Math.random() * GRID);
  } while (used.has(`${pr},${pc}`) || grid[pr][pc].blocked);
  used.add(`${pr},${pc}`);

  // Build power grid connections (each hospital connects to nearest substation)
  const powerEdges = buildPowerGrid(hospitals, substations, { r: pr, c: pc });

  // Vehicle start (far from hospitals)
  let vr, vc;
  let maxDist = -1;
  for (let attempt = 0; attempt < 200; attempt++) {
    const tr = Math.floor(Math.random() * GRID);
    const tc = Math.floor(Math.random() * GRID);
    if (used.has(`${tr},${tc}`) || grid[tr][tc].blocked) continue;
    const minD = Math.min(...hospitals.map(h => Math.abs(h.r - tr) + Math.abs(h.c - tc)));
    if (minD > maxDist) { maxDist = minD; vr = tr; vc = tc; }
  }

  // Gas leak source
  const gasOrigins = [];
  let gsr, gsc;
  for (let attempt = 0; attempt < 200; attempt++) {
    gsr = Math.floor(Math.random() * GRID);
    gsc = Math.floor(Math.random() * GRID);
    if (!used.has(`${gsr},${gsc}`) && !grid[gsr][gsc].blocked) {
      gasOrigins.push({ r: gsr, c: gsc });
      break;
    }
  }

  return {
    grid,
    hospitals,
    substations,
    powerPlant: { r: pr, c: pc },
    powerEdges,
    vehicle: { r: vr, c: vc },
    gasOrigins,
    gasActive: false,
    path: [],
    pathIndex: 0,
    targetHospital: null,
    replans: 0,
    nodesExpanded: 0,
    pathsAvoided: 0,
    steps: 0,
    replanTimes: [],
    naiveCost: null,
    ourCost: null,
    missionDone: false,
    missionFailed: false,
  };
}

function buildPowerGrid(hospitals, substations, plant) {
  // Each hospital connects to nearest 2 substations
  // Each substation connects to power plant
  const edges = [];
  for (const h of hospitals) {
    const sorted = [...substations].sort((a, b) =>
      (Math.abs(a.r - h.r) + Math.abs(a.c - h.c)) - (Math.abs(b.r - h.r) + Math.abs(b.c - h.c))
    );
    for (let i = 0; i < Math.min(2, sorted.length); i++) {
      edges.push({ from: h, to: sorted[i], type: 'h-s' });
    }
  }
  for (const s of substations) {
    edges.push({ from: s, to: plant, type: 's-p' });
  }
  return edges;
}

function updatePowerGrid() {
  // Check which substations are gassed
  for (const sub of state.substations) {
    const cell = state.grid[sub.r][sub.c];
    if (cell.gas) sub.active = false;
  }

  // BFS from power plant through active substations to hospitals
  const poweredSubs = new Set(state.substations.filter(s => s.active).map(s => s.id));

  for (const h of state.hospitals) {
    // Hospital is powered if it can reach power plant via active substations
    // Find connected substations
    const reachable = new Set();
    const queue = [...state.substations.filter(s => s.active)];
    // All active substations are connected to plant
    for (const s of state.substations) {
      if (s.active) reachable.add(s.id);
    }
    // Hospital is powered if any of its 2 connected substations are reachable
    const myEdges = state.powerEdges.filter(e => e.from === h && e.type === 'h-s');
    h.powered = myEdges.some(e => reachable.has(e.to.id));
  }
}

// =============================================
// PREDICTIVE GAS SPREAD MODEL
// Simulates where gas WILL be in N ticks
// =============================================
function predictGasSpread(ticks) {
  const spreadRate = parseInt(document.getElementById('spreadRate').value);
  // Simulate gas spread on a copy
  const gasCells = new Set();
  const predictedCells = new Set();

  // Current gas
  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      if (state.grid[r][c].gas) gasCells.add(`${r},${c}`);
    }
  }

  // Simulate spread
  let current = new Set(gasCells);
  for (let t = 0; t < ticks; t++) {
    const next = new Set(current);
    for (const key of current) {
      const [r, c] = key.split(',').map(Number);
      const dirs = [[0,1],[0,-1],[1,0],[-1,0]];
      for (const [dr, dc] of dirs) {
        const nr = r + dr, nc = c + dc;
        if (nr >= 0 && nr < GRID && nc >= 0 && nc < GRID) {
          if (!state.grid[nr][nc].blocked) {
            if (Math.random() < 0.4 * spreadRate / 3) {
              next.add(`${nr},${nc}`);
              predictedCells.add(`${nr},${nc}`);
            }
          }
        }
      }
    }
    current = next;
  }

  // Clear old predictions
  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      state.grid[r][c].gasPredicted = false;
    }
  }
  // Mark new predictions
  for (const key of predictedCells) {
    const [r, c] = key.split(',').map(Number);
    if (!state.grid[r][c].gas) {
      state.grid[r][c].gasPredicted = true;
    }
  }
}

// =============================================
// A* / PREDICTIVE D* LITE PATHFINDER
// =============================================
function heuristic(r1, c1, r2, c2) {
  return Math.abs(r1 - r2) + Math.abs(c1 - c2);
}

function getCellCost(r, c) {
  const cell = state.grid[r][c];
  if (cell.blocked) return Infinity;
  if (cell.gas) return 50; // Heavy penalty for gas zones
  let cost = cell.cost;
  // Predictive penalty: if gas predicted to arrive, penalize
  if (cell.gasPredicted) {
    const horizon = parseInt(document.getElementById('predictHorizon').value);
    cost += 15 * (1 - cell.gasAge / 10); // Graduated penalty
  }
  return cost;
}

function findPath(startR, startC, goals) {
  // Multi-goal A* with predictive penalties
  if (goals.length === 0) return null;

  const t0 = performance.now();
  const open = new MinHeap();
  const gScore = new Map();
  const parent = new Map();
  let nodesExp = 0;

  const startKey = `${startR},${startC}`;
  gScore.set(startKey, 0);

  // Heuristic: min distance to any goal
  const h = (r, c) => Math.min(...goals.map(g => heuristic(r, c, g.r, g.c)));

  open.push({ f: h(startR, startC), g: 0, r: startR, c: startC });

  while (!open.isEmpty()) {
    const curr = open.pop();
    nodesExp++;
    const key = `${curr.r},${curr.c}`;

    // Check if goal
    for (const g of goals) {
      if (curr.r === g.r && curr.c === g.c) {
        // Reconstruct path
        const path = [];
        let k = key;
        while (k) {
          const [r, c] = k.split(',').map(Number);
          path.unshift({ r, c });
          k = parent.get(k);
        }
        const elapsed = performance.now() - t0;
        state.nodesExpanded += nodesExp;
        state.replanTimes.push(elapsed);
        return { path, cost: curr.g, nodesExpanded: nodesExp, targetHospital: g };
      }
    }

    const dirs = [[0,1],[0,-1],[1,0],[-1,0]];
    for (const [dr, dc] of dirs) {
      const nr = curr.r + dr, nc = curr.c + dc;
      if (nr < 0 || nr >= GRID || nc < 0 || nc >= GRID) continue;
      const moveCost = getCellCost(nr, nc);
      if (moveCost === Infinity) continue;

      const newG = curr.g + moveCost;
      const nkey = `${nr},${nc}`;
      if (!gScore.has(nkey) || newG < gScore.get(nkey)) {
        gScore.set(nkey, newG);
        parent.set(nkey, key);
        open.push({ f: newG + h(nr, nc), g: newG, r: nr, c: nc });
      }
    }
  }
  return null; // No path found
}

// Naive A* without prediction (for comparison)
function findPathNaive(startR, startC, goals) {
  if (goals.length === 0) return null;
  const open = new MinHeap();
  const gScore = new Map();
  const parent = new Map();

  const startKey = `${startR},${startC}`;
  gScore.set(startKey, 0);
  const h = (r, c) => Math.min(...goals.map(g => heuristic(r, c, g.r, g.c)));
  open.push({ f: h(startR, startC), g: 0, r: startR, c: startC });

  while (!open.isEmpty()) {
    const curr = open.pop();
    const key = `${curr.r},${curr.c}`;

    for (const g of goals) {
      if (curr.r === g.r && curr.c === g.c) {
        return { cost: curr.g };
      }
    }

    const dirs = [[0,1],[0,-1],[1,0],[-1,0]];
    for (const [dr, dc] of dirs) {
      const nr = curr.r + dr, nc = curr.c + dc;
      if (nr < 0 || nr >= GRID || nc < 0 || nc >= GRID) continue;
      // Naive: only avoids current gas, not predicted
      const cell = state.grid[nr][nc];
      if (cell.blocked) continue;
      const moveCost = cell.gas ? 50 : cell.cost;
      if (moveCost === Infinity) continue;

      const newG = curr.g + moveCost;
      const nkey = `${nr},${nc}`;
      if (!gScore.has(nkey) || newG < gScore.get(nkey)) {
        gScore.set(nkey, newG);
        parent.set(nkey, key);
        open.push({ f: newG + h(nr, nc), g: newG, r: nr, c: nc });
      }
    }
  }
  return null;
}

// Min Heap for A*
class MinHeap {
  constructor() { this.data = []; }
  push(item) {
    this.data.push(item);
    this._bubbleUp(this.data.length - 1);
  }
  pop() {
    const top = this.data[0];
    const last = this.data.pop();
    if (this.data.length > 0) {
      this.data[0] = last;
      this._sinkDown(0);
    }
    return top;
  }
  isEmpty() { return this.data.length === 0; }
  _bubbleUp(i) {
    while (i > 0) {
      const p = Math.floor((i - 1) / 2);
      if (this.data[p].f <= this.data[i].f) break;
      [this.data[p], this.data[i]] = [this.data[i], this.data[p]];
      i = p;
    }
  }
  _sinkDown(i) {
    while (true) {
      let min = i, l = 2*i+1, r = 2*i+2;
      if (l < this.data.length && this.data[l].f < this.data[min].f) min = l;
      if (r < this.data.length && this.data[r].f < this.data[min].f) min = r;
      if (min === i) break;
      [this.data[min], this.data[i]] = [this.data[i], this.data[min]];
      i = min;
    }
  }
}

// =============================================
// SIMULATION ENGINE
// =============================================
function computeReplan() {
  updatePowerGrid();
  const horizon = parseInt(document.getElementById('predictHorizon').value);
  if (state.gasActive) predictGasSpread(horizon);

  const poweredHospitals = state.hospitals.filter(h => h.powered);
  const result = findPath(state.vehicle.r, state.vehicle.c, poweredHospitals);

  if (result) {
    const oldTarget = state.targetHospital;
    state.path = result.path;
    state.pathIndex = 0;
    state.targetHospital = result.targetHospital;
    state.ourCost = result.cost;

    // Count paths avoided due to prediction
    let avoided = 0;
    for (const node of result.path) {
      if (state.grid[node.r][node.c].gasPredicted) avoided++;
    }

    // Get naive comparison
    const naive = findPathNaive(state.vehicle.r, state.vehicle.c, poweredHospitals);
    if (naive) state.naiveCost = naive.cost;

    if (state.replans > 0 && (oldTarget?.id !== state.targetHospital?.id)) {
      log(`⚡ TARGET CHANGED → Hospital #${state.targetHospital.id}`, 'warn');
    }
    return true;
  } else {
    log('⚠ NO VALID PATH FOUND — all routes blocked!', 'danger');
    state.missionFailed = true;
    return false;
  }
}

function stepGasSpread() {
  const spreadRate = parseInt(document.getElementById('spreadRate').value);
  const newGas = [];

  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      if (state.grid[r][c].gas) {
        state.grid[r][c].gasAge++;
        const dirs = [[0,1],[0,-1],[1,0],[-1,0],[1,1],[-1,-1],[1,-1],[-1,1]];
        for (const [dr, dc] of dirs) {
          const nr = r + dr, nc = c + dc;
          if (nr >= 0 && nr < GRID && nc >= 0 && nc < GRID) {
            if (!state.grid[nr][nc].blocked && !state.grid[nr][nc].gas) {
              const prob = (Math.abs(dr) + Math.abs(dc) === 1 ? 0.35 : 0.12) * (spreadRate / 2);
              if (Math.random() < prob) {
                newGas.push([nr, nc]);
              }
            }
          }
        }
      }
    }
  }

  let newGasCount = 0;
  for (const [r, c] of newGas) {
    if (!state.grid[r][c].gas) {
      state.grid[r][c].gas = true;
      state.grid[r][c].cost = 50;
      newGasCount++;
    }
  }

  if (newGasCount > 0) {
    log(`☣ Gas spread: +${newGasCount} cells`, 'warn');
  }
}

function simTick() {
  if (state.missionDone || state.missionFailed) {
    stopSim();
    return;
  }

  tick++;
  document.getElementById('tickLabel').textContent = `TICK: ${tick}`;

  // Every 3 ticks spread gas
  if (state.gasActive && tick % 3 === 0) {
    stepGasSpread();
    // Check if gas hit a substation
    for (const sub of state.substations) {
      if (state.grid[sub.r][sub.c].gas && sub.active) {
        sub.active = false;
        log(`⚡ Substation #${sub.id} DISABLED by gas leak!`, 'danger');
      }
    }
    // Replan triggered by environment change
    const before = state.targetHospital?.id;
    state.replans++;
    computeReplan();
    if (state.targetHospital?.id !== before) {
      log(`↺ REPLAN #${state.replans}: new target Hospital #${state.targetHospital?.id}`, 'warn');
    }
  }

  // Move vehicle one step along path
  if (state.path.length > 1 && state.pathIndex < state.path.length - 1) {
    state.pathIndex++;
    const next = state.path[state.pathIndex];
    state.vehicle.r = next.r;
    state.vehicle.c = next.c;
    state.steps++;

    // If vehicle enters gas zone, force replan
    if (state.grid[next.r][next.c].gas) {
      log(`⚠ Vehicle entered gas zone! Emergency replan...`, 'danger');
      state.replans++;
      computeReplan();
    }

    // Check if arrived
    if (state.targetHospital && state.vehicle.r === state.targetHospital.r && state.vehicle.c === state.targetHospital.c) {
      state.missionDone = true;
      const savings = state.naiveCost ? (state.naiveCost - state.ourCost).toFixed(1) : '—';
      log(`✓ MISSION SUCCESS! Reached Hospital #${state.targetHospital.id}`, 'good');
      log(`✓ Path cost: ${state.ourCost?.toFixed(1)} | Replans: ${state.replans} | Steps: ${state.steps}`, 'good');
      log(`✓ Savings vs Naive A*: ${savings}`, 'good');
      document.getElementById('systemStatus').textContent = '✓ MISSION COMPLETE';
      document.getElementById('systemStatus').className = 'status-badge badge-active';
      document.getElementById('phaseLabel').textContent = 'PHASE: SUCCESS';
      stopSim();
      return;
    }
  }

  // Activate gas after 5 ticks
  if (!state.gasActive && tick === 5) {
    state.gasActive = true;
    for (const o of state.gasOrigins) {
      state.grid[o.r][o.c].gas = true;
      state.grid[o.r][o.c].cost = 50;
    }
    log(`☣ GAS LEAK DETECTED at (${state.gasOrigins[0]?.r},${state.gasOrigins[0]?.c})!`, 'danger');
    log(`⟳ Initiating predictive rerouting...`, 'info');
    computeReplan();
    document.getElementById('hazardStatus').textContent = '⚠ GAS LEAK ACTIVE';
    document.getElementById('hazardStatus').className = 'status-badge badge-danger pulse';
  }

  updateStats();
  render();
}

// =============================================
// RENDERING ENGINE
// =============================================
function render() {
  resizeCanvas();
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  CELL = Math.floor(Math.min(canvas.width, canvas.height) / GRID);
  const offsetX = Math.floor((canvas.width - CELL * GRID) / 2);
  const offsetY = Math.floor((canvas.height - CELL * GRID) / 2);

  // Draw grid cells
  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      const x = offsetX + c * CELL, y = offsetY + r * CELL;
      const cell = state.grid[r][c];

      // Base
      if (cell.blocked) {
        ctx.fillStyle = '#0a0a0a';
        ctx.fillRect(x, y, CELL, CELL);
        // Hatch pattern
        ctx.strokeStyle = '#1a1a2a';
        ctx.lineWidth = 0.5;
        for (let i = 0; i < CELL; i += 5) {
          ctx.beginPath(); ctx.moveTo(x+i, y); ctx.lineTo(x, y+i); ctx.stroke();
          ctx.beginPath(); ctx.moveTo(x+CELL, y+i); ctx.lineTo(x+i, y+CELL); ctx.stroke();
        }
      } else {
        ctx.fillStyle = C.road;
        ctx.fillRect(x, y, CELL, CELL);
      }

      // Gas zones
      if (cell.gas) {
        const age = Math.min(cell.gasAge / 5, 1);
        ctx.fillStyle = `rgba(57,255,20,${0.25 + age * 0.25})`;
        ctx.fillRect(x, y, CELL, CELL);
        // Gas particle effect
        if (CELL > 20) {
          ctx.fillStyle = `rgba(57,255,20,${0.6})`;
          for (let i = 0; i < 2; i++) {
            const px = x + 2 + Math.random() * (CELL-4);
            const py = y + 2 + Math.random() * (CELL-4);
            ctx.fillRect(px, py, 1.5, 1.5);
          }
        }
      }

      // Predicted gas (dashed overlay)
      if (cell.gasPredicted && !cell.gas) {
        ctx.fillStyle = 'rgba(57,255,20,0.07)';
        ctx.fillRect(x, y, CELL, CELL);
        ctx.strokeStyle = 'rgba(57,255,20,0.3)';
        ctx.lineWidth = 0.5;
        ctx.setLineDash([2, 3]);
        ctx.strokeRect(x+1, y+1, CELL-2, CELL-2);
        ctx.setLineDash([]);
      }

      // Grid lines
      ctx.strokeStyle = 'rgba(26,58,92,0.5)';
      ctx.lineWidth = 0.5;
      ctx.strokeRect(x, y, CELL, CELL);
    }
  }

  // Draw planned path
  if (state.path.length > 0) {
    ctx.beginPath();
    let pathStarted = false;
    for (let i = state.pathIndex; i < state.path.length; i++) {
      const { r, c } = state.path[i];
      const x = offsetX + c * CELL + CELL / 2;
      const y = offsetY + r * CELL + CELL / 2;
      if (!pathStarted) { ctx.moveTo(x, y); pathStarted = true; }
      else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = 'rgba(0,229,255,0.5)';
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    // Path dots
    for (let i = state.pathIndex; i < state.path.length; i++) {
      const { r, c } = state.path[i];
      const x = offsetX + c * CELL + CELL / 2;
      const y = offsetY + r * CELL + CELL / 2;
      const alpha = 0.2 + 0.5 * (i / state.path.length);
      ctx.beginPath();
      ctx.arc(x, y, 2, 0, Math.PI * 2);
      ctx.fillStyle = `rgba(0,229,255,${alpha})`;
      ctx.fill();
    }
  }

  // Draw power grid lines
  ctx.save();
  ctx.globalAlpha = 0.25;
  for (const edge of state.powerEdges) {
    const active = (edge.type === 'h-s' ? edge.to.active : true) && edge.from.powered !== false;
    ctx.strokeStyle = active ? '#bf5af2' : '#3a1a5c';
    ctx.lineWidth = 0.8;
    ctx.setLineDash([3, 5]);
    ctx.beginPath();
    ctx.moveTo(offsetX + edge.from.c * CELL + CELL/2, offsetY + edge.from.r * CELL + CELL/2);
    ctx.lineTo(offsetX + edge.to.c * CELL + CELL/2, offsetY + edge.to.r * CELL + CELL/2);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  ctx.restore();

  // Draw substations
  for (const sub of state.substations) {
    const x = offsetX + sub.c * CELL + CELL/2;
    const y = offsetY + sub.r * CELL + CELL/2;
    const sz = CELL * 0.35;
    ctx.fillStyle = sub.active ? C.substation : C.substationOff;
    ctx.strokeStyle = sub.active ? '#ffd60a' : '#4a3000';
    ctx.lineWidth = 1;
    // Diamond shape
    ctx.beginPath();
    ctx.moveTo(x, y - sz);
    ctx.lineTo(x + sz, y);
    ctx.lineTo(x, y + sz);
    ctx.lineTo(x - sz, y);
    ctx.closePath();
    ctx.fill(); ctx.stroke();

    if (sub.active && CELL > 20) {
      ctx.fillStyle = 'rgba(255,214,10,0.15)';
      ctx.beginPath();
      ctx.arc(x, y, sz * 2, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  // Power plant
  const pp = state.powerPlant;
  if (pp) {
    const x = offsetX + pp.c * CELL + CELL/2;
    const y = offsetY + pp.r * CELL + CELL/2;
    const sz = CELL * 0.42;
    // Glow
    const grad = ctx.createRadialGradient(x, y, 0, x, y, sz * 2.5);
    grad.addColorStop(0, 'rgba(191,90,242,0.3)');
    grad.addColorStop(1, 'rgba(191,90,242,0)');
    ctx.fillStyle = grad;
    ctx.beginPath(); ctx.arc(x, y, sz * 2.5, 0, Math.PI * 2); ctx.fill();

    ctx.fillStyle = C.powerPlant;
    ctx.strokeStyle = '#d090ff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y - sz);
    for (let i = 1; i <= 6; i++) {
      const angle = (i * Math.PI * 2 / 6) - Math.PI/2;
      ctx.lineTo(x + sz * Math.cos(angle), y + sz * Math.sin(angle));
    }
    ctx.closePath(); ctx.fill(); ctx.stroke();

    if (CELL > 18) {
      ctx.fillStyle = 'white';
      ctx.font = `bold ${Math.max(8, CELL * 0.3)}px Orbitron`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('⚡', x, y);
    }
  }

  // Draw hospitals
  for (const h of state.hospitals) {
    const x = offsetX + h.c * CELL + CELL/2;
    const y = offsetY + h.r * CELL + CELL/2;
    const sz = CELL * 0.42;
    const isTarget = state.targetHospital?.id === h.id;

    // Glow for target
    if (isTarget && h.powered) {
      const grad = ctx.createRadialGradient(x, y, 0, x, y, sz * 3);
      grad.addColorStop(0, 'rgba(0,229,255,0.25)');
      grad.addColorStop(1, 'rgba(0,229,255,0)');
      ctx.fillStyle = grad;
      ctx.beginPath(); ctx.arc(x, y, sz * 3, 0, Math.PI * 2); ctx.fill();

      // Pulsing ring
      const pulse = 0.6 + 0.4 * Math.sin(tick * 0.3);
      ctx.strokeStyle = `rgba(0,229,255,${pulse})`;
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y, sz * 2.2, 0, Math.PI * 2); ctx.stroke();
    }

    // Hospital square with cross
    ctx.fillStyle = h.powered ? C.hospitalOn : C.hospitalOff;
    ctx.strokeStyle = h.powered ? '#00ff88' : '#ff2d55';
    ctx.lineWidth = isTarget ? 2 : 1;
    ctx.beginPath();
    ctx.roundRect(x - sz, y - sz, sz * 2, sz * 2, 3);
    ctx.fill(); ctx.stroke();

    // H cross symbol
    if (CELL > 14) {
      ctx.fillStyle = 'rgba(0,0,0,0.6)';
      const cw = sz * 0.35, ch = sz * 1.2;
      ctx.fillRect(x - cw/2, y - ch/2, cw, ch);
      ctx.fillRect(x - ch/2, y - cw/2, ch, cw);
      ctx.fillStyle = 'white';
      ctx.font = `bold ${Math.max(7, CELL * 0.28)}px Exo 2`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('H', x, y);
    }

    // Hospital ID label
    if (CELL > 18) {
      ctx.fillStyle = h.powered ? C.hospitalOn : C.hospitalOff;
      ctx.font = `${Math.max(7, CELL * 0.22)}px Share Tech Mono`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      ctx.fillText(`#${h.id}`, x, y + sz + 2);
    }
  }

  // Gas origin marker
  for (const o of state.gasOrigins) {
    const x = offsetX + o.c * CELL + CELL/2;
    const y = offsetY + o.r * CELL + CELL/2;
    if (state.gasActive) {
      ctx.strokeStyle = 'rgba(57,255,20,0.8)';
      ctx.lineWidth = 2;
      const sz = CELL * 0.45;
      ctx.beginPath();
      ctx.arc(x, y, sz, 0, Math.PI * 2);
      ctx.stroke();
    }
  }

  // Draw vehicle
  const vx = offsetX + state.vehicle.c * CELL + CELL/2;
  const vy = offsetY + state.vehicle.r * CELL + CELL/2;
  const vsz = CELL * 0.38;

  // Vehicle glow
  const vGrad = ctx.createRadialGradient(vx, vy, 0, vx, vy, vsz * 3);
  vGrad.addColorStop(0, 'rgba(0,229,255,0.4)');
  vGrad.addColorStop(1, 'rgba(0,229,255,0)');
  ctx.fillStyle = vGrad;
  ctx.beginPath(); ctx.arc(vx, vy, vsz * 3, 0, Math.PI * 2); ctx.fill();

  // Vehicle body
  ctx.fillStyle = C.vehicle;
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(vx, vy, vsz, 0, Math.PI * 2);
  ctx.fill(); ctx.stroke();

  // Vehicle symbol
  if (CELL > 14) {
    ctx.fillStyle = '#060a0f';
    ctx.font = `bold ${Math.max(8, CELL * 0.32)}px Arial`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('🚑', vx, vy);
  }

  // Mission done/failed overlay
  if (state.missionDone || state.missionFailed) {
    ctx.fillStyle = state.missionDone ? 'rgba(0,255,136,0.12)' : 'rgba(255,45,85,0.12)';
    ctx.fillRect(offsetX, offsetY, GRID * CELL, GRID * CELL);
    ctx.fillStyle = state.missionDone ? C.hospitalOn : C.hospitalOff;
    ctx.font = `bold ${CELL * 1.2}px Orbitron`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(state.missionDone ? '✓' : '✗', offsetX + GRID * CELL / 2, offsetY + GRID * CELL / 2);
  }
}

function resizeCanvas() {
  const area = canvas.parentElement;
  const rect = area.getBoundingClientRect();
  const toolbar = document.querySelector('.toolbar');
  const toolbarHeight = toolbar ? toolbar.getBoundingClientRect().height : 0;
  const newWidth = rect.width;
  const newHeight = rect.height - toolbarHeight;
  if (canvas.width !== newWidth || canvas.height !== newHeight) {
    canvas.width = newWidth;
    canvas.height = newHeight;
  }
}

// =============================================
// STATS & UI
// =============================================
function updateStats() {
  let gasCount = 0;
  for (let r = 0; r < GRID; r++)
    for (let c = 0; c < GRID; c++)
      if (state.grid[r][c].gas) gasCount++;

  document.getElementById('statCost').textContent = state.ourCost?.toFixed(1) || '—';
  document.getElementById('statNodes').textContent = state.nodesExpanded || '—';
  document.getElementById('statReplans').textContent = state.replans;
  document.getElementById('statGas').textContent = gasCount;
  document.getElementById('statSteps').textContent = state.steps;
  document.getElementById('statAvoided').textContent = state.pathsAvoided;

  const activeSubs = state.substations.filter(s => s.active).length;
  const poweredH = state.hospitals.filter(h => h.powered).length;
  document.getElementById('statSubs').textContent = `${activeSubs}/${state.substations.length}`;
  document.getElementById('statHospsPowered').textContent = `${poweredH}/${state.hospitals.length}`;
  const health = Math.round((activeSubs / state.substations.length) * 100);
  document.getElementById('statGridHealth').textContent = `${health}%`;

  if (state.replanTimes.length > 0) {
    const avg = state.replanTimes.reduce((a,b)=>a+b,0)/state.replanTimes.length;
    document.getElementById('statReplanTime').textContent = `${avg.toFixed(1)}ms`;
  }
  document.getElementById('statHeuristic').textContent = 'Admissible ✓';

  // Progress
  if (state.path.length > 1) {
    const pct = Math.round((state.pathIndex / (state.path.length - 1)) * 100);
    document.getElementById('progressBar').style.width = `${pct}%`;
    document.getElementById('progressLabel').textContent =
      `Step ${state.pathIndex}/${state.path.length-1} → Hospital #${state.targetHospital?.id || '?'}`;
  }

  // Comparison
  if (state.naiveCost && state.ourCost) {
    document.getElementById('comparisonBox').style.display = 'block';
    document.getElementById('cmpOurs').textContent = state.ourCost.toFixed(1);
    document.getElementById('cmpNaive').textContent = state.naiveCost.toFixed(1);
    const savings = (state.naiveCost - state.ourCost).toFixed(1);
    document.getElementById('cmpSavings').textContent = savings > 0 ? `${savings} (${Math.round(savings/state.naiveCost*100)}%)` : '—';
  }

  updateHospitalList();
}

function updateHospitalList() {
  const list = document.getElementById('hospitalList');
  list.innerHTML = '';
  for (const h of state.hospitals) {
    const isTarget = state.targetHospital?.id === h.id;
    const cls = isTarget ? 'target' : h.powered ? 'powered' : 'unpowered';
    const dist = Math.abs(h.r - state.vehicle.r) + Math.abs(h.c - state.vehicle.c);
    list.innerHTML += `
      <div class="hospital-item ${cls}">
        <div class="h-dot" style="background:${h.powered ? 'var(--green)' : 'var(--red)'}"></div>
        <div style="flex:1">
          <div style="color:${isTarget ? 'var(--accent)' : h.powered ? 'var(--green)' : 'var(--red)'}">
            Hospital #${h.id} ${isTarget ? '← TARGET' : ''}
          </div>
          <div style="color:var(--text-dim);font-size:9px">
            (${h.r},${h.c}) · dist: ${dist} · ${h.powered ? 'POWERED' : 'OFFLINE'}
          </div>
        </div>
      </div>`;
  }
}

function log(msg, type = 'info') {
  const box = document.getElementById('logBox');
  const t = String(tick).padStart(3, '0');
  box.innerHTML += `<div class="log-entry ${type}">[${t}] ${msg}</div>`;
  box.scrollTop = box.scrollHeight;

  // Switch to log tab if danger
  if (type === 'danger') switchTab('log');
}

function switchTab(name) {
  document.querySelectorAll('.tab').forEach((t, i) => {
    const tabs = ['stats','hospitals','log'];
    t.classList.toggle('active', tabs[i] === name);
  });
  document.querySelectorAll('.tab-content').forEach(tc => {
    tc.classList.toggle('active', tc.id === `tab-${name}`);
  });
}

// =============================================
// CONTROL FUNCTIONS
// =============================================
function generateCity() {
  stopSim();
  tick = 0;
  state = initState();
  initialState = JSON.parse(JSON.stringify(state)); // Save initial state for reset
  updatePowerGrid();
  computeReplan();
  updateStats();
  render();
  log('► New city generated. Ready for mission launch.', 'info');
  document.getElementById('systemStatus').textContent = '● STANDBY';
  document.getElementById('systemStatus').className = 'status-badge badge-active';
  document.getElementById('hazardStatus').textContent = '◈ NO HAZARD';
  document.getElementById('hazardStatus').className = 'status-badge badge-warn';
  document.getElementById('phaseLabel').textContent = 'PHASE: INIT';
  document.getElementById('progressBar').style.width = '0%';
}

function startSim() {
  if (simRunning) {
    stopSim();
    document.getElementById('btnStart').textContent = '▶ LAUNCH MISSION';
    return;
  }
  if (!state.path || state.path.length === 0) {
    computeReplan();
  }
  simRunning = true;
  document.getElementById('btnStart').textContent = '⏸ PAUSE';
  document.getElementById('btnStep').disabled = false;
  document.getElementById('systemStatus').textContent = '● MISSION ACTIVE';
  document.getElementById('phaseLabel').textContent = 'PHASE: NAVIGATE';
  log('► Mission launched. Vehicle deploying...', 'info');

  const speed = parseInt(document.getElementById('speedSlider').value);
  const delay = Math.round(1200 / speed);
  simInterval = setInterval(simTick, delay);
}

function stopSim() {
  simRunning = false;
  if (simInterval) { clearInterval(simInterval); simInterval = null; }
  document.getElementById('btnStart').textContent = '▶ LAUNCH MISSION';
}

function stepSim() {
  simTick();
  render();
}

function resetSim() {
  stopSim();
  tick = 0;
  state = initialState ? JSON.parse(JSON.stringify(initialState)) : initState();
  // Rebuild powerEdges with correct object references
  if (state.hospitals && state.substations && state.powerPlant) {
    state.powerEdges = buildPowerGrid(state.hospitals, state.substations, state.powerPlant);
  }
  updatePowerGrid();
  computeReplan();
  updateStats();
  render();
  document.getElementById('systemStatus').textContent = '● STANDBY';
  document.getElementById('systemStatus').className = 'status-badge badge-active';
  document.getElementById('hazardStatus').textContent = '◈ NO HAZARD';
  document.getElementById('hazardStatus').className = 'status-badge badge-warn';
  document.getElementById('phaseLabel').textContent = 'PHASE: INIT';
  document.getElementById('progressBar').style.width = '0%';
  document.getElementById('btnStep').disabled = true;
  log('↺ Simulation reset.', 'dim');
}

function setMode(m) {
  mode = m;
  ['modeView','modeBlock','modeGas'].forEach(id => {
    const btn = document.getElementById(id);
    btn.style.borderColor = '';
    btn.style.color = '';
    btn.style.background = '';
  });
  const activeBtn = document.getElementById(`mode${m.charAt(0).toUpperCase()+m.slice(1)}`);
  if (activeBtn) {
    activeBtn.style.borderColor = 'var(--accent2)';
    activeBtn.style.color = 'var(--accent2)';
    activeBtn.style.background = 'rgba(255,107,53,0.15)';
  }
}

canvas.addEventListener('click', (e) => {
  if (mode === 'view') return;
  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left, my = e.clientY - rect.top;
  const CELL2 = Math.floor(Math.min(canvas.width, canvas.height) / GRID);
  const offsetX = Math.floor((canvas.width - CELL2 * GRID) / 2);
  const offsetY = Math.floor((canvas.height - CELL2 * GRID) / 2);
  const c = Math.floor((mx - offsetX) / CELL2);
  const r = Math.floor((my - offsetY) / CELL2);
  if (r < 0 || r >= GRID || c < 0 || c >= GRID) return;

  if (mode === 'block') {
    state.grid[r][c].blocked = !state.grid[r][c].blocked;
    state.grid[r][c].gas = false;
    computeReplan();
    log(`✎ Road ${state.grid[r][c].blocked ? 'BLOCKED' : 'OPENED'} at (${r},${c})`, 'warn');
  } else if (mode === 'gas') {
    state.grid[r][c].gas = true;
    state.grid[r][c].cost = 50;
    state.gasActive = true;
    document.getElementById('hazardStatus').textContent = '⚠ GAS LEAK ACTIVE';
    document.getElementById('hazardStatus').className = 'status-badge badge-danger pulse';
    computeReplan();
    log(`☣ Gas manually placed at (${r},${c})`, 'danger');
  }
  updateStats();
  render();
});

// Speed slider
document.getElementById('speedSlider').addEventListener('input', function() {
  document.getElementById('speedVal').textContent = `${this.value}x`;
  if (simRunning) {
    stopSim();
    simRunning = true;
    document.getElementById('btnStart').textContent = '⏸ PAUSE';
    const delay = Math.round(1200 / parseInt(this.value));
    simInterval = setInterval(simTick, delay);
  }
});

document.getElementById('spreadRate').addEventListener('input', function() {
  document.getElementById('spreadVal').textContent = this.value;
});
document.getElementById('predictHorizon').addEventListener('input', function() {
  document.getElementById('predictVal').textContent = this.value;
});

// Render loop
function renderLoop() {
  if (!simRunning) render();
  animFrame = requestAnimationFrame(renderLoop);
}

// INIT
window.addEventListener('resize', () => { resizeCanvas(); render(); });
generateCity();
renderLoop();
log('► System initialized. Predictive D* Lite engine ready.', 'info');
log('► Click LAUNCH MISSION to begin. Gas leak will trigger at tick 5.', 'dim');
log('► Use BLOCK ROAD or PLACE GAS modes to interact.', 'dim');