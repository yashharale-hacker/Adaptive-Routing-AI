// =============================================
// ADAPTIVE ROUTING AI — CORE ENGINE
// Algorithms:
//   1. Predictive A* — full A* re-run with hazard penalty p(n)
//   2. D* Lite       — true incremental D* Lite (Koenig & Likhachev 2002)
//                      with predictive hazard costs injected into edge weights
// =============================================

const canvas = document.getElementById('cityCanvas');
const ctx = canvas.getContext('2d');

let GRID = 16;
let CELL = 0;
let state = {};
let initialState = null;
let preLaunchState = null;
let simInterval = null;
let mode = 'view';
let selectedHospital = null;
let selectedSubstation = null;
let animFrame = null;
let tick = 0;
let simRunning = false;

// Which algorithm is active: 'astar' | 'dstar'
let activeAlgorithm = 'astar';

// D* Lite planner instance (persists across ticks for incremental replanning)
let dstarPlanner = null;

// Colors
const C = {
  bg: '#060a0f',
  road: '#0d1e36',
  vehicle: '#00e5ff',
  hospitalOn: '#00ff88',
  hospitalOff: '#ff2d55',
  gas: '#39ff14',
  substation: '#ffd60a',
  substationOff: '#4a3000',
  powerPlant: '#bf5af2',
  pathAstar: 'rgba(0,229,255,',
  pathDstar: 'rgba(255,107,53,',
};

// =============================================
// WORLD INIT
// =============================================
function initState() {
  GRID = parseInt(document.getElementById('gridSize').value);

  const grid = [];
  for (let r = 0; r < GRID; r++) {
    grid[r] = [];
    for (let c = 0; c < GRID; c++) {
      grid[r][c] = { blocked: false, gas: false, gasPredicted: false, cost: 1, gasAge: 0 };
    }
  }

  for (let r = 0; r < GRID; r++)
    for (let c = 0; c < GRID; c++)
      if (Math.random() < 0.08) grid[r][c].blocked = true;

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

  let pr, pc;
  do {
    pr = Math.floor(Math.random() * GRID);
    pc = Math.floor(Math.random() * GRID);
  } while (used.has(`${pr},${pc}`) || grid[pr][pc].blocked);
  used.add(`${pr},${pc}`);

  const powerEdges = buildPowerGrid(hospitals, substations, { r: pr, c: pc });

  let vr = 0, vc = 0;
  let maxDist = -1;
  for (let attempt = 0; attempt < 300; attempt++) {
    const tr = Math.floor(Math.random() * GRID);
    const tc = Math.floor(Math.random() * GRID);
    if (used.has(`${tr},${tc}`) || grid[tr][tc].blocked) continue;
    const minD = Math.min(...hospitals.map(h => Math.abs(h.r - tr) + Math.abs(h.c - tc)));
    if (minD > maxDist) { maxDist = minD; vr = tr; vc = tc; }
  }

  const gasOrigins = [];
  for (let attempt = 0; attempt < 200; attempt++) {
    const gsr = Math.floor(Math.random() * GRID);
    const gsc = Math.floor(Math.random() * GRID);
    if (!used.has(`${gsr},${gsc}`) && !grid[gsr][gsc].blocked) {
      gasOrigins.push({ r: gsr, c: gsc });
      break;
    }
  }

  return {
    grid, hospitals, substations,
    powerPlant: { r: pr, c: pc },
    powerEdges,
    vehicle: { r: vr, c: vc },
    gasOrigins,
    gasActive: false,
    path: [], pathIndex: 0,
    targetHospital: null,
    replans: 0, nodesExpanded: 0, nodesUpdated: 0,
    pathsAvoided: 0, steps: 0,
    replanTimes: [],
    naiveCost: null, ourCost: null,
    missionDone: false, missionFailed: false,
  };
}

function buildPowerGrid(hospitals, substations, plant) {
  const edges = [];
  for (const h of hospitals) {
    const sorted = [...substations].sort((a, b) =>
      (Math.abs(a.r - h.r) + Math.abs(a.c - h.c)) - (Math.abs(b.r - h.r) + Math.abs(b.c - h.c))
    );
    for (let i = 0; i < Math.min(2, sorted.length); i++)
      edges.push({ from: h, to: sorted[i], type: 'h-s' });
  }
  for (const s of substations)
    edges.push({ from: s, to: plant, type: 's-p' });
  return edges;
}

function updatePowerGrid() {
  for (const sub of state.substations)
    if (state.grid[sub.r][sub.c].gas) sub.active = false;

  const reachable = new Set(state.substations.filter(s => s.active).map(s => s.id));
  for (const h of state.hospitals) {
    const myEdges = state.powerEdges.filter(e => e.from === h && e.type === 'h-s');
    h.powered = myEdges.some(e => reachable.has(e.to.id));
  }
}

// =============================================
// GAS SPREAD PREDICTION
// =============================================
function predictGasSpread(ticks) {
  const spreadRate = parseInt(document.getElementById('spreadRate').value);
  const predictedCells = new Set();
  const oldPredicted = new Set();
  let current = new Set();

  for (let r = 0; r < GRID; r++)
    for (let c = 0; c < GRID; c++)
      if (state.grid[r][c].gasPredicted) oldPredicted.add(`${r},${c}`);

  for (let r = 0; r < GRID; r++)
    for (let c = 0; c < GRID; c++)
      if (state.grid[r][c].gas) current.add(`${r},${c}`);

  for (let t = 0; t < ticks; t++) {
    const next = new Set(current);
    for (const key of current) {
      const [r, c] = key.split(',').map(Number);
      for (const [dr, dc] of [[0,1],[0,-1],[1,0],[-1,0]]) {
        const nr = r + dr, nc = c + dc;
        if (nr >= 0 && nr < GRID && nc >= 0 && nc < GRID && !state.grid[nr][nc].blocked) {
          if (Math.random() < 0.4 * spreadRate / 3) {
            next.add(`${nr},${nc}`);
            predictedCells.add(`${nr},${nc}`);
          }
        }
      }
    }
    current = next;
  }

  const changed = [];
  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      const key = `${r},${c}`;
      const wasPredicted = oldPredicted.has(key);
      const nowPredicted = predictedCells.has(key) && !state.grid[r][c].gas;
      state.grid[r][c].gasPredicted = nowPredicted;
      if (wasPredicted !== nowPredicted) changed.push({ r, c });
    }
  }

  return changed;
}

// =============================================
// EDGE COST FUNCTION (shared by both algorithms)
// c(u, v) — cost to traverse cell v from u
// =============================================
function edgeCost(r, c) {
  const cell = state.grid[r][c];
  if (cell.blocked) return Infinity;
  if (cell.gas) return 50;
  let cost = cell.cost;
  const horizon = parseInt(document.getElementById('predictHorizon').value);
  if (horizon > 0 && cell.gasPredicted) cost += 15;
  return cost;
}

function heuristic(r1, c1, r2, c2) {
  return Math.abs(r1 - r2) + Math.abs(c1 - c2);
}

// =============================================
// ALGORITHM 1: PREDICTIVE A*
// Full A* re-run each time environment changes.
// f(n) = g(n) + h(n) + p(n)
// p(n) is embedded in edgeCost via gasPredicted flag.
// =============================================
function astarFindPath(startR, startC, goal) {
  const t0 = performance.now();
  const open = new MinHeap();
  const gScore = new Map();
  const parent = new Map();
  let nodesExp = 0;

  const sk = `${startR},${startC}`;
  gScore.set(sk, 0);
  open.push({ f: heuristic(startR, startC, goal.r, goal.c), g: 0, r: startR, c: startC });

  while (!open.isEmpty()) {
    const curr = open.pop();
    nodesExp++;
    const key = `${curr.r},${curr.c}`;

    if (curr.r === goal.r && curr.c === goal.c) {
      const path = [];
      let k = key;
      while (k) { const [r, c] = k.split(',').map(Number); path.unshift({ r, c }); k = parent.get(k); }
      state.replanTimes.push(performance.now() - t0);
      return { path, cost: curr.g, nodesExpanded: nodesExp };
    }

    for (const [dr, dc] of [[0,1],[0,-1],[1,0],[-1,0]]) {
      const nr = curr.r + dr, nc = curr.c + dc;
      if (nr < 0 || nr >= GRID || nc < 0 || nc >= GRID) continue;
      const moveCost = edgeCost(nr, nc);
      if (moveCost === Infinity) continue;
      const newG = curr.g + moveCost;
      const nkey = `${nr},${nc}`;
      if (!gScore.has(nkey) || newG < gScore.get(nkey)) {
        gScore.set(nkey, newG);
        parent.set(nkey, key);
        open.push({ f: newG + heuristic(nr, nc, goal.r, goal.c), g: newG, r: nr, c: nc });
      }
    }
  }
  return null;
}

function astarNaive(startR, startC, goal) {
  // No predictive penalty — only reacts to current gas
  const open = new MinHeap();
  const gScore = new Map();
  gScore.set(`${startR},${startC}`, 0);
  open.push({ f: heuristic(startR, startC, goal.r, goal.c), g: 0, r: startR, c: startC });

  while (!open.isEmpty()) {
    const curr = open.pop();
    const key = `${curr.r},${curr.c}`;
    if (curr.r === goal.r && curr.c === goal.c) return { cost: curr.g };
    for (const [dr, dc] of [[0,1],[0,-1],[1,0],[-1,0]]) {
      const nr = curr.r + dr, nc = curr.c + dc;
      if (nr < 0 || nr >= GRID || nc < 0 || nc >= GRID) continue;
      const cell = state.grid[nr][nc];
      if (cell.blocked) continue;
      const moveCost = cell.gas ? 50 : cell.cost;
      const newG = curr.g + moveCost;
      const nkey = `${nr},${nc}`;
      if (!gScore.has(nkey) || newG < gScore.get(nkey)) {
        gScore.set(nkey, newG);
        open.push({ f: newG + heuristic(nr, nc, goal.r, goal.c), g: newG, r: nr, c: nc });
      }
    }
  }
  return null;
}

function distanceToNearestGas(r, c) {
  let best = Infinity;
  for (let i = 0; i < GRID; i++)
    for (let j = 0; j < GRID; j++)
      if (state.grid[i][j].gas || state.grid[i][j].gasPredicted)
        best = Math.min(best, Math.abs(i - r) + Math.abs(j - c));
  return best;
}

// =============================================
// ALGORITHM 2: TRUE D* LITE
// Koenig & Likhachev 2002.
// Searches BACKWARD from goal to start.
// rhs(s): one-step lookahead value
// g(s): current best cost estimate
// key(s): priority [min(g,rhs)+h(s,start), min(g,rhs)]
// Only nodes affected by cost changes are re-expanded.
// =============================================
class DStarLite {
  constructor(goal, start) {
    // D* Lite searches BACKWARD: goal → start
    // "s_goal" is our routing goal (hospital), "s_start" is vehicle position
    this.goal = goal;   // { r, c }
    this.start = start; // { r, c }
    this.km = 0;        // key modifier (accumulated heuristic shift)
    this.g = new Map();
    this.rhs = new Map();
    this.U = new MinHeapDstar(); // priority queue
    this.nodesUpdated = 0;

    // Init: all g = rhs = Infinity except goal node rhs = 0
    this._rhs(goal.r, goal.c, 0);
    this.U.insert(this._key(goal.r, goal.c), { r: goal.r, c: goal.c });
  }

  _key(r, c) {
    const gv = this._g(r, c);
    const rv = this._rhs(r, c);
    const m = Math.min(gv, rv);
    const h = heuristic(r, c, this.start.r, this.start.c);
    return [m + h + this.km, m];
  }

  _g(r, c, val) {
    const k = `${r},${c}`;
    if (val !== undefined) { this.g.set(k, val); return val; }
    return this.g.has(k) ? this.g.get(k) : Infinity;
  }

  _rhs(r, c, val) {
    const k = `${r},${c}`;
    if (val !== undefined) { this.rhs.set(k, val); return val; }
    return this.rhs.has(k) ? this.rhs.get(k) : Infinity;
  }

  neighbors(r, c) {
    const result = [];
    for (const [dr, dc] of [[0,1],[0,-1],[1,0],[-1,0]]) {
      const nr = r + dr, nc = c + dc;
      if (nr >= 0 && nr < GRID && nc >= 0 && nc < GRID)
        result.push({ r: nr, c: nc });
    }
    return result;
  }

  // Cost of moving INTO cell (nr, nc) [backward search: predecessor cost]
  _cost(nr, nc) {
    return edgeCost(nr, nc);
  }

  updateVertex(r, c) {
    this.nodesUpdated++;
    const k = `${r},${c}`;
    const isGoal = (r === this.goal.r && c === this.goal.c);

    if (!isGoal) {
      // rhs(s) = min over successors s' of [c(s,s') + g(s')]
      // In backward search: successors of s are its neighbors
      let minRhs = Infinity;
      for (const nb of this.neighbors(r, c)) {
        const cost = this._cost(nb.r, nb.c); // cost to move into nb (when going backward)
        // Wait — in backward D*, moving FROM s TO nb means the edge cost is c(s,nb)
        // which in our grid is the cost of entering nb FROM s
        // But we search backward: "predecessor" of nb in the path to goal
        // Actually: rhs(s) = min_s'∈Succ(s) [c(s,s') + g(s')]
        // In grid: Succ(s) = neighbors of s (same as pred in undirected grid)
        // c(s,s') = cost of cell s' (cost to enter s')
        const g_nb = this._g(nb.r, nb.c);
        const c_to_nb = edgeCost(nb.r, nb.c); // cost to enter nb
        if (c_to_nb + g_nb < minRhs) minRhs = c_to_nb + g_nb;
      }
      this._rhs(r, c, minRhs);
    }

    this.U.remove(`${r},${c}`);
    const gv = this._g(r, c);
    const rv = this._rhs(r, c);
    if (gv !== rv) {
      this.U.insert(this._key(r, c), { r, c });
    }
  }

  computeShortestPath() {
    let iters = 0;
    const maxIters = GRID * GRID * 4;
    while (!this.U.isEmpty()) {
      const topKey = this.U.topKey();
      const startKey = this._key(this.start.r, this.start.c);
      const rv_start = this._rhs(this.start.r, this.start.c);
      const gv_start = this._g(this.start.r, this.start.c);

      if (compareDstarKeys(topKey, startKey) >= 0 && rv_start === gv_start) break;
      if (iters++ > maxIters) break;

      const kOld = topKey;
      const u = this.U.pop();
      const kNew = this._key(u.r, u.c);

      if (compareDstarKeys(kOld, kNew) < 0) {
        this.U.insert(kNew, u);
      } else {
        const gv = this._g(u.r, u.c);
        const rv = this._rhs(u.r, u.c);
        if (gv > rv) {
          this._g(u.r, u.c, rv);
          for (const nb of this.neighbors(u.r, u.c)) {
            this.updateVertex(nb.r, nb.c);
          }
        } else {
          this._g(u.r, u.c, Infinity);
          this.updateVertex(u.r, u.c);
          for (const nb of this.neighbors(u.r, u.c)) {
            this.updateVertex(nb.r, nb.c);
          }
        }
      }
    }
  }

  // Call when edge costs change (e.g. new gas, blocked road)
  // changedCells: array of {r, c} whose costs changed
  notifyCostChange(changedCells, newStart) {
    // Update km for heuristic shift
    this.km += heuristic(this.start.r, this.start.c, newStart.r, newStart.c);
    this.start = { ...newStart };

    for (const { r, c } of changedCells) {
      // Affected: the cell itself and its neighbors
      this.updateVertex(r, c);
      for (const nb of this.neighbors(r, c)) {
        this.updateVertex(nb.r, nb.c);
      }
    }
    this.computeShortestPath();
  }

  // Extract path from start → goal by greedily following min g(neighbor)
  extractPath() {
    const path = [];
    let cur = { ...this.start };
    const visited = new Set();
    path.push(cur);

    for (let step = 0; step < GRID * GRID; step++) {
      const key = `${cur.r},${cur.c}`;
      if (visited.has(key)) break; // cycle protection
      visited.add(key);
      if (cur.r === this.goal.r && cur.c === this.goal.c) break;

      let bestNeighbor = null;
      let bestCost = Infinity;

      for (const nb of this.neighbors(cur.r, cur.c)) {
        const moveCost = edgeCost(nb.r, nb.c);
        if (moveCost === Infinity) continue;
        const total = moveCost + this._g(nb.r, nb.c);
        if (total < bestCost) {
          bestCost = total;
          bestNeighbor = nb;
        }
      }

      if (!bestNeighbor || bestCost === Infinity) break;
      cur = bestNeighbor;
      path.push({ ...cur });
    }

    const reachedGoal = path.length > 0 && path[path.length-1].r === this.goal.r && path[path.length-1].c === this.goal.c;
    return reachedGoal ? path : null;
  }

  getPathCost() {
    return this._g(this.start.r, this.start.c);
  }

  // Full initialization (when target changes or first run)
  initialize(newGoal, newStart) {
    this.goal = { ...newGoal };
    this.start = { ...newStart };
    this.km = 0;
    this.g = new Map();
    this.rhs = new Map();
    this.U = new MinHeapDstar();
    this.nodesUpdated = 0;
    this._rhs(newGoal.r, newGoal.c, 0);
    this.U.insert(this._key(newGoal.r, newGoal.c), { r: newGoal.r, c: newGoal.c });
    this.computeShortestPath();
  }
}

function compareDstarKeys(a, b) {
  if (a[0] !== b[0]) return a[0] < b[0] ? -1 : 1;
  if (a[1] !== b[1]) return a[1] < b[1] ? -1 : 1;
  return 0;
}

// Priority queue for D* Lite keyed by [k1, k2] pairs
class MinHeapDstar {
  constructor() { this.data = []; this.keyMap = new Map(); }

  insert(key, node) {
    const id = `${node.r},${node.c}`;
    this.remove(id); // Remove existing if present
    this.data.push({ key, node, id });
    this.keyMap.set(id, this.data.length - 1);
    this._bubbleUp(this.data.length - 1);
  }

  remove(id) {
    if (!this.keyMap.has(id)) return;
    const i = this.keyMap.get(id);
    this.keyMap.delete(id);
    const last = this.data.pop();
    if (i < this.data.length) {
      this.data[i] = last;
      this.keyMap.set(last.id, i);
      this._bubbleUp(i);
      this._sinkDown(i);
    }
  }

  pop() {
    const top = this.data[0];
    this.keyMap.delete(top.id);
    const last = this.data.pop();
    if (this.data.length > 0) {
      this.data[0] = last;
      this.keyMap.set(last.id, 0);
      this._sinkDown(0);
    }
    return top.node;
  }

  topKey() { return this.data.length > 0 ? this.data[0].key : [Infinity, Infinity]; }
  isEmpty() { return this.data.length === 0; }

  _bubbleUp(i) {
    while (i > 0) {
      const p = Math.floor((i - 1) / 2);
      if (compareDstarKeys(this.data[p].key, this.data[i].key) <= 0) break;
      this._swap(p, i);
      i = p;
    }
  }
  _sinkDown(i) {
    while (true) {
      let min = i, l = 2*i+1, r = 2*i+2;
      if (l < this.data.length && compareDstarKeys(this.data[l].key, this.data[min].key) < 0) min = l;
      if (r < this.data.length && compareDstarKeys(this.data[r].key, this.data[min].key) < 0) min = r;
      if (min === i) break;
      this._swap(min, i);
      i = min;
    }
  }
  _swap(i, j) {
    [this.data[i], this.data[j]] = [this.data[j], this.data[i]];
    this.keyMap.set(this.data[i].id, i);
    this.keyMap.set(this.data[j].id, j);
  }
}

// Standard Min Heap for A*
class MinHeap {
  constructor() { this.data = []; }
  push(item) { this.data.push(item); this._bubbleUp(this.data.length - 1); }
  pop() {
    const top = this.data[0];
    const last = this.data.pop();
    if (this.data.length > 0) { this.data[0] = last; this._sinkDown(0); }
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
// UNIFIED REPLAN DISPATCH
// =============================================

// changedCells: for D* Lite incremental update (array of {r,c})
// forceFullReplan: if true, reinitialize D* Lite (e.g., target changed)
function computeReplan(changedCells = [], forceFullReplan = false) {
  updatePowerGrid();
  const horizon = parseInt(document.getElementById('predictHorizon').value);
  let predictedChanges = [];
  if (state.gasActive && horizon > 0) predictedChanges = predictGasSpread(horizon);

  const poweredHospitals = state.hospitals.filter(h => h.powered);
  if (predictedChanges.length > 0) changedCells = changedCells.concat(predictedChanges);
  if (poweredHospitals.length === 0) {
    log('⚠ NO POWERED HOSPITALS AVAILABLE', 'danger');
    state.missionFailed = true;
    return false;
  }

  if (activeAlgorithm === 'astar') {
    return computeReplanAstar(poweredHospitals);
  } else {
    return computeReplanDstar(poweredHospitals, changedCells, forceFullReplan);
  }
}

// --- A* Replan ---
function computeReplanAstar(poweredHospitals) {
  const t0 = performance.now();
  let bestPlan = null;
  let totalNodes = 0;

  for (const h of poweredHospitals) {
    const result = astarFindPath(state.vehicle.r, state.vehicle.c, h);
    if (!result) continue;
    totalNodes += result.nodesExpanded;
    const safetyPenalty = Math.max(0, 6 - distanceToNearestGas(h.r, h.c)) * 7;
    const score = result.cost + safetyPenalty;
    if (!bestPlan || score < bestPlan.score) {
      bestPlan = { hospital: h, path: result.path, cost: result.cost, score };
    }
  }

  state.nodesExpanded += totalNodes;
  state.replanTimes.push(performance.now() - t0);

  if (!bestPlan) {
    log('⚠ A*: NO VALID PATH FOUND', 'danger');
    state.missionFailed = true;
    return false;
  }

  const oldTarget = state.targetHospital;
  state.path = bestPlan.path;
  state.pathIndex = 0;
  state.targetHospital = bestPlan.hospital;
  state.ourCost = bestPlan.cost;
  state.nodesUpdated = totalNodes; // A* expands = updates

  // Naive comparison
  const naive = astarNaive(state.vehicle.r, state.vehicle.c, bestPlan.hospital);
  if (naive) state.naiveCost = naive.cost;

  let avoided = 0;
  for (const node of state.path)
    if (state.grid[node.r][node.c].gasPredicted) avoided++;
  state.pathsAvoided = avoided;

  if (state.replans > 0 && oldTarget?.id !== state.targetHospital?.id)
    log(`⚡ A*: TARGET CHANGED → Hospital #${state.targetHospital.id}`, 'warn');

  return true;
}

// --- D* Lite Replan ---
function computeReplanDstar(poweredHospitals, changedCells, forceFullReplan) {
  const t0 = performance.now();

  const hospitalCandidates = [...poweredHospitals]
    .map(h => {
      const dist = heuristic(state.vehicle.r, state.vehicle.c, h.r, h.c);
      const safetyPenalty = Math.max(0, 6 - distanceToNearestGas(h.r, h.c)) * 7;
      return { hospital: h, score: dist + safetyPenalty };
    })
    .sort((a, b) => a.score - b.score);

  let chosenHospital = null;
  let chosenPath = null;

  for (const candidate of hospitalCandidates) {
    const h = candidate.hospital;
    const targetChanged = !state.targetHospital || state.targetHospital.id !== h.id;
    const needFullInit = !dstarPlanner || forceFullReplan || targetChanged;

    if (needFullInit) {
      dstarPlanner = new DStarLite(h, { r: state.vehicle.r, c: state.vehicle.c });
      dstarPlanner.computeShortestPath();
      if (targetChanged && state.replans > 0)
        log(`⚡ D*: TARGET CHANGED → Hospital #${h.id}`, 'dstar');
    } else {
      if (changedCells.length > 0) {
        dstarPlanner.notifyCostChange(changedCells, { r: state.vehicle.r, c: state.vehicle.c });
      } else {
        const affectedCells = [];
        for (let r = 0; r < GRID; r++)
          for (let c = 0; c < GRID; c++)
            if (state.grid[r][c].gas || state.grid[r][c].gasPredicted)
              affectedCells.push({ r, c });
        if (affectedCells.length > 0)
          dstarPlanner.notifyCostChange(affectedCells, { r: state.vehicle.r, c: state.vehicle.c });
        else
          dstarPlanner.start = { r: state.vehicle.r, c: state.vehicle.c };
      }
    }

    const path = dstarPlanner.extractPath();
    if (path) {
      chosenHospital = h;
      chosenPath = path;
      break;
    }
  }

  state.nodesUpdated = dstarPlanner?.nodesUpdated ?? 0;
  if (dstarPlanner) dstarPlanner.nodesUpdated = 0;

  if (!chosenPath) {
    log('⚠ D*: NO VALID PATH FOUND — all routes blocked!', 'danger');
    state.missionFailed = true;
    return false;
  }

  state.path = chosenPath;
  state.pathIndex = 0;
  state.targetHospital = chosenHospital;
  state.ourCost = dstarPlanner.getPathCost();
  state.nodesExpanded += chosenPath.length;

  const naive = astarNaive(state.vehicle.r, state.vehicle.c, chosenHospital);
  if (naive) state.naiveCost = naive.cost;

  let avoided = 0;
  for (const node of state.path)
    if (state.grid[node.r][node.c].gasPredicted) avoided++;
  state.pathsAvoided = avoided;

  return true;
}

// =============================================
// GAS STEP (spread per tick)
// =============================================
function stepGasSpread() {
  const spreadRate = parseInt(document.getElementById('spreadRate').value);
  const newGas = [];

  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      if (state.grid[r][c].gas) {
        state.grid[r][c].gasAge++;
        for (const [dr, dc] of [[0,1],[0,-1],[1,0],[-1,0],[1,1],[-1,-1],[1,-1],[-1,1]]) {
          const nr = r + dr, nc = c + dc;
          if (nr >= 0 && nr < GRID && nc >= 0 && nc < GRID && !state.grid[nr][nc].blocked && !state.grid[nr][nc].gas) {
            const prob = (Math.abs(dr) + Math.abs(dc) === 1 ? 0.35 : 0.12) * (spreadRate / 2);
            if (Math.random() < prob) newGas.push([nr, nc]);
          }
        }
      }
    }
  }

  const newGasCells = [];
  for (const [r, c] of newGas) {
    if (!state.grid[r][c].gas) {
      state.grid[r][c].gas = true;
      state.grid[r][c].cost = 50;
      newGasCells.push({ r, c });
    }
  }

  if (newGasCells.length > 0)
    log(`☣ Gas spread: +${newGasCells.length} cells`, 'warn');

  return newGasCells;
}

// =============================================
// SIMULATION TICK
// =============================================
function simTick() {
  if (state.missionDone || state.missionFailed) { stopSim(); return; }

  tick++;
  document.getElementById('tickLabel').textContent = `TICK: ${tick}`;

  let changedCells = [];

  // Every 3 ticks: spread gas and replan
  if (state.gasActive && tick % 3 === 0) {
    const newCells = stepGasSpread();
    changedCells = newCells;

    for (const sub of state.substations) {
      if (state.grid[sub.r][sub.c].gas && sub.active) {
        sub.active = false;
        log(`⚡ Substation #${sub.id} DISABLED by gas!`, 'danger');
        changedCells.push({ r: sub.r, c: sub.c });
      }
    }

    state.replans++;
    computeReplan(changedCells, false);

    const label = activeAlgorithm === 'dstar' ? 'D*' : 'A*';
    log(`↺ REPLAN #${state.replans} [${label}] → Hospital #${state.targetHospital?.id}`, 'warn');
  }

  // Move vehicle one step
  if (state.path.length > 1 && state.pathIndex < state.path.length - 1) {
    state.pathIndex++;
    const next = state.path[state.pathIndex];
    state.vehicle.r = next.r;
    state.vehicle.c = next.c;
    state.steps++;

    if (state.grid[next.r][next.c].gas) {
      log(`⚠ Vehicle in gas zone! Emergency replan...`, 'danger');
      state.replans++;
      computeReplan([{ r: next.r, c: next.c }], false);
    }

    // Update D* Lite start position incrementally (vehicle moved)
    if (activeAlgorithm === 'dstar' && dstarPlanner) {
      dstarPlanner.start = { r: state.vehicle.r, c: state.vehicle.c };
    }

    if (state.targetHospital && state.vehicle.r === state.targetHospital.r && state.vehicle.c === state.targetHospital.c) {
      state.missionDone = true;
      const savings = state.naiveCost ? (state.naiveCost - state.ourCost).toFixed(1) : '—';
      const label = activeAlgorithm === 'dstar' ? 'D* Lite' : 'Predictive A*';
      log(`✓ MISSION SUCCESS! [${label}] Reached Hospital #${state.targetHospital.id}`, 'good');
      log(`✓ Path cost: ${state.ourCost?.toFixed(1)} | Replans: ${state.replans} | Steps: ${state.steps}`, 'good');
      log(`✓ Savings vs Naive A*: ${savings}`, 'good');
      document.getElementById('systemStatus').textContent = '✓ MISSION COMPLETE';
      document.getElementById('systemStatus').className = 'status-badge badge-active';
      document.getElementById('phaseLabel').textContent = 'PHASE: SUCCESS';
      stopSim(); return;
    }
  }

  // Activate gas at tick 5
  if (!state.gasActive && tick === 5) {
    state.gasActive = true;
    const activated = [];
    for (const o of state.gasOrigins) {
      state.grid[o.r][o.c].gas = true;
      state.grid[o.r][o.c].cost = 50;
      activated.push({ r: o.r, c: o.c });
    }
    log(`☣ GAS LEAK at (${state.gasOrigins[0]?.r},${state.gasOrigins[0]?.c})!`, 'danger');
    log(`⟳ Initiating predictive rerouting [${activeAlgorithm === 'dstar' ? 'D* Lite' : 'A*'}]...`, 'info');
    state.replans++;
    computeReplan(activated, true);
    document.getElementById('hazardStatus').textContent = '⚠ GAS LEAK ACTIVE';
    document.getElementById('hazardStatus').className = 'status-badge badge-danger pulse';
  }

  updateStats();
  render();
}

// =============================================
// RENDERING
// =============================================
function render() {
  resizeCanvas();
  ctx.fillStyle = C.bg;
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  CELL = Math.floor(Math.min(canvas.width, canvas.height) / GRID);
  const offsetX = Math.floor((canvas.width - CELL * GRID) / 2);
  const offsetY = Math.floor((canvas.height - CELL * GRID) / 2);

  // Grid cells
  for (let r = 0; r < GRID; r++) {
    for (let c = 0; c < GRID; c++) {
      const x = offsetX + c * CELL, y = offsetY + r * CELL;
      const cell = state.grid[r][c];

      if (cell.blocked) {
        ctx.fillStyle = '#0a0a0a';
        ctx.fillRect(x, y, CELL, CELL);
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

      if (cell.gas) {
        const age = Math.min(cell.gasAge / 5, 1);
        ctx.fillStyle = `rgba(57,255,20,${0.25 + age * 0.25})`;
        ctx.fillRect(x, y, CELL, CELL);
        if (CELL > 20) {
          ctx.fillStyle = `rgba(57,255,20,0.6)`;
          for (let i = 0; i < 2; i++) {
            ctx.fillRect(x + 2 + Math.random() * (CELL-4), y + 2 + Math.random() * (CELL-4), 1.5, 1.5);
          }
        }
      }

      if (cell.gasPredicted && !cell.gas) {
        ctx.fillStyle = 'rgba(57,255,20,0.07)';
        ctx.fillRect(x, y, CELL, CELL);
        ctx.strokeStyle = 'rgba(57,255,20,0.3)';
        ctx.lineWidth = 0.5;
        ctx.setLineDash([2, 3]);
        ctx.strokeRect(x+1, y+1, CELL-2, CELL-2);
        ctx.setLineDash([]);
      }

      // D* Lite: show g-values as subtle heatmap
      if (activeAlgorithm === 'dstar' && dstarPlanner && !cell.blocked && !cell.gas && CELL > 20) {
        const gv = dstarPlanner._g(r, c);
        if (gv !== Infinity && gv > 0) {
          const maxG = GRID * 3;
          const alpha = Math.min(gv / maxG, 1) * 0.08;
          ctx.fillStyle = `rgba(255,107,53,${alpha})`;
          ctx.fillRect(x, y, CELL, CELL);
        }
      }

      ctx.strokeStyle = 'rgba(26,58,92,0.5)';
      ctx.lineWidth = 0.5;
      ctx.strokeRect(x, y, CELL, CELL);
    }
  }

  // Path
  const pathColor = activeAlgorithm === 'dstar' ? C.pathDstar : C.pathAstar;
  if (state.path.length > 0) {
    ctx.beginPath();
    let started = false;
    for (let i = state.pathIndex; i < state.path.length; i++) {
      const { r, c } = state.path[i];
      const x = offsetX + c * CELL + CELL/2;
      const y = offsetY + r * CELL + CELL/2;
      if (!started) { ctx.moveTo(x, y); started = true; }
      else ctx.lineTo(x, y);
    }
    ctx.strokeStyle = `${pathColor}0.55)`;
    ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.stroke();
    ctx.setLineDash([]);

    for (let i = state.pathIndex; i < state.path.length; i++) {
      const { r, c } = state.path[i];
      const x = offsetX + c * CELL + CELL/2;
      const y = offsetY + r * CELL + CELL/2;
      const alpha = 0.2 + 0.5 * (i / state.path.length);
      ctx.beginPath();
      ctx.arc(x, y, 2, 0, Math.PI * 2);
      ctx.fillStyle = `${pathColor}${alpha})`;
      ctx.fill();
    }
  }

  // Power grid lines
  ctx.save();
  ctx.globalAlpha = 0.25;
  for (const edge of state.powerEdges) {
    const active = edge.type === 'h-s' ? edge.to.active : true;
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

  // Substations
  for (const sub of state.substations) {
    const x = offsetX + sub.c * CELL + CELL/2;
    const y = offsetY + sub.r * CELL + CELL/2;
    const sz = CELL * 0.35;
    ctx.fillStyle = sub.active ? C.substation : C.substationOff;
    ctx.strokeStyle = sub.active ? '#ffd60a' : '#4a3000';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(x, y-sz); ctx.lineTo(x+sz, y); ctx.lineTo(x, y+sz); ctx.lineTo(x-sz, y);
    ctx.closePath(); ctx.fill(); ctx.stroke();
    if (sub.active && CELL > 20) {
      ctx.fillStyle = 'rgba(255,214,10,0.15)';
      ctx.beginPath(); ctx.arc(x, y, sz*2, 0, Math.PI*2); ctx.fill();
    }
  }

  // Power plant
  const pp = state.powerPlant;
  if (pp) {
    const x = offsetX + pp.c * CELL + CELL/2;
    const y = offsetY + pp.r * CELL + CELL/2;
    const sz = CELL * 0.42;
    const grad = ctx.createRadialGradient(x, y, 0, x, y, sz*2.5);
    grad.addColorStop(0, 'rgba(191,90,242,0.3)');
    grad.addColorStop(1, 'rgba(191,90,242,0)');
    ctx.fillStyle = grad;
    ctx.beginPath(); ctx.arc(x, y, sz*2.5, 0, Math.PI*2); ctx.fill();
    ctx.fillStyle = C.powerPlant;
    ctx.strokeStyle = '#d090ff'; ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(x, y-sz);
    for (let i = 1; i <= 6; i++) {
      const angle = (i * Math.PI*2/6) - Math.PI/2;
      ctx.lineTo(x + sz*Math.cos(angle), y + sz*Math.sin(angle));
    }
    ctx.closePath(); ctx.fill(); ctx.stroke();
    if (CELL > 18) {
      ctx.fillStyle = 'white';
      ctx.font = `bold ${Math.max(8, CELL*0.3)}px Orbitron`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('⚡', x, y);
    }
  }

  // Hospitals
  for (const h of state.hospitals) {
    const x = offsetX + h.c * CELL + CELL/2;
    const y = offsetY + h.r * CELL + CELL/2;
    const sz = CELL * 0.42;
    const isTarget = state.targetHospital?.id === h.id;

    if (isTarget && h.powered) {
      const grad = ctx.createRadialGradient(x, y, 0, x, y, sz*3);
      grad.addColorStop(0, activeAlgorithm === 'dstar' ? 'rgba(255,107,53,0.25)' : 'rgba(0,229,255,0.25)');
      grad.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = grad;
      ctx.beginPath(); ctx.arc(x, y, sz*3, 0, Math.PI*2); ctx.fill();
      const pulse = 0.6 + 0.4 * Math.sin(tick*0.3);
      ctx.strokeStyle = activeAlgorithm === 'dstar' ? `rgba(255,107,53,${pulse})` : `rgba(0,229,255,${pulse})`;
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(x, y, sz*2.2, 0, Math.PI*2); ctx.stroke();
    }

    ctx.fillStyle = h.powered ? C.hospitalOn : C.hospitalOff;
    ctx.strokeStyle = h.powered ? '#00ff88' : '#ff2d55';
    ctx.lineWidth = isTarget ? 2 : 1;
    ctx.beginPath();
    ctx.roundRect(x-sz, y-sz, sz*2, sz*2, 3);
    ctx.fill(); ctx.stroke();

    if (CELL > 14) {
      ctx.fillStyle = 'rgba(0,0,0,0.6)';
      const cw = sz*0.35, ch = sz*1.2;
      ctx.fillRect(x-cw/2, y-ch/2, cw, ch);
      ctx.fillRect(x-ch/2, y-cw/2, ch, cw);
      ctx.fillStyle = 'white';
      ctx.font = `bold ${Math.max(7, CELL*0.28)}px Exo 2`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText('H', x, y);
    }
    if (CELL > 18) {
      ctx.fillStyle = h.powered ? C.hospitalOn : C.hospitalOff;
      ctx.font = `${Math.max(7, CELL*0.22)}px Share Tech Mono`;
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      ctx.fillText(`#${h.id}`, x, y+sz+2);
    }
  }

  // Gas origin markers
  for (const o of state.gasOrigins) {
    if (state.gasActive) {
      const x = offsetX + o.c * CELL + CELL/2;
      const y = offsetY + o.r * CELL + CELL/2;
      ctx.strokeStyle = 'rgba(57,255,20,0.8)'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(x, y, CELL*0.45, 0, Math.PI*2); ctx.stroke();
    }
  }

  // Vehicle
  const vx = offsetX + state.vehicle.c * CELL + CELL/2;
  const vy = offsetY + state.vehicle.r * CELL + CELL/2;
  const vsz = CELL * 0.38;
  const vGrad = ctx.createRadialGradient(vx, vy, 0, vx, vy, vsz*3);
  vGrad.addColorStop(0, 'rgba(0,229,255,0.4)');
  vGrad.addColorStop(1, 'rgba(0,229,255,0)');
  ctx.fillStyle = vGrad;
  ctx.beginPath(); ctx.arc(vx, vy, vsz*3, 0, Math.PI*2); ctx.fill();
  ctx.fillStyle = C.vehicle;
  ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.arc(vx, vy, vsz, 0, Math.PI*2); ctx.fill(); ctx.stroke();
  if (CELL > 14) {
    ctx.fillStyle = '#060a0f';
    ctx.font = `bold ${Math.max(8, CELL*0.32)}px Arial`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('🚑', vx, vy);
  }

  // Mission done/failed overlay
  if (state.missionDone || state.missionFailed) {
    ctx.fillStyle = state.missionDone ? 'rgba(0,255,136,0.12)' : 'rgba(255,45,85,0.12)';
    ctx.fillRect(offsetX, offsetY, GRID*CELL, GRID*CELL);
    ctx.fillStyle = state.missionDone ? C.hospitalOn : C.hospitalOff;
    ctx.font = `bold ${CELL*1.2}px Orbitron`;
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(state.missionDone ? '✓' : '✗', offsetX+GRID*CELL/2, offsetY+GRID*CELL/2);
  }
}

function resizeCanvas() {
  const area = canvas.parentElement;
  const rect = area.getBoundingClientRect();
  const toolbar = document.querySelector('.toolbar');
  const th = toolbar ? toolbar.getBoundingClientRect().height : 0;
  const nw = rect.width, nh = rect.height - th;
  if (canvas.width !== nw || canvas.height !== nh) { canvas.width = nw; canvas.height = nh; }
}

// =============================================
// STATS UI
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
  document.getElementById('statGridHealth').textContent = `${Math.round((activeSubs/state.substations.length)*100)}%`;

  if (state.replanTimes.length > 0) {
    const avg = state.replanTimes.reduce((a,b)=>a+b,0)/state.replanTimes.length;
    document.getElementById('statReplanTime').textContent = `${avg.toFixed(2)}ms`;
  }
  document.getElementById('statHeuristic').textContent = 'Admissible ✓';

  // D* Lite specific
  if (activeAlgorithm === 'dstar') {
    document.getElementById('replanStrategy').textContent = 'Incremental repair';
    document.getElementById('statNodesUpdated').textContent = state.nodesUpdated || '—';
  } else {
    document.getElementById('replanStrategy').textContent = 'Full re-run';
    document.getElementById('statNodesUpdated').textContent = state.nodesExpanded || '—';
  }

  // Progress
  if (state.path.length > 1) {
    const pct = Math.round((state.pathIndex/(state.path.length-1))*100);
    document.getElementById('progressBar').style.width = `${pct}%`;
    document.getElementById('progressLabel').textContent =
      `Step ${state.pathIndex}/${state.path.length-1} → Hospital #${state.targetHospital?.id||'?'}`;
  }

  // Comparison
  if (state.naiveCost && state.ourCost) {
    document.getElementById('cmpOurs').textContent = state.ourCost.toFixed(1);
    document.getElementById('cmpNaive').textContent = state.naiveCost.toFixed(1);
    const savings = (state.naiveCost - state.ourCost).toFixed(1);
    document.getElementById('cmpSavings').textContent =
      savings > 0 ? `${savings} (${Math.round(savings/state.naiveCost*100)}%)` : '0';
  }

  updateHospitalList();
}

function updateHospitalList() {
  const list = document.getElementById('hospitalList');
  list.innerHTML = '';
  for (const h of state.hospitals) {
    const isTarget = state.targetHospital?.id === h.id;
    const cls = isTarget ? 'target' : h.powered ? 'powered' : 'unpowered';
    const dist = Math.abs(h.r-state.vehicle.r) + Math.abs(h.c-state.vehicle.c);
    list.innerHTML += `
      <div class="hospital-item ${cls}">
        <div class="h-dot" style="background:${h.powered?'var(--green)':'var(--red)'}"></div>
        <div style="flex:1">
          <div style="color:${isTarget?'var(--accent)':h.powered?'var(--green)':'var(--red)'}">
            Hospital #${h.id} ${isTarget?'← TARGET':''}
          </div>
          <div style="color:var(--text-dim);font-size:9px">
            (${h.r},${h.c}) · dist: ${dist} · ${h.powered?'POWERED':'OFFLINE'}
          </div>
        </div>
      </div>`;
  }
}

function log(msg, type='info') {
  const box = document.getElementById('logBox');
  const t = String(tick).padStart(3,'0');
  box.innerHTML += `<div class="log-entry ${type}">[${t}] ${msg}</div>`;
  box.scrollTop = box.scrollHeight;
}

function switchTab(name) {
  document.querySelectorAll('.tab').forEach((t, i) => {
    t.classList.toggle('active', ['stats','hospitals','log'][i] === name);
  });
  document.querySelectorAll('.tab-content').forEach(tc => {
    tc.classList.toggle('active', tc.id === `tab-${name}`);
  });
}

// =============================================
// ALGORITHM TOGGLE
// =============================================
function setAlgorithm(algo) {
  activeAlgorithm = algo;
  dstarPlanner = null; // Reset D* state on switch

  document.getElementById('algoBtnAstar').classList.toggle('active', algo === 'astar');
  document.getElementById('algoBtnDstar').classList.toggle('active', algo === 'dstar');
  document.getElementById('algoFormulaAstar').style.display = algo === 'astar' ? 'block' : 'none';
  document.getElementById('algoFormulaDstar').style.display = algo === 'dstar' ? 'block' : 'none';

  // Update badge
  const badge = document.getElementById('algoBadgeDisplay');
  const compareCard = document.querySelector('.algo-compare-card');
  if (algo === 'dstar') {
    badge.innerHTML = '<span class="algo-badge dstar-badge">D* LITE</span>';
    compareCard.classList.add('dstar-mode');
    document.getElementById('replanStrategy').textContent = 'Incremental repair';
  } else {
    badge.innerHTML = '<span class="algo-badge astar-badge">A* PREDICTIVE</span>';
    compareCard.classList.remove('dstar-mode');
    document.getElementById('replanStrategy').textContent = 'Full re-run';
  }

  // Recompute path with new algorithm
  computeReplan([], true);
  render();

  log(`⚙ Algorithm switched to: ${algo === 'dstar' ? 'D* Lite (incremental)' : 'Predictive A* (full re-run)'}`, algo === 'dstar' ? 'dstar' : 'info');
}

// =============================================
// CONTROL FUNCTIONS
// =============================================
function generateCity() {
  stopSim();
  tick = 0;
  dstarPlanner = null;
  state = initState();
  initialState = JSON.parse(JSON.stringify(state));
  preLaunchState = null;
  updatePowerGrid();
  computeReplan([], true);
  updateStats();
  render();
  log('► New city generated. Ready for mission.', 'info');
  log(`► Algorithm: ${activeAlgorithm === 'dstar' ? 'D* Lite' : 'Predictive A*'}`, 'dim');
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
  preLaunchState = JSON.parse(JSON.stringify(state));
  if (!state.path || state.path.length === 0) computeReplan([], true);
  simRunning = true;
  document.getElementById('btnStart').textContent = '⏸ PAUSE';
  document.getElementById('btnStep').disabled = false;
  document.getElementById('systemStatus').textContent = '● MISSION ACTIVE';
  document.getElementById('phaseLabel').textContent = 'PHASE: NAVIGATE';
  log(`► Mission launched [${activeAlgorithm === 'dstar' ? 'D* Lite' : 'Predictive A*'}]`, 'info');

  const speed = parseInt(document.getElementById('speedSlider').value);
  simInterval = setInterval(simTick, Math.round(1200/speed));
}

function stopSim() {
  simRunning = false;
  if (simInterval) { clearInterval(simInterval); simInterval = null; }
  document.getElementById('btnStart').textContent = '▶ LAUNCH MISSION';
}

function stepSim() { simTick(); render(); }

function resetSim() {
  stopSim();
  tick = 0;
  dstarPlanner = null;
  const resetState = preLaunchState || initialState;
  state = resetState ? JSON.parse(JSON.stringify(resetState)) : initState();
  if (state.hospitals && state.substations && state.powerPlant)
    state.powerEdges = buildPowerGrid(state.hospitals, state.substations, state.powerPlant);
  updatePowerGrid();
  computeReplan([], true);
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
  selectedHospital = null;
  selectedSubstation = null;
  ['modeView','modeBlock','modeGas','modeAmbulance','modeHospital','modeSubstation'].forEach(id => {
    const btn = document.getElementById(id);
    if (btn) { btn.style.borderColor=''; btn.style.color=''; btn.style.background=''; }
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
  const offsetX = Math.floor((canvas.width - CELL2*GRID) / 2);
  const offsetY = Math.floor((canvas.height - CELL2*GRID) / 2);
  const c = Math.floor((mx-offsetX)/CELL2);
  const r = Math.floor((my-offsetY)/CELL2);
  if (r < 0 || r >= GRID || c < 0 || c >= GRID) return;

  const cell = state.grid[r][c];
  let changed = [];

  if (mode === 'block') {
    cell.blocked = !cell.blocked;
    cell.gas = false;
    changed = [{ r, c }];
    computeReplan(changed, false);
    log(`✎ Road ${cell.blocked?'BLOCKED':'OPENED'} at (${r},${c})`, 'warn');
  } else if (mode === 'gas') {
    cell.gas = true; cell.cost = 50;
    state.gasActive = true;
    changed = [{ r, c }];
    document.getElementById('hazardStatus').textContent = '⚠ GAS LEAK ACTIVE';
    document.getElementById('hazardStatus').className = 'status-badge badge-danger pulse';
    computeReplan(changed, false);
    log(`☣ Gas placed at (${r},${c})`, 'danger');
  } else if (mode === 'ambulance') {
    if (!cell.blocked && !cell.gas) {
      state.vehicle.r = r; state.vehicle.c = c;
      if (activeAlgorithm === 'dstar' && dstarPlanner) dstarPlanner.start = { r, c };
      computeReplan([], false);
      log(`🚑 Ambulance moved to (${r},${c})`, 'info');
    } else log('🚑 Invalid location.', 'warn');
  } else if (mode === 'hospital') {
    const existing = state.hospitals.find(h => h.r===r && h.c===c);
    if (selectedHospital) {
      if (!cell.blocked && !cell.gas) {
        selectedHospital.r = r; selectedHospital.c = c;
        selectedHospital = null;
        state.powerEdges = buildPowerGrid(state.hospitals, state.substations, state.powerPlant);
        computeReplan([], true);
        log(`🏥 Hospital moved to (${r},${c})`, 'info');
      } else log('🏥 Cannot place on blocked/gas.', 'warn');
    } else if (existing) {
      selectedHospital = existing;
      log(`🏥 Hospital #${existing.id} selected. Click target cell.`, 'info');
    }
  } else if (mode === 'substation') {
    const existing = state.substations.find(s => s.r===r && s.c===c);
    if (selectedSubstation) {
      if (!cell.blocked && !cell.gas) {
        selectedSubstation.r = r; selectedSubstation.c = c;
        selectedSubstation = null;
        state.powerEdges = buildPowerGrid(state.hospitals, state.substations, state.powerPlant);
        computeReplan([], false);
        log(`⚡ Substation moved to (${r},${c})`, 'info');
      } else log('⚡ Cannot place on blocked/gas.', 'warn');
    } else if (existing) {
      selectedSubstation = existing;
      log(`⚡ Substation #${existing.id} selected. Click target cell.`, 'info');
    }
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
    simInterval = setInterval(simTick, Math.round(1200/parseInt(this.value)));
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
  requestAnimationFrame(renderLoop);
}

// INIT
window.addEventListener('resize', () => { resizeCanvas(); render(); });
generateCity();
renderLoop();
log('► System initialized. Both algorithms ready.', 'info');
log('► A* = full replan each time. D* Lite = incremental repair only.', 'dim');
log('► Toggle algorithm using the A* / D* buttons in the left panel.', 'dim');
log('► Gas leak activates at tick 5. Watch the replan behavior differ!', 'dim');
