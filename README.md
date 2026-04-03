# Adaptive Routing AI — Smart City Emergency Navigation

A full interactive simulation demonstrating Predictive D* Lite with gas spread prediction and dual-graph systems (road network + power grid).

## Features

🧠 **Algorithm**: Predictive D* Lite
- Cost function: f(n) = g(n) + h(n) + p(n)
- g(n): actual path cost (with hazard penalties)
- h(n): Manhattan distance heuristic to nearest powered hospital
- p(n): predictive penalty for cells gas will reach in the future

🔥 **Key Features**
- Dual-graph system: road network + power grid running simultaneously
- Substations go offline when gas reaches them, cutting power to hospitals
- Predictive gas spread: AI forecasts gas movement and avoids future danger zones
- Dynamic replanning: path recomputed instantly when environment changes
- Comparison vs Naive A*: shows cost savings over non-predictive A*

🎮 **How to Use**
1. Click **LAUNCH MISSION** — gas spawns at tick 5 and vehicle starts navigating
2. Use **BLOCK ROAD** mode + click cells to create manual blockages
3. Use **PLACE GAS** mode + click to manually seed hazard zones
4. Adjust **Speed**, **Spread Rate**, and **Predict Horizon** sliders
5. Watch the **Stats** panel for replans, path cost savings, and power grid health

## File Structure

- `index.html` - Main HTML structure
- `styles.css` - All CSS styling (dark ops/tactical aesthetic)
- `script.js` - Complete JavaScript engine (simulation, rendering, AI)

## Demo
![Project Demo](./path-to-your-demo.gif)

## How to Run

### Option 1: Direct Browser Opening
1. Open `index.html` in any modern web browser (Chrome, Firefox, Safari, Edge)
2. The simulation will load immediately

### Option 2: Local Web Server (Recommended)
For better performance and to avoid browser security restrictions:

```bash
# Using Python (if installed)
python -m http.server 8000

# Or using Node.js (if installed)
npx http-server

# Or using PHP (if installed)
php -S localhost:8000

# Then open http://localhost:8000 in your browser
```

### Option 3: VS Code Live Server Extension
1. Install the "Live Server" extension in VS Code
2. Right-click `index.html` and select "Open with Live Server"

## Controls

- **LAUNCH MISSION**: Start the simulation
- **STEP**: Advance one tick at a time
- **RESET**: Reset to initial state
- **NEW CITY MAP**: Generate a new random city layout
- **VIEW/BLOCK ROAD/PLACE GAS**: Interaction modes for the grid
- **Speed Slider**: Control simulation speed (1x to 10x)
- **Spread Rate**: How fast gas spreads (1-5)
- **Predict Horizon**: How many ticks ahead the AI predicts (1-8)

## Technical Details

- **Grid Sizes**: 12×12 (Small), 16×16 (Medium), 20×20 (Large)
- **Gas Spread**: Probabilistic model with directional bias
- **Power Grid**: Hospitals connect to 2 nearest substations each
- **Pathfinding**: A* with predictive penalties (D* Lite inspired)
- **Rendering**: HTML5 Canvas with real-time updates

## Browser Compatibility

Works in all modern browsers with HTML5 Canvas support:
- Chrome 60+
- Firefox 55+
- Safari 12+
- Edge 79+

## Performance

- Optimized for 60 FPS rendering
- Efficient A* implementation with min-heap
- Real-time gas spread simulation
- Responsive UI with CSS Grid layout

## Educational Value

This simulation demonstrates:
- Advanced pathfinding algorithms (A* variants)
- Predictive AI and lookahead planning
- Multi-agent systems (roads + power grid)
- Real-time simulation techniques
- Interactive data visualization
- Algorithm performance analysis

Perfect for AI/CS courses studying search algorithms, planning, and multi-agent systems.