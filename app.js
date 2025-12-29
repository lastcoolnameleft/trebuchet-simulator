// Main application logic - handles UI interactions and coordinates the simulation

let simulator;

// Initialize the application
document.addEventListener('DOMContentLoaded', () => {
    const canvas = document.getElementById('simulationCanvas');
    simulator = new TrebuchetSimulator(canvas);
    
    // Build initial trebuchet
    simulator.buildTrebuchet('hinged', simulator.getDefaultParameters());
    
    // Setup UI event listeners
    setupTrebuchetTypeButtons();
    setupParameterControls();
    setupSimulationControls();
    setupStatsUpdater();
});

// Handle window resize
window.addEventListener('resize', () => {
    if (simulator) {
        const canvas = document.getElementById('simulationCanvas');
        const rect = canvas.getBoundingClientRect();
        simulator.render.canvas.width = rect.width;
        simulator.render.canvas.height = rect.height;
        simulator.render.options.width = rect.width;
        simulator.render.options.height = rect.height;
        simulator.reset();
    }
});

// Setup trebuchet type selection buttons
function setupTrebuchetTypeButtons() {
    const typeButtons = document.querySelectorAll('.type-btn');
    
    typeButtons.forEach(button => {
        button.addEventListener('click', () => {
            // Update active state
            typeButtons.forEach(btn => btn.classList.remove('active'));
            button.classList.add('active');
            
            // Build new trebuchet type
            const type = button.getAttribute('data-type');
            simulator.buildTrebuchet(type, simulator.parameters);
            
            // Update button states
            updateButtonStates();
        });
    });
}

// Setup parameter controls (sliders)
function setupParameterControls() {
    const parameters = [
        'armLength',
        'counterweightMass',
        'counterweightSize',
        'projectileMass',
        'projectileSize',
        'slingLength',
        'armMass',
        'releaseAngle'
    ];
    
    parameters.forEach(param => {
        const slider = document.getElementById(param);
        const valueDisplay = document.getElementById(param + 'Value');
        
        if (slider && valueDisplay) {
            // Update display when slider changes
            slider.addEventListener('input', (e) => {
                const value = e.target.value;
                valueDisplay.textContent = value;
            });
            
            // Update trebuchet when slider is released
            slider.addEventListener('change', (e) => {
                const value = e.target.value;
                simulator.updateParameter(param, value);
                updateButtonStates();
            });
            
            // Initialize display
            valueDisplay.textContent = slider.value;
        }
    });
}

// Setup simulation control buttons
function setupSimulationControls() {
    const fireBtn = document.getElementById('fireBtn');
    const resetBtn = document.getElementById('resetBtn');
    const pauseBtn = document.getElementById('pauseBtn');
    
    fireBtn.addEventListener('click', () => {
        simulator.fire();
        updateButtonStates();
    });
    
    resetBtn.addEventListener('click', () => {
        simulator.reset();
        updateButtonStates();
        pauseBtn.textContent = 'Pause';
        updateStats({ distance: '0', maxHeight: '0', time: '0' });
    });
    
    pauseBtn.addEventListener('click', () => {
        const isPaused = simulator.pause();
        pauseBtn.textContent = isPaused ? 'Resume' : 'Pause';
    });
}

// Update button states based on simulation state
function updateButtonStates() {
    const fireBtn = document.getElementById('fireBtn');
    const resetBtn = document.getElementById('resetBtn');
    
    if (simulator.fired) {
        fireBtn.disabled = true;
        resetBtn.disabled = false;
    } else {
        fireBtn.disabled = false;
        resetBtn.disabled = false;
    }
}

// Setup stats updater
function setupStatsUpdater() {
    setInterval(() => {
        if (simulator && simulator.fired) {
            const stats = simulator.getStats();
            updateStats(stats);
        }
    }, 100); // Update stats 10 times per second
}

// Update stats display
function updateStats(stats) {
    document.getElementById('distanceValue').textContent = stats.distance + ' m';
    document.getElementById('heightValue').textContent = stats.maxHeight + ' m';
    document.getElementById('timeValue').textContent = stats.time + ' s';
}

// Utility function to format numbers
function formatNumber(num, decimals = 2) {
    return parseFloat(num).toFixed(decimals);
}
