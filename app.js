// Main application logic - handles UI interactions and coordinates the simulation

let simulator;

// Initialize the application
document.addEventListener('DOMContentLoaded', () => {
    const canvas = document.getElementById('simulationCanvas');
    simulator = new TrebuchetSimulator(canvas);
    
    // Populate parameter inputs with default values
    const defaultParams = simulator.getDefaultParameters();
    Object.keys(defaultParams).forEach(param => {
        const input = document.getElementById(param);
        if (input) {
            input.value = defaultParams[param];
        }
    });
    
    // Build initial trebuchet
    simulator.buildTrebuchet('sandbox', simulator.getDefaultParameters());
    
    // Setup UI event listeners
    setupTrebuchetTypeButtons();
    setupParameterControls();
    setupSimulationControls();
    setupKeyboardShortcuts();
    setupStatsUpdater();
    
    // Set initial pause button text since we start paused
    document.getElementById('pauseBtn').textContent = 'Play';
});

// Handle window resize
window.addEventListener('resize', () => {
    if (simulator) {
        const canvas = document.getElementById('simulationCanvas');
        const rect = canvas.getBoundingClientRect();
        canvas.width = rect.width;
        canvas.height = rect.height;
        simulator.reset();
    }
});

// Setup trebuchet type selection dropdown
function setupTrebuchetTypeButtons() {
    const typeSelect = document.getElementById('trebuchetType');
    
    if (typeSelect) {
        typeSelect.addEventListener('change', (e) => {
            const type = e.target.value;
            buildParameterInputs(type);
            simulator.buildTrebuchet(type, simulator.parameters);
            updateParameterInputs();
            updateButtonStates();
            updateStats(simulator.getStats()); // Update stats display with estimated distance
        });
        
        // Initialize with current type
        buildParameterInputs(typeSelect.value);
    }
}

// Update parameter input fields to reflect current values
function updateParameterInputs() {
    const params = simulator.parameters;
    Object.keys(params).forEach(param => {
        const input = document.getElementById(param);
        if (input) {
            input.value = params[param];
        }
    });
}

// Build parameter inputs dynamically based on builder config
function buildParameterInputs(type) {
    const container = document.querySelector('.section .parameters');
    if (!container) return;
    
    // Get the builder class and its parameter config
    const builderClass = simulator.getBuilderClass(type);
    if (!builderClass || !builderClass.getParameterConfig) {
        console.warn(`No parameter config for type: ${type}`);
        return;
    }
    
    const paramConfig = builderClass.getParameterConfig();
    
    // Clear existing parameter inputs
    container.innerHTML = '';
    
    // Build new parameter inputs
    paramConfig.forEach(param => {
        const paramGroup = document.createElement('div');
        paramGroup.className = 'param-group';
        
        const label = document.createElement('label');
        label.setAttribute('for', param.id);
        label.textContent = param.label;
        
        const input = document.createElement('input');
        input.type = 'number';
        input.id = param.id;
        input.step = param.step;
        input.min = param.min || 0;
        input.max = param.max || 10000;
        input.value = simulator.parameters[param.id] !== undefined ? simulator.parameters[param.id] : param.default;
        
        const unit = document.createElement('span');
        unit.className = 'unit';
        unit.textContent = param.unit;
        
        paramGroup.appendChild(label);
        paramGroup.appendChild(input);
        paramGroup.appendChild(unit);
        container.appendChild(paramGroup);
        
        // Add event listener for parameter changes
        input.addEventListener('change', (e) => {
            const value = e.target.value;
            simulator.updateParameter(param.id, value);
            updateButtonStates();
        });
    });
}

// Setup parameter controls (sliders)
function setupParameterControls() {
    // Setup play speed control separately
    const playSpeedSlider = document.getElementById('playSpeed');
    const playSpeedValue = document.getElementById('playSpeedValue');
    
    if (playSpeedSlider && playSpeedValue) {
        playSpeedSlider.addEventListener('input', (e) => {
            const value = e.target.value;
            playSpeedValue.textContent = value + 'x';
            simulator.setPlaySpeed(value);
        });
        playSpeedValue.textContent = playSpeedSlider.value + 'x';
    }
    // Note: Individual parameter event listeners are now added dynamically in buildParameterInputs()
}

// Setup simulation control buttons
function setupSimulationControls() {
    const resetBtn = document.getElementById('resetBtn');
    const pauseBtn = document.getElementById('pauseBtn');
    
    resetBtn.addEventListener('click', () => {
        simulator.reset();
        simulator.pause();
        updateButtonStates();
        pauseBtn.textContent = 'Play';
        updateStats({ distance: '0', height: '0', velocity: '0', maxDistance: '0', maxHeight: '0', maxVelocity: '0', time: '0', estimatedDistance: '0' });
    });
    
    pauseBtn.addEventListener('click', () => {
        const isPaused = simulator.pause();
        pauseBtn.textContent = isPaused ? 'Play' : 'Pause';
    });
    
    const stepBtn = document.getElementById('stepBtn');
    const stepMultiBtn = document.getElementById('stepMultiBtn');
    const stepCountInput = document.getElementById('stepCount');
    
    stepBtn.addEventListener('click', () => {
        simulator.step(1);
    });
    
    stepMultiBtn.addEventListener('click', () => {
        const numSteps = parseInt(stepCountInput.value) || 1;
        simulator.step(numSteps);
    });
}

// Setup keyboard shortcuts
function setupKeyboardShortcuts() {
    document.addEventListener('keydown', (e) => {
        // Space bar to toggle pause
        if (e.code === 'Space') {
            e.preventDefault();
            const pauseBtn = document.getElementById('pauseBtn');
            const isPaused = simulator.pause();
            pauseBtn.textContent = isPaused ? 'Play' : 'Pause';
        }
        // Enter to reset
        else if (e.code === 'Enter') {
            e.preventDefault();
            simulator.reset();
            simulator.pause();
            updateButtonStates();
            document.getElementById('pauseBtn').textContent = 'Play';
            updateStats({ distance: '0', height: '0', velocity: '0', maxDistance: '0', maxHeight: '0', maxVelocity: '0', time: '0', estimatedDistance: '0' });
        }
        // Arrow right to step forward (single frame)
        else if (e.code === 'ArrowRight') {
            e.preventDefault();
            simulator.step(1);
        }
        // Shift + Arrow right to step forward (10 frames)
        else if (e.code === 'ArrowRight' && e.shiftKey) {
            e.preventDefault();
            simulator.step(10);
        }
    });
}

// Update button states based on simulation state
function updateButtonStates() {
    // No special button state management needed anymore
}

// Setup stats updater
function setupStatsUpdater() {
    setInterval(() => {
        if (simulator && !simulator.paused) {
            const stats = simulator.getStats();
            updateStats(stats);
        }
    }, 100); // Update stats 10 times per second
}

// Update stats display
function updateStats(stats) {
    document.getElementById('distanceValue').textContent = stats.distance + ' m';
    document.getElementById('currentHeightValue').textContent = stats.height + ' m';
    document.getElementById('velocityValue').textContent = stats.velocity + ' m/s';
    document.getElementById('maxDistanceValue').textContent = stats.maxDistance + ' m';
    document.getElementById('heightValue').textContent = stats.maxHeight + ' m';
    document.getElementById('maxVelocityValue').textContent = stats.maxVelocity + ' m/s';
    document.getElementById('timeValue').textContent = stats.time + ' s';
    if (stats.estimatedDistance !== undefined) {
        document.getElementById('estimatedDistanceValue').textContent = stats.estimatedDistance + ' m';
    }
}

// Utility function to format numbers
function formatNumber(num, decimals = 2) {
    return parseFloat(num).toFixed(decimals);
}


