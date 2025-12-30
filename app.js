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
            simulator.buildTrebuchet(type, simulator.parameters);
            updateButtonStates();
        });
    }
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
        'releaseAngle',
        'armHeight'
    ];
    
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
    
    parameters.forEach(param => {
        const input = document.getElementById(param);
        
        if (input) {
            // Update trebuchet when value changes
            input.addEventListener('change', (e) => {
                const value = e.target.value;
                simulator.updateParameter(param, value);
                updateButtonStates();
            });
        }
    });
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
        updateStats({ distance: '0', maxHeight: '0', time: '0' });
    });
    
    pauseBtn.addEventListener('click', () => {
        const isPaused = simulator.pause();
        pauseBtn.textContent = isPaused ? 'Play' : 'Pause';
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
            updateStats({ distance: '0', maxHeight: '0', time: '0' });
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
    document.getElementById('heightValue').textContent = stats.maxHeight + ' m';
    document.getElementById('timeValue').textContent = stats.time + ' s';
}

// Utility function to format numbers
function formatNumber(num, decimals = 2) {
    return parseFloat(num).toFixed(decimals);
}
