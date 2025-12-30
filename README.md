# Advanced Trebuchet Simulator

An interactive web-based trebuchet simulator built with Planck.js physics engine, supporting multiple trebuchet designs with fully customizable parameters.

## Features

### Multiple Trebuchet Types
- **Hinged Counterweight**: Traditional trebuchet with a hinged counterweight for maximum efficiency
- **Whipper**: Features an additional arm segment for enhanced whipping action and increased velocity
- **Floating Arm**: Pivot point can move horizontally, allowing for more efficient energy transfer
- **Walking Arm**: Base can rock and pivot, creating a walking motion during launch

### Customizable Parameters
- Arm Length (2-10 meters)
- Counterweight Mass (50-500 kg)
- Counterweight Size (0.5-2 meters)
- Projectile Mass (1-50 kg)
- Projectile Size (0.1-0.5 meters)
- Sling Length (2-8 meters)
- Arm Mass (10-100 kg)
- Release Angle (30-60 degrees)

### Real-time Statistics
- Distance traveled
- Maximum height reached
- Flight time

## Getting Started

### Quick Start
Simply open `index.html` in a modern web browser. No build process or dependencies required!

### Online Hosting
To host this simulator online, you can use any of these free static hosting services:

1. **GitHub Pages**
   - Create a new GitHub repository
   - Upload all files to the repository
   - Enable GitHub Pages in repository settings
   - Access at `https://yourusername.github.io/repositoryname`

2. **Netlify**
   - Drag and drop the project folder to [Netlify Drop](https://app.netlify.com/drop)
   - Get instant deployment with a URL

3. **Vercel**
   - Install Vercel CLI: `npm install -g vercel`
   - Run `vercel` in the project directory
   - Follow the prompts

## How to Use

1. **Select a Trebuchet Type**: Click one of the four trebuchet type buttons
2. **Customize Parameters**: Adjust sliders to modify the trebuchet design
3. **Fire**: Click the "Fire!" button to launch the projectile
4. **Reset**: Click "Reset" to return to initial state
5. **Pause/Resume**: Click "Pause" to pause the simulation

## Technical Details

### Built With
- **Planck.js**: 2D Box2D physics engine for realistic simulation
- **Vanilla JavaScript**: No frameworks required
- **HTML5 Canvas**: For rendering the simulation
- **CSS3**: Modern, responsive design

### Physics Simulation
The simulator uses Planck.js to accurately model:
- Rigid body dynamics
- Constraint systems (revolute joints, distance joints)
- Collision detection
- Gravitational forces
- Angular momentum and rotation

### Browser Compatibility
Works in all modern browsers that support:
- HTML5 Canvas
- ES6 JavaScript
- CSS3 Flexbox

## Project Structure
```
trebuchet-simulator/
├── index.html                    # Main HTML structure
├── styles.css                    # Styling and layout
├── planck.min.js                # Planck.js physics engine
├── trebuchets/                   # Trebuchet type implementations
│   ├── base-trebuchet.js        # Base class for trebuchet builders
│   ├── hinged-trebuchet.js      # Hinged counterweight trebuchet
│   ├── whipper-trebuchet.js     # Whipper trebuchet implementation
│   ├── floating-arm-trebuchet.js # Floating arm trebuchet
│   └── walking-arm-trebuchet.js  # Walking arm trebuchet
├── trebuchet.js                 # Main simulator and physics engine
├── app.js                       # UI logic and event handlers
└── README.md                    # This file
```

### Code Architecture
The project uses an object-oriented design with a base builder class:
- **BaseTrebuchetBuilder**: Abstract base class with common methods for creating trebuchet components
- **Specific Builders**: Each trebuchet type extends the base class with its unique build logic
- **TrebuchetSimulator**: Main class that manages physics engine and coordinates with builders
- **Separation of Concerns**: Each trebuchet type is in its own file for better maintainability

## Future Enhancements
- Add trajectory visualization trails
- Export/import custom designs
- Leaderboard for distance records
- More trebuchet types (traction, hybrid)
- Adjustable gravity and air resistance
- Multiple projectile shapes
- Sound effects

## Inspiration
Inspired by [VirtualTrebuchet.com](https://virtualtrebuchet.com/), this simulator extends the concept to support additional trebuchet types and provides more customization options.

## License
This project is open source and available for educational and recreational use.

## Contributing
Feel free to fork, modify, and submit pull requests to enhance the simulator!
