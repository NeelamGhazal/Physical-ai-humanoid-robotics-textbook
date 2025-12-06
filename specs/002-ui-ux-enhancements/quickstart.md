# Quickstart Guide: UI/UX Enhancements for Physical AI & Humanoid Robotics Textbook

## Overview
This guide provides instructions for setting up and customizing the futuristic UI/UX enhancements for the Physical AI & Humanoid Robotics textbook platform.

## Prerequisites
- Node.js 18+ (for Docusaurus frontend)
- npm or yarn package manager
- Git for version control
- A modern web browser for testing

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Website Directory
```bash
cd website
```

### 3. Install Dependencies
```bash
npm install
```

### 4. Add Required Google Fonts
Add the following to your `docusaurus.config.js` in the `headTags` section:

```javascript
headTags: [
  {
    tagName: 'link',
    attributes: {
      rel: 'preconnect',
      href: 'https://fonts.googleapis.com',
    },
  },
  {
    tagName: 'link',
    attributes: {
      rel: 'preconnect',
      href: 'https://fonts.gstatic.com',
      crossOrigin: 'anonymous',
    },
  },
  {
    tagName: 'link',
    attributes: {
      href: 'https://fonts.googleapis.com/css2?family=Exo+2:wght@400;500;600;700&family=Orbitron:wght@400;500;600;700&display=swap',
      rel: 'stylesheet',
    },
  },
],
```

### 5. Configure Tailwind CSS
Create/update `tailwind.config.js`:

```javascript
/** @type {import('tailwindcss').Config} */
module.exports = {
  presets: [require('@docusaurus/core/lib/babel/preset')],
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./pages/**/*.{js,jsx,ts,tsx}",
    "./components/**/*.{js,jsx,ts,tsx}",
    "./theme/**/*.{js,jsx,ts,tsx}",
    "./node_modules/@docusaurus/core/lib/**/*.{js,jsx,ts,tsx}",
    "./docs/**/*.{md,mdx}",
  ],
  darkMode: 'class', // or 'media' if using system preference
  theme: {
    extend: {
      colors: {
        'primary-cyan': '#00f0ff',
        'secondary-magenta': '#ff2a6d',
        'dark-bg': '#0f0f1b',
        'dark-surface': '#1a1a2e',
      },
      boxShadow: {
        'neon': '0 0 8px theme("colors.primary-cyan"), inset 0 0 8px theme("colors.primary-cyan")',
        'neon-magenta': '0 0 8px theme("colors.secondary-magenta"), inset 0 0 8px theme("colors.secondary-magenta")',
        'neon-lg': '0 0 16px theme("colors.primary-cyan"), inset 0 0 16px theme("colors.primary-cyan")',
      },
      animation: {
        'float': 'float 6s ease-in-out infinite',
        'pulse-glow': 'pulse-glow 2s cubic-bezier(0.4, 0, 0.6, 1) infinite',
        'neon-flicker': 'neon-flicker 3s infinite alternate',
      },
      keyframes: {
        float: {
          '0%, 100%': { transform: 'translateY(0)' },
          '50%': { transform: 'translateY(-10px)' },
        },
        'pulse-glow': {
          '0%, 100%': {
            boxShadow: '0 0 0 0 rgba(0, 240, 255, 0.4)',
          },
          '50%': {
            boxShadow: '0 0 0 10px rgba(0, 240, 255, 0)',
          },
        },
        'neon-flicker': {
          '0%, 100%': {
            textShadow: '0 0 5px currentColor, 0 0 10px currentColor, 0 0 20px currentColor, 0 0 40px #00f0ff, 0 0 80px #00f0ff',
          },
          '25%': {
            textShadow: '0 0 2px currentColor, 0 0 5px currentColor, 0 0 10px currentColor, 0 0 20px #00f0ff, 0 0 40px #00f0ff',
          },
          '50%': {
            textShadow: '0 0 0px currentColor, 0 0 0px currentColor, 0 0 0px currentColor, 0 0 0px #00f0ff, 0 0 0px #00f0ff',
          },
          '75%': {
            textShadow: '0 0 3px currentColor, 0 0 8px currentColor, 0 0 15px currentColor, 0 0 30px #00f0ff, 0 0 60px #00f0ff',
          },
        },
      },
    },
  },
  plugins: [],
};
```

### 6. Update PostCSS Configuration
Make sure `postcss.config.js` includes Tailwind:

```javascript
module.exports = {
  plugins: {
    tailwindcss: {},
    autoprefixer: {},
  },
};
```

## Creating Custom Components

### 1. Hero Section Component
Create `src/components/HeroSection/index.js`:

```jsx
import React, { useState } from 'react';
import clsx from 'clsx';

const HeroSection = ({ title, subtitle, ctaText }) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <section className="relative bg-gradient-to-br from-indigo-900 via-purple-900 to-violet-900 overflow-hidden">
      {/* Animated background elements */}
      <div className="absolute inset-0 z-0">
        {[...Array(20)].map((_, i) => (
          <div
            key={i}
            className="absolute rounded-full opacity-10 animate-pulse"
            style={{
              top: `${Math.random() * 100}%`,
              left: `${Math.random() * 100}%`,
              width: `${Math.random() * 100 + 20}px`,
              height: `${Math.random() * 100 + 20}px`,
              backgroundColor: Math.random() > 0.5 ? '#00f0ff' : '#ff2a6d',
              animationDuration: `${Math.random() * 10 + 5}s`,
            }}
          />
        ))}
      </div>

      <div className="relative z-10 max-w-7xl mx-auto px-4 sm:px-6 lg:px-8 py-24 lg:py-32">
        <div className="text-center">
          <h1 className="text-4xl sm:text-5xl md:text-6xl font-extrabold tracking-tight text-white mb-6 font-orbitron">
            <span className={clsx(
              "block",
              isHovered ? "animate-neon-flicker" : ""
            )}>
              {title}
            </span>
          </h1>
          <p className="mt-6 max-w-2xl mx-auto text-xl text-gray-300">
            {subtitle}
          </p>
          <div className="mt-10">
            <button
              className={clsx(
                "inline-block px-8 py-4 border-0 rounded-md text-base font-medium text-white",
                "bg-gradient-to-r from-cyan-500 to-blue-600",
                "hover:from-cyan-400 hover:to-blue-500",
                "transform transition-all duration-300 ease-in-out",
                "hover:scale-105",
                "focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-cyan-500",
                "shadow-lg shadow-cyan-500/50 hover:shadow-cyan-500/75",
                isHovered ? "shadow-neon-lg" : "shadow-neon"
              )}
              onMouseEnter={() => setIsHovered(true)}
              onMouseLeave={() => setIsHovered(false)}
            >
              {ctaText}
            </button>
          </div>
        </div>
      </div>
    </section>
  );
};

export default HeroSection;
```

### 2. Module Card Component
Create `src/components/ModuleCard/index.js`:

```jsx
import React, { useState } from 'react';
import clsx from 'clsx';

const ModuleCard = ({ title, description, moduleId, icon }) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <div
      className={clsx(
        "relative rounded-xl border-2 bg-dark-surface overflow-hidden",
        "transition-all duration-300 ease-in-out transform",
        "hover:scale-[1.02]",
        "before:absolute before:inset-0 before:rounded-xl before:p-[2px] before:bg-[linear-gradient(90deg,#00f0ff,#ff2a6d)]",
        "before:-z-10 before:transition-opacity before:duration-300",
        isHovered ? "before:opacity-100" : "before:opacity-70",
        "after:absolute after:inset-[2px] after:rounded-[calc(0.75rem-2px)] after:bg-dark-bg after:-z-20"
      )}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
    >
      <div className="p-6">
        <div className="flex items-center mb-4">
          {icon && <span className="mr-3 text-2xl">{icon}</span>}
          <h3 className="text-xl font-bold text-white font-orbitron">{title}</h3>
        </div>
        <p className="text-gray-300 mb-4">{description}</p>
        <div className="flex justify-between items-center">
          <span className="text-sm text-cyan-400 font-mono">Module {moduleId}</span>
          <span className="text-xs px-2 py-1 bg-gray-800 text-gray-300 rounded">
            Interactive
          </span>
        </div>
      </div>
    </div>
  );
};

export default ModuleCard;
```

### 3. Neon Mode Toggle Component
Create `src/components/NeonModeToggle/index.js`:

```jsx
import React, { useState, useEffect } from 'react';
import clsx from 'clsx';

const NeonModeToggle = () => {
  const [neonMode, setNeonMode] = useState(false);
  const [mounted, setMounted] = useState(false);

  useEffect(() => {
    setMounted(true);
    // Check for saved preference
    const savedMode = localStorage.getItem('neonMode');
    if (savedMode !== null) {
      setNeonMode(savedMode === 'true');
    }
  }, []);

  useEffect(() => {
    if (!mounted) return;

    if (neonMode) {
      document.documentElement.classList.add('neon-mode');
      localStorage.setItem('neonMode', 'true');
    } else {
      document.documentElement.classList.remove('neon-mode');
      localStorage.setItem('neonMode', 'false');
    }
  }, [neonMode, mounted]);

  if (!mounted) {
    return (
      <div className="w-12 h-6 bg-gray-700 rounded-full flex items-center px-1">
        <div className="w-4 h-4 bg-gray-500 rounded-full"></div>
      </div>
    );
  }

  return (
    <button
      onClick={() => setNeonMode(!neonMode)}
      className={clsx(
        "relative w-12 h-6 bg-gray-700 rounded-full flex items-center px-1 transition-colors duration-300",
        "focus:outline-none focus:ring-2 focus:ring-cyan-500",
        neonMode ? "bg-cyan-500" : "bg-gray-700"
      )}
      aria-label={neonMode ? "Disable neon mode" : "Enable neon mode"}
    >
      <div
        className={clsx(
          "bg-white rounded-full aspect-square h-4 transition-transform duration-300",
          "shadow-md",
          neonMode ? "transform translate-x-6 shadow-cyan-400/80" : "shadow-gray-400/80"
        )}
        style={{
          boxShadow: neonMode
            ? "0 0 8px #00f0ff, 0 0 16px #00f0ff"
            : "0 0 4px #6b7280"
        }}
      />
      <span className="sr-only">
        {neonMode ? "Neon mode enabled" : "Neon mode disabled"}
      </span>
    </button>
  );
};

export default NeonModeToggle;
```

## Custom CSS Styles
Add to `src/css/custom.css`:

```css
/* Custom styles for futuristic theme */
@import url('https://fonts.googleapis.com/css2?family=Exo+2:wght@400;500;600;700&family=Orbitron:wght@400;500;600;700&display=swap');

:root {
  /* Color palette */
  --ifm-color-primary: #00f0ff;
  --ifm-color-primary-dark: #00d1d9;
  --ifm-color-primary-darker: #00c5cc;
  --ifm-color-primary-darkest: #00a1a8;
  --ifm-color-primary-light: #1ff9ff;
  --ifm-color-primary-lighter: #2dffff;
  --ifm-color-primary-lightest: #5affff;
  --ifm-background-color: #0f0f1b;
  --ifm-background-surface-color: #1a1a2e;
  --ifm-navbar-background-color: #0f0f1b;
  --ifm-footer-background-color: #0f0f1b;
}

/* Dark mode base */
html[data-theme='dark'] {
  --ifm-background-color: #0f0f1b;
  --ifm-background-surface-color: #1a1a2e;
  --ifm-navbar-background-color: #0f0f1b;
  --ifm-footer-background-color: #0f0f1b;
}

/* Neon mode enhancements */
html.neon-mode {
  --ifm-color-primary: #00f0ff;
  --ifm-color-primary-dark: #00f0ff;
  --ifm-color-primary-darker: #00f0ff;
  --ifm-color-primary-darkest: #00f0ff;
  --ifm-color-primary-light: #ff2a6d;
  --ifm-color-primary-lighter: #ff2a6d;
  --ifm-color-primary-lightest: #ff2a6d;
}

/* Apply neon enhancements only in neon mode */
html.neon-mode .navbar,
html.neon-mode .menu,
html.neon-mode .theme-doc-sidebar-menu {
  box-shadow: 0 0 15px rgba(0, 240, 255, 0.3);
}

/* Glow effects for interactive elements */
html.neon-mode button:hover,
html.neon-mode .pagination-nav__link:hover,
html.neon-mode .card:hover {
  box-shadow: 0 0 15px rgba(0, 240, 255, 0.5) !important;
  transition: box-shadow 0.3s ease;
}

/* Custom font families */
.font-orbitron {
  font-family: 'Orbitron', monospace;
}

.font-exo2 {
  font-family: 'Exo 2', sans-serif;
}

/* Animated floating elements */
.floating-element {
  animation: float 6s ease-in-out infinite;
}

@keyframes float {
  0% { transform: translateY(0px); }
  50% { transform: translateY(-10px); }
  100% { transform: translateY(0px); }
}

/* Neon flicker effect */
.neon-flicker {
  animation: neon-flicker 3s infinite alternate;
}

@keyframes neon-flicker {
  0%, 100% {
    text-shadow: 0 0 5px currentColor, 0 0 10px currentColor, 0 0 20px currentColor, 0 0 40px currentColor, 0 0 80px currentColor;
  }
  25% {
    text-shadow: 0 0 2px currentColor, 0 0 5px currentColor, 0 0 10px currentColor, 0 0 20px currentColor, 0 0 40px currentColor;
  }
  50% {
    text-shadow: 0 0 0px currentColor, 0 0 0px currentColor, 0 0 0px currentColor, 0 0 0px currentColor, 0 0 0px currentColor;
  }
  75% {
    text-shadow: 0 0 3px currentColor, 0 0 8px currentColor, 0 0 15px currentColor, 0 0 30px currentColor, 0 0 60px currentColor;
  }
}

/* Custom card styling */
.theme-card {
  border: 2px solid transparent;
  position: relative;
  overflow: hidden;
}

.theme-card::before {
  content: '';
  position: absolute;
  inset: 0;
  border-radius: inherit;
  padding: 2px;
  background: linear-gradient(90deg, #00f0ff, #ff2a6d);
  mask: linear-gradient(#fff 0 0) content-box, linear-gradient(#fff 0 0);
  mask-composite: exclude;
  -webkit-mask-composite: xor;
}

/* Reduced motion support */
@media (prefers-reduced-motion: reduce) {
  .floating-element,
  .neon-flicker {
    animation: none;
  }

  *,
  *::before,
  *::after {
    animation-duration: 0.01ms !important;
    animation-iteration-count: 1 !important;
    transition-duration: 0.01ms !important;
  }
}
```

## Adding Animated SVGs
Place animated robot SVGs in `static/img/`:

### Floating Robot SVG (static/img/floating-robot.svg)
```svg
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" class="floating-element">
  <defs>
    <linearGradient id="neonGlow" x1="0%" y1="0%" x2="100%" y2="100%">
      <stop offset="0%" style="stop-color:#00f0ff;stop-opacity:1" />
      <stop offset="100%" style="stop-color:#ff2a6d;stop-opacity:1" />
    </linearGradient>
  </defs>
  <g fill="none" stroke="url(#neonGlow)" stroke-width="2">
    <!-- Robot body -->
    <rect x="35" y="30" width="30" height="40" rx="5" class="neon-flicker" />
    <!-- Head -->
    <circle cx="50" cy="20" r="10" class="neon-flicker" />
    <!-- Arms -->
    <line x1="25" y1="40" x2="35" y2="40" class="neon-flicker" />
    <line x1="65" y1="40" x2="75" y2="40" class="neon-flicker" />
    <!-- Legs -->
    <line x1="40" y1="70" x2="40" y2="85" class="neon-flicker" />
    <line x1="60" y1="70" x2="60" y2="85" class="neon-flicker" />
    <!-- Eyes -->
    <circle cx="45" cy="18" r="1.5" fill="url(#neonGlow)" class="neon-flicker" />
    <circle cx="55" cy="18" r="1.5" fill="url(#neonGlow)" class="neon-flicker" />
  </g>
</svg>
```

## Integration with Docusaurus
To use these components in your documentation, import them in your MDX files:

```mdx
import HeroSection from '@site/src/components/HeroSection';
import ModuleCard from '@site/src/components/ModuleCard';
import NeonModeToggle from '@site/src/components/NeonModeToggle';

<HeroSection
  title="Physical AI & Humanoid Robotics"
  subtitle="A comprehensive textbook on embodied artificial intelligence"
  ctaText="Start Learning"
/>

## Core Modules

<div className="row">
  <div className="col col--6">
    <ModuleCard
      title="ROS 2"
      description="Learn the Robot Operating System for building robotic applications"
      moduleId="1"
      icon="🤖"
    />
  </div>
  <div className="col col--6">
    <ModuleCard
      title="Gazebo/Unity"
      description="Simulation environments for testing and training"
      moduleId="2"
      icon="🎮"
    />
  </div>
</div>
```

## Running the Development Server
```bash
npm run start
```

Your textbook with the new UI/UX enhancements will be available at http://localhost:3000.

## Customization Tips
1. Modify the color variables in `tailwind.config.js` to adjust the color scheme
2. Adjust animation durations in the CSS to change the speed of visual effects
3. Update the font families in both the Docusaurus config and CSS to change typography
4. Customize the gradient colors in the HeroSection component
5. Adjust the glow intensity in the custom CSS by modifying the shadow values