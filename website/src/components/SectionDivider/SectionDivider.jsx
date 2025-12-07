import React from 'react';

const SectionDivider = ({ type = 'waves', className = "" }) => {
  if (type === 'circuit-lines') {
    return (
      <div
        className={`relative w-full h-16 ${className}`}
        role="separator"
        aria-label="Circuit line section divider"
      >
        <svg
          className="w-full h-full"
          viewBox="0 0 100 10"
          preserveAspectRatio="none"
          xmlns="http://www.w3.org/2000/svg"
          focusable="false"
          aria-hidden="true"
        >
          <path
            d="M0,5 L10,0 L20,10 L30,0 L40,10 L50,0 L60,10 L70,0 L80,10 L90,0 L100,5"
            stroke="url(#circuitGradient)"
            strokeWidth="0.5"
            fill="none"
          />
          <defs>
            <linearGradient id="circuitGradient" x1="0%" y1="0%" x2="100%" y2="0%">
              <stop offset="0%" stopColor="#00f0ff" stopOpacity="0.8" />
              <stop offset="50%" stopColor="#ff2a6d" stopOpacity="0.8" />
              <stop offset="100%" stopColor="#00f0ff" stopOpacity="0.8" />
            </linearGradient>
          </defs>
        </svg>
      </div>
    );
  }

  // Default to waves
  return (
    <div
      className={`relative w-full h-16 ${className}`}
      role="separator"
      aria-label="Wave section divider"
    >
      <svg
        className="w-full h-full"
        viewBox="0 0 100 10"
        preserveAspectRatio="none"
        xmlns="http://www.w3.org/2000/svg"
        focusable="false"
        aria-hidden="true"
      >
        <path
          d="M0,5 Q10,0 20,5 T40,5 T60,5 T80,5 T100,5 L100,10 L0,10 Z"
          fill="url(#waveGradient)"
        />
        <defs>
          <linearGradient id="waveGradient" x1="0%" y1="0%" x2="100%" y2="0%">
            <stop offset="0%" stopColor="#00f0ff" stopOpacity="0.3" />
            <stop offset="50%" stopColor="#ff2a6d" stopOpacity="0.3" />
            <stop offset="100%" stopColor="#00f0ff" stopOpacity="0.3" />
          </linearGradient>
        </defs>
      </svg>
    </div>
  );
};

export default SectionDivider;