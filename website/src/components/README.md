# UI Components Documentation

This directory contains custom React components for the Physical AI & Humanoid Robotics textbook with the futuristic theme.

## Component Structure

```
src/components/
├── Navbar/
│   └── CustomNavbar.jsx
├── Footer/
│   └── CustomFooter.jsx
├── TitlePage/
│   └── TitlePage.jsx
├── ContentCard/
│   └── ContentCard.jsx
└── SectionDivider/
    └── SectionDivider.jsx
```

## Component Descriptions

### Navbar/CustomNavbar.jsx
A custom navigation bar with:
- Text-based logo in Orbitron font
- Responsive design with mobile menu toggle
- Urdu language toggle functionality
- Sticky behavior for persistent navigation

### Footer/CustomFooter.jsx
A custom footer with:
- Copyright information
- Social media links
- Urdu language toggle
- RTL layout support for Urdu text

### TitlePage/TitlePage.jsx
A full-viewport hero section with:
- Gradient background (deep indigo to space black)
- Animated particle effects
- Floating humanoid SVGs
- Title and subtitle with proper i18n support

### ContentCard/ContentCard.jsx
Styled content containers with:
- Animated neon borders
- Framer Motion hover effects
- Consistent spacing and typography
- Accessibility attributes

### SectionDivider/SectionDivider.jsx
Custom section dividers with:
- SVG wave patterns
- Circuit line patterns (alternative)
- Smooth transitions between sections
- Color scheme integration

## Design System

### Color Palette
- **Neon Cyan**: `#00f0ff` - Primary accent color
- **Magenta**: `#ff2a6d` - Secondary accent color
- **Space Black**: `#0d0d1a` - Background color
- **Text Light**: `#e0e0ff` - Text color
- **Deep Indigo**: `#4169e1` - Gradient component

### Typography
- **Headings**: Orbitron font (weights 700-900)
- **Body**: Inter font (weights 400-700)
- **Fallback**: System fonts for performance

### Accessibility Features
- WCAG AA compliant color contrast
- Proper ARIA attributes
- Keyboard navigation support
- Focus indicators
- RTL layout support
- Motion reduction options

## Usage Examples

### Content Card
```jsx
import ContentCard from '@site/src/components/ContentCard/ContentCard';

<ContentCard title="Example Card">
  <p>This is an example of a content card with the futuristic theme.</p>
</ContentCard>
```

### Section Divider
```jsx
import SectionDivider from '@site/src/components/SectionDivider/SectionDivider';

// Wave divider
<SectionDivider type="waves" />

// Circuit line divider
<SectionDivider type="circuit-lines" />
```

## Internationalization

All components support English and Urdu languages through the i18next integration. Text content is loaded from translation files in `src/i18n/locales/`.

## Responsive Design

All components are built with responsive design in mind using Tailwind CSS utility classes. They adapt to mobile, tablet, and desktop screen sizes automatically.