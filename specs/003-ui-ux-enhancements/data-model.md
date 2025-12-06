# Data Model: UI/UX Enhancements for Physical AI & Humanoid Robotics Textbook

## Overview
This document describes the data models and entities for the UI/UX enhancements feature, based on the key entities identified in the feature specification.

## Entity: TextbookPage
**Description**: Represents a page in the textbook with enhanced visual styling, typography, and layout elements

**Fields**:
- `id`: string - Unique identifier for the page
- `title`: string - Page title displayed in the UI
- `content`: MDX content - The educational content in MDX format
- `theme`: ThemeConfig - Theme configuration for this page
- `layout`: LayoutType - Layout type (e.g., 'standard', 'title-page', 'content')
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `title` must be 1-200 characters
- `content` must be valid MDX
- `layout` must be one of the predefined layout types

**Relationships**:
- One-to-many with ContentCard (contains multiple content cards)
- Many-to-one with ThemeConfig (uses one theme configuration)

## Entity: ThemeConfig
**Description**: Configuration for visual styling, colors, typography, and design elements

**Fields**:
- `id`: string - Unique identifier for the theme
- `name`: string - Theme name (e.g., 'futuristic', 'dark-mode')
- `colors`: ColorScheme - Color configuration object
- `typography`: TypographyConfig - Typography settings
- `designElements`: DesignElementConfig - Configuration for borders, dividers, etc.
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `name` must be 1-50 characters
- `colors` must meet WCAG AA contrast requirements
- `typography` must specify valid font families

**Relationships**:
- Many-to-many with TextbookPage (multiple pages can use the same theme)
- One-to-one with ColorScheme (has one color scheme)

## Entity: ColorScheme
**Description**: Color configuration following the specified color scheme requirements

**Fields**:
- `id`: string - Unique identifier for the color scheme
- `primary`: string - Primary color (#00f0ff as per FR-001)
- `secondary`: string - Secondary color (#ff2a6d as per FR-001)
- `background`: string - Background color (#0d0d1a as per FR-001)
- `text`: string - Text color (#e0e0ff as per FR-001)
- `contrastRatios`: ContrastRatios - Calculated contrast ratios
- `createdAt`: Date - Creation timestamp

**Validation Rules**:
- All colors must be valid hex codes
- Contrast ratios must meet WCAG AA requirements (4.5:1 for normal text, 3:1 for large text)

**Relationships**:
- One-to-one with ThemeConfig (used by one theme configuration)

## Entity: TypographyConfig
**Description**: Typography settings for headings and body text

**Fields**:
- `id`: string - Unique identifier for typography config
- `headingFont`: string - Font for headings (Orbitron as per FR-002)
- `headingWeights`: number[] - Available font weights for headings (700-900 as per FR-002)
- `bodyFont`: string - Font for body text (Inter as per FR-002)
- `fontSizeScale`: FontSizeScale - Size scale for different text elements
- `lineHeight`: number - Default line height for readability
- `createdAt`: Date - Creation timestamp

**Validation Rules**:
- `headingFont` must be 'Orbitron' or a valid fallback
- `bodyFont` must be 'Inter' or a valid fallback
- `headingWeights` must include 700-900 range

**Relationships**:
- One-to-one with ThemeConfig (used by one theme configuration)

## Entity: DesignElementConfig
**Description**: Configuration for design elements like borders, dividers, and spacing

**Fields**:
- `id`: string - Unique identifier for design element config
- `animatedBorders`: boolean - Whether content cards have animated borders (FR-005)
- `sectionDividers`: DividerType - Type of section dividers (SVG waves or circuit lines per FR-006)
- `spacingScale`: string - Tailwind spacing scale to use (FR-007)
- `printMargins`: boolean - Whether print-friendly margins are enabled (FR-008)
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `sectionDividers` must be one of the allowed types (SVG waves, circuit lines)
- `spacingScale` must follow Tailwind's spacing scale

**Relationships**:
- One-to-one with ThemeConfig (used by one theme configuration)

## Entity: NavigationComponent
**Description**: UI element containing sticky navbar with logo and navigation

**Fields**:
- `id`: string - Unique identifier for the navigation component
- `sticky`: boolean - Whether navbar stays at top while scrolling (FR-009)
- `logoType`: LogoType - Type of logo ('text-based' as per clarifications)
- `logoText`: string - Text for the logo ("Physical AI & Humanoid Robotics Textbook")
- `navigationLinks`: NavigationLink[] - Array of navigation links
- `languageToggle`: boolean - Whether language toggle is visible
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `logoText` must be 1-100 characters
- `navigationLinks` must have valid URLs
- `logoType` must be 'text-based' per clarifications

**Relationships**:
- Many-to-one with ThemeConfig (uses one theme configuration)
- One-to-many with NavigationLink (contains multiple navigation links)

## Entity: FooterComponent
**Description**: UI element containing copyright, social links, and Urdu toggle functionality

**Fields**:
- `id`: string - Unique identifier for the footer component
- `copyrightText`: string - Copyright information text
- `socialLinks`: SocialLink[] - Array of social media links
- `urduToggleEnabled`: boolean - Whether Urdu toggle functionality is available (FR-010)
- `urduTranslations`: UrduTranslation[] - Available Urdu translations
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `copyrightText` must be 1-200 characters
- `urduToggleEnabled` must be true per FR-010
- `socialLinks` must have valid URLs

**Relationships**:
- Many-to-one with ThemeConfig (uses one theme configuration)
- One-to-many with UrduTranslation (contains multiple translations)

## Entity: ContentCard
**Description**: Styled container with animated borders for presenting textbook content sections

**Fields**:
- `id`: string - Unique identifier for the content card
- `title`: string - Card title
- `content`: string - Card content in MDX format
- `animatedBorder`: boolean - Whether the border is animated (FR-005)
- `borderColor`: string - Color of the animated border
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `title` must be 1-100 characters
- `content` must be valid MDX
- `borderColor` must meet contrast requirements

**Relationships**:
- Many-to-one with TextbookPage (belongs to one page)
- Many-to-one with ThemeConfig (uses one theme configuration)

## Entity: SectionDivider
**Description**: Custom SVG element for visually separating content sections

**Fields**:
- `id`: string - Unique identifier for the section divider
- `type`: DividerType - Type of divider ('svg-waves', 'circuit-lines', etc.)
- `color`: string - Color of the divider
- `height`: number - Height of the divider in pixels
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `type` must be one of the allowed divider types (SVG waves, circuit lines per FR-006)
- `height` must be between 10-100 pixels

**Relationships**:
- Many-to-one with TextbookPage (used in one page)
- Many-to-one with ThemeConfig (uses one theme configuration)

## Entity: UrduTranslation
**Description**: Translation data for Urdu language support

**Fields**:
- `id`: string - Unique identifier for the translation
- `key`: string - Translation key (e.g., 'navbar.home', 'footer.copyright')
- `englishText`: string - Original English text
- `urduText`: string - Translated Urdu text
- `rtlLayout`: boolean - Whether to apply RTL layout for this content
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `key` must be unique
- `urduText` must be valid Urdu text
- `rtlLayout` must be true for appropriate content

**Relationships**:
- Many-to-one with FooterComponent (used by one footer)
- Many-to-one with NavigationComponent (used by one navigation)

## Entity: AnimationConfig
**Description**: Configuration for animations and motion reduction options

**Fields**:
- `id`: string - Unique identifier for animation config
- `particleEffects`: boolean - Whether particle effects are enabled (FR-004)
- `floatingSvg`: boolean - Whether floating humanoid SVG is enabled (FR-004)
- `animatedBorders`: boolean - Whether animated borders are enabled (FR-005)
- `motionReduction`: boolean - Whether motion reduction is enabled (FR-015)
- `performanceDetection`: boolean - Whether performance detection is enabled
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `motionReduction` must be enabled per FR-015
- `performanceDetection` must be true per clarifications

**Relationships**:
- One-to-one with ThemeConfig (used by one theme configuration)
- One-to-many with TitlePage (applied to title pages)

## Entity: TitlePage
**Description**: Specialized page with full-viewport hero layout, gradient background, and animated elements

**Fields**:
- `id`: string - Unique identifier for the title page
- `title`: string - Main title text
- `subtitle`: string - Subtitle text
- `author`: string - Author name
- `gradientFrom`: string - Starting color for gradient (deep indigo)
- `gradientTo`: string - Ending color for gradient (space black)
- `animatedElements`: AnimationConfig - Configuration for animated elements
- `fontFamily`: string - Font family for title (Orbitron)
- `createdAt`: Date - Creation timestamp
- `updatedAt`: Date - Last modification timestamp

**Validation Rules**:
- `title` must be 1-100 characters
- `gradientFrom` and `gradientTo` must create a valid gradient
- Must meet FR-003 requirements for full-viewport hero

**Relationships**:
- One-to-one with AnimationConfig (uses one animation configuration)
- Many-to-one with ThemeConfig (uses one theme configuration)