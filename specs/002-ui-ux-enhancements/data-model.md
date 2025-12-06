# Data Model: UI/UX Enhancements for Physical AI & Humanoid Robotics Textbook

## Entity Definitions

### ThemeConfiguration
- **id**: UUID (Primary Key)
- **user_id**: UUID (Foreign Key to User, Optional - null for global settings)
- **theme_mode**: String (Required, enum: "dark", "neon", "light")
- **accent_color**: String (Default: "#00f0ff", primary cyan)
- **secondary_color**: String (Default: "#ff2a6d", secondary magenta)
- **glow_intensity**: Float (Default: 1.0, range: 0.0-2.0)
- **animation_enabled**: Boolean (Default: true)
- **font_family_heading**: String (Default: "Orbitron", enum: "Orbitron", "Exo 2", "system")
- **font_family_body**: String (Default: "Exo 2", enum: "Orbitron", "Exo 2", "system")
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

### VisualElement
- **id**: UUID (Primary Key)
- **element_type**: String (Required, enum: "card", "button", "navigation", "hero", "module")
- **element_id**: String (Optional, for specific element identification)
- **glow_effect**: Boolean (Default: true)
- **neon_border**: Boolean (Default: true)
- **hover_animation**: Boolean (Default: true)
- **animation_speed**: String (Default: "normal", enum: "slow", "normal", "fast")
- **color_scheme**: JSON (Optional, custom color overrides)
- **settings**: JSON (Optional, element-specific settings)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

### AnimationController
- **id**: UUID (Primary Key)
- **animation_type**: String (Required, enum: "floating-robot", "glow-pulse", "neon-flicker", "hover-bounce")
- **element_id**: String (Optional, to associate with specific UI element)
- **is_active**: Boolean (Default: true)
- **speed_factor**: Float (Default: 1.0, range: 0.1-3.0)
- **motion_preference_respected**: Boolean (Default: true, respects user's reduced motion settings)
- **performance_priority**: String (Default: "balanced", enum: "performance", "balanced", "quality")
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

### ResponsiveLayout
- **id**: UUID (Primary Key)
- **breakpoint_target**: String (Required, enum: "mobile", "tablet", "desktop", "wide")
- **layout_type**: String (Required, enum: "grid", "flex", "column", "row")
- **element_adjustments**: JSON (Optional, specific adjustments for this breakpoint)
- **font_scaling**: Float (Default: 1.0, adjusts font sizes for screen size)
- **animation_adaptation**: String (Default: "maintain", enum: "maintain", "reduce", "remove")
- **interaction_modifications**: JSON (Optional, changes to interactions based on screen size)
- **created_at**: DateTime (Required)
- **updated_at**: DateTime (Required)

## Relationships

- **User** 1 ←→ * **ThemeConfiguration**: A user can have multiple theme configurations (global and per-section)
- **ThemeConfiguration** 1 ←→ * **VisualElement**: A theme configuration can affect multiple visual elements
- **VisualElement** 1 ←→ * **AnimationController**: A visual element can have multiple associated animations
- **ResponsiveLayout** 1 ←→ * **VisualElement**: A responsive layout configuration can apply to multiple visual elements

## Validation Rules

### ThemeConfiguration
- theme_mode must be one of: "dark", "neon", "light"
- accent_color and secondary_color must be valid hex color codes
- glow_intensity must be between 0.0 and 2.0
- font_family_heading and font_family_body must be from the allowed enum values

### VisualElement
- element_type must be one of: "card", "button", "navigation", "hero", "module"
- animation_speed must be one of: "slow", "normal", "fast"
- color_scheme must be valid JSON with proper color definitions

### AnimationController
- animation_type must be one of: "floating-robot", "glow-pulse", "neon-flicker", "hover-bounce"
- speed_factor must be between 0.1 and 3.0
- performance_priority must be one of: "performance", "balanced", "quality"

### ResponsiveLayout
- breakpoint_target must be one of: "mobile", "tablet", "desktop", "wide"
- layout_type must be one of: "grid", "flex", "column", "row"
- font_scaling must be positive value
- animation_adaptation must be one of: "maintain", "reduce", "remove"

## State Transitions

### ThemeConfiguration
- **Initial State**: theme_mode = "dark", glow_intensity = 1.0
- **Neon Mode Activated**: theme_mode = "neon", glow_intensity increased to 1.5, animations enhanced
- **Light Mode Activated**: theme_mode = "light", glow intensity reduced

### AnimationController
- **Initial State**: is_active = true, speed_factor = 1.0
- **Performance Reduced**: performance_priority = "performance", speed_factor adjusted accordingly
- **Motion Restricted**: animation_type modified to respect user's reduced motion preferences

## Indexes for Performance
- ThemeConfiguration.user_id (for quick user lookup)
- VisualElement.element_type (for filtering by element type)
- AnimationController.animation_type (for filtering by animation type)
- ResponsiveLayout.breakpoint_target (for quick breakpoint lookups)