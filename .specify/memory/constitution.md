# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Educational Excellence
Content must prioritize clarity and accessibility for O/A-Level to university students. All technical concepts should be explained with concrete examples, visual aids, and step-by-step tutorials. Complex topics must be broken down into digestible sections with progressive difficulty levels.

### II. Modular Chapter Structure
Each chapter must be self-contained and aligned with specific course modules (ROS 2, Gazebo, Isaac, VLA). Chapters should follow a consistent structure with learning objectives, theoretical foundations, practical examples, exercises, and real-world applications. Cross-chapter dependencies must be minimal and clearly documented.

### III. Technical Stack Integrity
All implementations must use the specified technology stack: OpenAI Agents SDK for the RAG chatbot, Qdrant for vector storage, and Neon Postgres for relational data. Code examples must be compatible with Docusaurus framework and deployable to GitHub Pages. Third-party dependencies should be minimized and justified.

### IV. Comprehensive Testing (NON-NEGOTIABLE)
All features, including the RAG chatbot, authentication, and personalization, must have unit, integration, and end-to-end tests. Test coverage must be at least 80% for production code. Chatbot responses must be validated for accuracy and safety before deployment.

### V. Security-First Implementation
Authentication, personalization, and all user data features must implement security best practices. Secure coding practices must be followed, including input validation, sanitization, and protection against common vulnerabilities (XSS, CSRF, injection attacks). User privacy must be protected with appropriate data handling.

### VI. Multi-Language Support
The platform must support Urdu localization alongside English content. Text, UI elements, and documentation must be internationalization-ready with clear separation of content and presentation. Cultural sensitivity must be maintained in all examples and explanations.

## Technical Requirements

### Docusaurus Framework Standards
- All content must be written in Markdown/MDX following Docusaurus conventions
- Navigation and search functionality must work seamlessly
- Responsive design for multiple device types
- SEO optimization for educational content discovery
- Accessibility compliance (WCAG 2.1 AA standards)

### RAG Chatbot Architecture
- OpenAI Agents SDK must be used for conversation management
- Qdrant vector database for document retrieval and similarity search
- Neon Postgres for user data, preferences, and conversation history
- Rate limiting and usage monitoring to control API costs
- Context-aware responses that reference specific textbook content

### Performance and Deployment
- GitHub Pages deployment with fast loading times (under 3 seconds)
- Optimized assets and lazy loading for media content
- CDN distribution for global accessibility
- Offline capability for core textbook content
- Caching strategies for chatbot responses and search results

## Development Workflow

### Quality Assurance Process
- Code reviews required for all changes with at least one senior reviewer
- Automated testing pipeline must pass before merge
- Documentation updates required for all feature changes
- Peer review of educational content for accuracy and clarity
- Accessibility testing for all UI components

### Version Control and Collaboration
- Git flow with feature branches, pull requests, and semantic versioning
- Changelog maintained for all releases with breaking changes clearly marked
- Branch protection rules for main/master branches
- Regular dependency updates with security scanning
- Collaborative authoring process for educational content

## Governance

This constitution serves as the authoritative guide for all development and content decisions. All contributors must adhere to these principles. Amendments require documentation of rationale, impact assessment, and approval from project maintainers. Code quality tools (linting, formatting) must be integrated into the development workflow. All implementations must pass security audits before production deployment.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
