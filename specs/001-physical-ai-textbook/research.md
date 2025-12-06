# Research Summary: Physical AI & Humanoid Robotics Textbook

## Tech Stack Decisions

### Frontend: Docusaurus v3 (React, TypeScript, Tailwind CSS)
- **Decision**: Use Docusaurus v3 as the static site generator for the textbook
- **Rationale**: Docusaurus is specifically designed for documentation sites, offers excellent Markdown/MDX support, built-in search, and is ideal for educational content. The v3 version provides modern React features and TypeScript support.
- **Alternatives considered**:
  - Next.js with custom documentation setup - more complex, requires more custom code
  - Gatsby - also good but Docusaurus has better built-in documentation features
  - VuePress - good but smaller ecosystem than Docusaurus

### Auth: better-auth (with PostgreSQL via Neon Serverless)
- **Decision**: Use better-auth for authentication with Neon PostgreSQL backend
- **Rationale**: better-auth provides a simple, secure authentication solution that works well with Docusaurus. Neon Serverless offers PostgreSQL with serverless scaling, which is cost-effective for this educational platform.
- **Alternatives considered**:
  - Auth.js (NextAuth.js) - would require backend API routes, more complex for Docusaurus
  - Clerk - good but adds external dependency and cost
  - Custom auth solution - more complex, security concerns

### Backend: FastAPI (for RAG + personalization + translation APIs)
- **Decision**: Use FastAPI for backend services
- **Rationale**: FastAPI provides excellent performance, automatic API documentation, strong typing, and async support. It's ideal for the RAG chatbot, personalization, and translation APIs.
- **Alternatives considered**:
  - Express.js - JavaScript ecosystem but slower than FastAPI
  - Django - more complex, heavier than needed
  - Flask - less performant and modern than FastAPI

### Vector DB: Qdrant Cloud (free tier)
- **Decision**: Use Qdrant Cloud for vector storage and similarity search
- **Rationale**: Qdrant is designed for vector similarity search, has good performance, and supports the OpenAI Agents SDK well. The free tier should be sufficient for initial development.
- **Alternatives considered**:
  - Pinecone - good but paid service from the start
  - Supabase Vector - good but Qdrant has more mature vector search features
  - Weaviate - good alternative but Qdrant has better documentation for this use case

### AI: OpenAI Agents SDK (for RAG agent), Whisper (optional for voice later)
- **Decision**: Use OpenAI Agents SDK for RAG functionality
- **Rationale**: The OpenAI Agents SDK provides a structured way to build RAG systems with conversation management, which is exactly what's needed for the textbook chatbot.
- **Alternatives considered**:
  - LangChain - good but more complex than needed
  - LlamaIndex - good but OpenAI Agents SDK is more focused on conversation
  - Custom RAG implementation - more complex, reinventing the wheel

### Deployment: GitHub Pages (frontend), Render/Vercel (FastAPI), Neon (DB)
- **Decision**: Use GitHub Pages for frontend, Render/Vercel for backend API, Neon for database
- **Rationale**: GitHub Pages is perfect for static Docusaurus site, cost-effective. Render/Vercel are good for hosting FastAPI applications. Neon provides serverless PostgreSQL.
- **Alternatives considered**:
  - Netlify - also good for frontend but GitHub Pages integrates well with GitHub workflow
  - AWS/Azure - more complex and expensive for this project

### Personalization: Store user preferences in Neon DB; adjust content via React context
- **Decision**: Store personalization data in Neon DB with React context for frontend state management
- **Rationale**: This approach provides persistent user preferences while maintaining responsive UI updates.
- **Alternatives considered**:
  - Local storage only - preferences not persistent across devices
  - Separate user preference service - over-engineering for this use case

### Urdu Translation: Use DeepL or OpenAI GPT-4o with Roman Urdu output
- **Decision**: Use OpenAI GPT-4o for translation to Roman Urdu
- **Rationale**: GPT-4o has good multilingual capabilities and can be prompted to output in Roman Urdu script. This aligns with the tech stack.
- **Alternatives considered**:
  - DeepL API - good but adds another dependency
  - Google Translate API - good but GPT-4o fits the existing OpenAI integration
  - Manual translation - not scalable

### CI: GitHub Actions for build & deploy
- **Decision**: Use GitHub Actions for CI/CD
- **Rationale**: GitHub Actions integrates well with the GitHub workflow and can handle both frontend and backend deployments.
- **Alternatives considered**:
  - Other CI tools - would require additional setup and integration

## Directory Structure

### Final Structure
```
/website → Docusaurus frontend
/backend → FastAPI backend
/rag → Qdrant + OpenAI Agent logic
/auth → better-auth integration
```

This structure separates concerns clearly while maintaining good organization for the multi-component application.