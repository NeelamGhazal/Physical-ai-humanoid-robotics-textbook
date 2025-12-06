# Physical AI & Humanoid Robotics Textbook

A comprehensive textbook on Physical AI & Humanoid Robotics built with Docusaurus and FastAPI.

## Overview

This project implements a professional-grade AI-native textbook on Physical AI & Humanoid Robotics with:

- Docusaurus frontend for textbook content
- FastAPI backend for dynamic features
- RAG chatbot for intelligent Q&A
- User authentication and personalization
- Multi-language support (English/Roman Urdu)
- Interactive learning elements

## Features

- **Modular Content**: Organized into 4 core modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- **Interactive Learning**: Code snippets, diagrams, and quizzes
- **Personalization**: Adjust content depth and language preference
- **AI-Powered Q&A**: RAG chatbot for textbook-related questions
- **Multi-Language Support**: English and Roman Urdu
- **Responsive Design**: Works on desktop and mobile devices

## Tech Stack

### Frontend
- **Framework**: Docusaurus v3
- **Language**: TypeScript/JavaScript
- **Styling**: Tailwind CSS

### Backend
- **Framework**: FastAPI
- **Database**: PostgreSQL (Neon Serverless)
- **Vector DB**: Qdrant Cloud
- **AI**: OpenAI Agents SDK
- **Authentication**: better-auth

### Infrastructure
- **Frontend Hosting**: GitHub Pages
- **Backend Hosting**: Render/Vercel
- **CI/CD**: GitHub Actions

## Project Structure

```
├── website/                 # Docusaurus frontend
│   ├── docs/               # Textbook content
│   ├── src/                # Custom components
│   ├── static/             # Static assets
│   └── docusaurus.config.js # Configuration
├── backend/                # FastAPI backend
│   ├── src/
│   │   ├── main.py         # Application entry point
│   │   ├── api/            # API routes
│   │   ├── services/       # Business logic
│   │   ├── models/         # Pydantic models
│   │   ├── database/       # Database models
│   │   └── rag/            # RAG functionality
│   └── requirements.txt
├── rag/                    # RAG-specific logic
├── auth/                   # Authentication logic
└── specs/                  # Project specifications
```

## Getting Started

### Prerequisites

- Node.js 18+
- Python 3.11+
- Docker (optional)

### Frontend Setup

```bash
cd website
npm install
npm start
```

### Backend Setup

```bash
cd backend
pip install -r requirements.txt
uvicorn src.main:app --reload
```

## Environment Variables

Create `.env` files in both frontend and backend directories with the necessary configuration.

## Deployment

### Frontend (GitHub Pages)

```bash
npm run build
npm run deploy
```

### Backend

Deploy to Render or Vercel with the appropriate configurations.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

[Specify your license here]