# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Prerequisites

- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for FastAPI backend)
- Docker (for local development, optional)
- Git
- Access to OpenAI API key
- Access to Qdrant Cloud account
- Neon PostgreSQL account

## Project Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Frontend Setup (Docusaurus)

```bash
# Navigate to website directory
cd website

# Install dependencies
npm install

# Create environment file
cp .env.example .env

# Update .env with your configuration
# (Currently no backend API configured, will be added during backend setup)
```

### 3. Backend Setup (FastAPI)

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Create environment file
cp .env.example .env

# Update .env with your configuration:
# - OPENAI_API_KEY (for RAG chatbot)
# - QDRANT_URL (for vector database)
# - QDRANT_API_KEY (if using cloud version)
# - DATABASE_URL (for Neon PostgreSQL)
# - JWT_SECRET_KEY (for authentication)
```

## Environment Configuration

### Frontend (.env in website/)

```env
# Backend API URL (will be updated when backend is running)
REACT_APP_API_URL=http://localhost:8000
NEXT_PUBLIC_API_URL=http://localhost:8000
```

### Backend (.env in backend/)

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here
OPENAI_MODEL=gpt-4o  # or your preferred model

# Qdrant Configuration
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key  # if using cloud version

# Database Configuration
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname

# Authentication Configuration
JWT_SECRET_KEY=your_jwt_secret_key_here
JWT_ALGORITHM=HS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=30

# Rate Limiting
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=3600  # 1 hour in seconds
```

## Running the Application

### 1. Start the Backend

```bash
# From the backend directory
cd backend
source venv/bin/activate  # Activate virtual environment

# Run the backend server
uvicorn src.main:app --reload --port 8000
```

### 2. Start the Frontend

```bash
# From the website directory
cd website

# Start the Docusaurus development server
npm run start
```

The application will be available at `http://localhost:3000` with the backend API at `http://localhost:8000`.

## Database Setup

### 1. Install Alembic (if not in requirements)

```bash
pip install alembic
```

### 2. Initialize Database

```bash
# From the backend directory
cd backend
source venv/bin/activate

# Run database migrations
alembic upgrade head
```

## Qdrant Setup

### 1. Create Collections

The application will automatically create required Qdrant collections on first run. For manual setup:

```python
from qdrant_client import QdrantClient
from qdrant_client.http import models

client = QdrantClient(url="your-qdrant-url", api_key="your-api-key")

# Create collection for textbook content
client.recreate_collection(
    collection_name="textbook_content",
    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
)
```

## Loading Textbook Content

### 1. Prepare Content

Content should be organized in the `website/docs/` directory in the following structure:

```
website/docs/
├── ros-2/
│   ├── introduction.md
│   ├── installation.md
│   └── basic-concepts.md
├── gazebo-unity/
│   ├── simulation-basics.md
│   └── physics-engines.md
├── nvidia-isaac/
│   └── robotics-framework.md
└── vla/
    └── vision-language-actions.md
```

### 2. Load Content to Vector Database

Run the content loader script:

```bash
# From the backend directory
cd backend
source venv/bin/activate

# Run the content loader
python -m src.rag.document_loader
```

## Testing the Application

### Backend Tests

```bash
# From the backend directory
cd backend
source venv/bin/activate

# Run tests
pytest
```

### Frontend Tests

```bash
# From the website directory
cd website

# Run tests
npm test
```

## Deployment

### Frontend (GitHub Pages)

```bash
# Build the static site
npm run build

# Deploy to GitHub Pages
npm run deploy
```

### Backend (Render/Vercel)

1. Push code to GitHub repository
2. Connect Render/Vercel to the repository
3. Set environment variables in the deployment platform
4. Deploy the application

## API Documentation

Once the backend is running, API documentation is available at:
- `http://localhost:8000/docs` (Swagger UI)
- `http://localhost:8000/redoc` (ReDoc)

## Troubleshooting

### Common Issues

1. **Backend not connecting to database**:
   - Verify DATABASE_URL is correct in backend/.env
   - Check Neon PostgreSQL connection settings

2. **OpenAI API errors**:
   - Verify OPENAI_API_KEY is correct in backend/.env
   - Check API usage limits in your OpenAI account

3. **Qdrant connection issues**:
   - Verify QDRANT_URL and QDRANT_API_KEY in backend/.env
   - Check Qdrant cluster status

4. **Frontend can't connect to backend**:
   - Ensure both services are running
   - Verify REACT_APP_API_URL in website/.env points to the correct backend URL