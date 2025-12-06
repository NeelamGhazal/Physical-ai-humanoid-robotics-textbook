from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
from backend.src.config import settings
import uuid


class QdrantService:
    def __init__(self):
        if settings.QDRANT_URL:
            # Connect to Qdrant Cloud
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                prefer_grpc=True
            )
        else:
            # Connect to local Qdrant instance (for development)
            self.client = QdrantClient(host="localhost", port=6333)

        # Collection name for textbook content
        self.collection_name = "textbook_content"

        # Initialize the collection if it doesn't exist
        self._init_collection()

    def _init_collection(self):
        """Initialize the Qdrant collection for textbook content"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Create collection if it doesn't exist
            self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # Using OpenAI embedding size
            )

    def add_textbook_content(self, content_id: str, content: str, metadata: Dict):
        """Add textbook content to the vector store"""
        from openai import OpenAI
        client = OpenAI(api_key=settings.OPENAI_API_KEY)

        # Create embedding for the content
        response = client.embeddings.create(
            input=content,
            model="text-embedding-ada-002"  # or another suitable embedding model
        )
        embedding = response.data[0].embedding

        # Add to Qdrant
        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=content_id,
                    vector=embedding,
                    payload={
                        "content": content,
                        "metadata": metadata
                    }
                )
            ]
        )

    def search_similar_content(self, query: str, limit: int = 5) -> List[Dict]:
        """Search for similar content in the vector store"""
        from openai import OpenAI
        client = OpenAI(api_key=settings.OPENAI_API_KEY)

        # Create embedding for the query
        response = client.embeddings.create(
            input=query,
            model="text-embedding-ada-002"
        )
        query_embedding = response.data[0].embedding

        # Search in Qdrant
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        # Extract content and metadata from results
        results = []
        for result in search_results:
            results.append({
                "id": result.id,
                "content": result.payload["content"],
                "metadata": result.payload["metadata"],
                "score": result.score
            })

        return results

    def delete_content(self, content_id: str):
        """Delete specific content from the vector store"""
        self.client.delete(
            collection_name=self.collection_name,
            points_selector=models.PointIdsList(
                points=[content_id]
            )
        )

    def update_content(self, content_id: str, content: str, metadata: Dict):
        """Update existing content in the vector store"""
        # First delete the old content
        self.delete_content(content_id)

        # Then add the updated content
        self.add_textbook_content(content_id, content, metadata)


# Global instance
qdrant_service = QdrantService()