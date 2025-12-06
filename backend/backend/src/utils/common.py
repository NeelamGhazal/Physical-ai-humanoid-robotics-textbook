import hashlib
import uuid
from typing import Any, Dict
from datetime import datetime, timedelta
import re


def generate_content_hash(content: str) -> str:
    """Generate a hash for content to use as identifier or for caching"""
    return hashlib.sha256(content.encode()).hexdigest()


def generate_uuid() -> str:
    """Generate a UUID string"""
    return str(uuid.uuid4())


def sanitize_input(text: str) -> str:
    """Basic input sanitization"""
    # Remove potentially harmful characters/sequences
    sanitized = re.sub(r'<script[^>]*>.*?</script>', '', text, flags=re.IGNORECASE | re.DOTALL)
    sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
    return sanitized.strip()


def validate_email(email: str) -> bool:
    """Basic email validation"""
    pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    return re.match(pattern, email) is not None


def format_duration(seconds: int) -> str:
    """Format duration in seconds to human readable format"""
    if seconds < 60:
        return f"{seconds}s"
    elif seconds < 3600:
        minutes = seconds // 60
        return f"{minutes}m {seconds % 60}s"
    else:
        hours = seconds // 3600
        minutes = (seconds % 3600) // 60
        return f"{hours}h {minutes}m {seconds % 60}s"


def create_response(success: bool, data: Any = None, message: str = "", error: str = "") -> Dict:
    """Standard response format"""
    response = {"success": success}
    if data is not None:
        response["data"] = data
    if message:
        response["message"] = message
    if error:
        response["error"] = error
    return response


def get_current_timestamp() -> datetime:
    """Get current timestamp"""
    return datetime.utcnow()


def calculate_age(birth_date: datetime) -> int:
    """Calculate age from birth date"""
    today = datetime.utcnow().date()
    birth = birth_date.date()
    return today.year - birth.year - ((today.month, today.day) < (birth.month, birth.day))