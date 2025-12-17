"""Query classification and greeting detection."""
from typing import Dict, Any


class QueryClassifier:
    """Classifies user queries and detects greetings."""

    GREETINGS = {
        'hi', 'hello', 'hey', 'hii', 'hiii', 'hiiii',
        'salam', 'assalam', 'assalamualaikum',
        'good morning', 'good afternoon', 'good evening',
        'morning', 'afternoon', 'evening',
        'whatsup', "what's up", 'sup', 'yo',
        'how are you', 'how r u', 'how are u',
        'kaise ho', 'kya hal hai', 'kese ho',
        'namaste', 'namaskar'
    }

    GREETING_RESPONSES = [
        "Hello! I'm your AI assistant for Physical AI & Humanoid Robotics. How can I help you today?",
        "Hi there! Ask me anything about robotics, AI, or the textbook content.",
        "Hey! I'm here to help you learn about Physical AI and Humanoid Robotics. What would you like to know?",
        "Salam! I can answer your questions about robotics, AI systems, and humanoid design. What interests you?",
        "Namaste! Ready to explore the world of Physical AI and robotics with you. What's your question?"
    ]

    @staticmethod
    def is_greeting(query: str) -> bool:
        """Check if query is a greeting.

        Args:
            query: User's input text

        Returns:
            True if query is a greeting
        """
        query_lower = query.lower().strip()

        # Remove punctuation
        query_clean = query_lower.replace('!', '').replace('?', '').replace('.', '').replace(',', '')

        # Check exact match
        if query_clean in QueryClassifier.GREETINGS:
            return True

        # Check if query is very short (<=5 chars) and contains common greeting words
        if len(query_clean) <= 5:
            greeting_words = ['hi', 'hey', 'sup', 'yo']
            if any(word in query_clean for word in greeting_words):
                return True

        return False

    @staticmethod
    def get_greeting_response(query: str) -> Dict[str, Any]:
        """Get greeting response.

        Args:
            query: User's greeting

        Returns:
            Response dict with greeting
        """
        import random

        response_text = random.choice(QueryClassifier.GREETING_RESPONSES)

        return {
            "answer": response_text,
            "status": "greeting"
        }

    @staticmethod
    def is_too_short(query: str) -> bool:
        """Check if query is too short to be meaningful.

        Args:
            query: User's input

        Returns:
            True if query is too short
        """
        # Remove whitespace and check length
        clean_query = query.strip()

        # If less than 3 characters (excluding greetings which are handled separately)
        if len(clean_query) < 3:
            return True

        # If only 1-2 words and very short
        words = clean_query.split()
        if len(words) <= 2 and all(len(word) < 3 for word in words):
            return True

        return False
