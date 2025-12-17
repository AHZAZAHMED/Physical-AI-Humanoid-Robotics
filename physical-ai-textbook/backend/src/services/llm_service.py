"""
LLM Service for the Physical AI Textbook Platform
Handles integration with Google Gemini API for response generation
"""
import google.generativeai as genai
import os
import logging
from typing import List, Dict, Any, Optional
from pydantic import BaseModel

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class LLMService:
    """
    Service class for LLM operations using Google Gemini
    """

    def __init__(self):
        # Get API key from environment
        api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("GEMINI_API_KEY environment variable is required")

        # Configure the API key
        genai.configure(api_key=api_key)

        # Initialize the model (using gemini-pro as recommended)
        self.model = genai.GenerativeModel('gemini-pro')

        logger.info("LLM Service initialized with Google Gemini API")

    def generate_response(self,
                         prompt: str,
                         context: Optional[str] = None,
                         conversation_history: Optional[List[Dict[str, str]]] = None) -> str:
        """
        Generate a response using the LLM with provided context and conversation history

        Args:
            prompt: The main question or prompt from the user
            context: Retrieved context from the vector database
            conversation_history: List of previous conversation turns

        Returns:
            Generated response from the LLM
        """
        try:
            # Build the full prompt with context and conversation history
            full_prompt = self._build_prompt(prompt, context, conversation_history)

            # Generate content using the model
            response = self.model.generate_content(full_prompt)

            # Return the text response
            return response.text if response.text else "I couldn't generate a response for your query."

        except Exception as e:
            logger.error(f"Error calling Gemini API: {str(e)}")
            # Return a fallback response
            return self._get_fallback_response(prompt)

    def _build_prompt(self,
                     prompt: str,
                     context: Optional[str] = None,
                     conversation_history: Optional[List[Dict[str, str]]] = None) -> str:
        """
        Build a comprehensive prompt for the LLM with context and conversation history

        Args:
            prompt: The main question or prompt from the user
            context: Retrieved context from the vector database
            conversation_history: List of previous conversation turns

        Returns:
            Formatted prompt string for the LLM
        """
        # Start with the main prompt
        parts = [f"User's question: {prompt}"]

        # Add conversation history if available
        if conversation_history:
            history_text = "\n\nPrevious conversation:\n"
            for turn in conversation_history[-5:]:  # Use last 5 turns to avoid exceeding token limits
                role = "User" if turn.get('role') == 'user' else 'Assistant'
                history_text += f"{role}: {turn.get('content', '')}\n"
            parts.append(history_text)

        # Add retrieved context if available
        if context:
            parts.append(f"\n\nRelevant textbook content:\n{context}")

        # Add instruction for response format
        parts.append("\n\nPlease provide a detailed, educational response that directly addresses the user's question. Structure your response as a tutor would:")
        parts.append("1. Start with a clear, direct answer to the question")
        parts.append("2. Provide detailed explanations with examples where relevant")
        parts.append("3. Connect concepts to broader principles in Physical AI and Humanoid Robotics")
        parts.append("4. If applicable, mention related topics the student might want to explore")
        parts.append("5. Use clear, educational language appropriate for textbook learners")
        parts.append("6. If you reference specific information from the provided context, please cite it appropriately")

        return "\n".join(parts)

    def _get_fallback_response(self, prompt: str) -> str:
        """
        Generate a fallback response when the LLM API is unavailable

        Args:
            prompt: The original user prompt

        Returns:
            Fallback response
        """
        return f"I'm having trouble generating a detailed response right now. Based on your question '{prompt}', I recommend checking the relevant sections in the Physical AI & Humanoid Robotics textbook for more information. Please try again later when the service is available."

    def generate_tutor_response(self,
                               question: str,
                               context: str = None,
                               sources: List[str] = None) -> str:
        """
        Generate a tutor-style response specifically for educational purposes

        Args:
            question: The question from the student
            context: Retrieved context from the vector database
            sources: List of sources used in the response

        Returns:
            Tutor-style educational response
        """
        try:
            # Build tutor-specific prompt
            tutor_prompt = f"""
            You are an expert tutor in Physical AI and Humanoid Robotics.
            Please answer the following question: "{question}"
            """

            if context:
                tutor_prompt += f"\n\nUse the following textbook information to inform your response:\n{context}"

            tutor_prompt += """

            Please structure your response as a tutor would:
            1. Start with a clear, direct answer to the question
            2. Provide detailed explanations with examples where relevant
            3. Connect concepts to broader principles in Physical AI and Humanoid Robotics
            4. If applicable, mention related topics the student might want to explore
            5. Use clear, educational language appropriate for textbook learners
            6. If you reference specific information from the provided context, please cite it appropriately
            """

            response = self.model.generate_content(tutor_prompt)
            return response.text if response.text else "I couldn't generate a response for your query."

        except Exception as e:
            logger.error(f"Error generating tutor response: {str(e)}")
            return self._get_fallback_response(question)


# Global instance for use in other modules
llm_service = LLMService()


def get_llm_response(question: str, context: str = None, sources: List[str] = None) -> str:
    """
    Convenience function to get LLM response
    """
    return llm_service.generate_tutor_response(question, context, sources)