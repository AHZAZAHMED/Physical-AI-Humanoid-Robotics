"""
Agent connector module to interface with the existing RAG agent.

This module provides a clean interface between the API endpoints and the RAG agent,
handling request validation, error propagation, and response formatting.
"""
from typing import Dict, Any, Optional
from src.shared_types import ChatRequest, ChatResponse, Source, ErrorResponse
import logging
from datetime import datetime

logger = logging.getLogger(__name__)


class AgentConnector:
    """
    Connector class to interface with the existing RAG agent.
    Provides a clean abstraction layer between API endpoints and the RAG agent.
    """

    def __init__(self):
        """Initialize the agent connector without instantiating the RAG agent."""
        self._agent = None
        self._config = None

    def _get_agent(self):
        """Lazy load the RAG agent to avoid import issues."""
        if self._agent is None:
            from src.rag_agent.config import AgentConfiguration

            # Try to initialize the real RAG agent
            try:
                from src.rag_agent.agent import RAGAgent
                self._config = AgentConfiguration()
                self._agent = RAGAgent(self._config)
                print("Successfully initialized the real RAG agent")
            except Exception as e:
                # Log the actual error for debugging
                print(f"Error initializing RAG agent: {e}")

                # Check if it's a specific known error that should use the mock
                if isinstance(e, AttributeError) and ("'tensorflow' has no attribute 'contrib'" in str(e) or "tf.contrib" in str(e)):
                    print("TensorFlow compatibility issue detected")
                elif "agents" in str(e).lower() or "openai" in str(e).lower() or "genai" in str(e).lower():
                    print("Agents package or API configuration issue detected")

                print("Creating a mock agent that returns sample responses")

                # Create a mock agent for testing purposes
                class MockRAGAgent:
                    def process_query(self, query_text):
                        # Create the response text beforehand to avoid scoping issues
                        response_text = f"I received your query: '{query_text}'. This is a mock response since there's an issue with the agents package. The actual RAG system would provide detailed information about Physical AI and Humanoid Robotics based on the textbook content."

                        # Return a mock response object that matches expected structure
                        class MockResponse:
                            def __init__(self, text, sources_data, time_ms):
                                self.text = text
                                self.sources = sources_data
                                self.response_time_ms = time_ms

                        sources_data = [{
                            'content': 'Sample content from textbook about Physical AI',
                            'source': {
                                'book': 'Physical AI & Humanoid Robotics Textbook',
                                'chapter': 'Introduction',
                                'section': 'Overview'
                            },
                            'similarity_score': 0.95
                        }]

                        return MockResponse(response_text, sources_data, 100)

                self._agent = MockRAGAgent()
        return self._agent

    def process_chat_request(self, chat_request: ChatRequest) -> ChatResponse:
        """
        Process a chat request through the RAG agent.

        Args:
            chat_request: The incoming chat request with query and session info

        Returns:
            ChatResponse with the agent's response and sources
        """
        try:
            # Get the agent (lazy loading)
            agent = self._get_agent()


            # Process the query through the RAG agent
            # For async methods, we need to handle them properly without nest_asyncio
            import asyncio

            # Check if the method is async
            if asyncio.iscoroutinefunction(agent.process_query):
                # If it's a coroutine, run it properly in the current event loop
                try:
                    loop = asyncio.get_running_loop()
                    # If we're already in a loop, run in executor or create task
                    import concurrent.futures
                    with concurrent.futures.ThreadPoolExecutor() as executor:
                        response = executor.submit(lambda: agent.process_query(chat_request.query)).result()
                except RuntimeError:
                    # No event loop running, safe to use asyncio.run
                    response = asyncio.run(agent.process_query(chat_request.query))
            else:
                # If it's not async, call directly
                response = agent.process_query(chat_request.query)

            # Format the response according to our schema
            sources = []
            if hasattr(response, 'sources') and response.sources:
                for source in response.sources:
                    source_obj = Source(
                        content=source.get('content', '')[:500],  # Limit content length
                        metadata=source.get('source', {}),
                        similarityScore=source.get('similarity_score'),
                        bookTitle=source.get('source', {}).get('book', 'Unknown Book')
                    )
                    sources.append(source_obj)

            # Generate a session ID if not provided
            session_id = chat_request.sessionId or self._generate_session_id()

            return ChatResponse(
                response=response.text if hasattr(response, 'text') else str(response),
                sources=sources,
                sessionId=session_id,
                timestamp=datetime.now().isoformat()  # Always use proper string format
            )

        except Exception as e:
            logger.error(f"Error processing chat request: {str(e)}", exc_info=True)
            return ChatResponse(
                response="",
                sources=[],
                sessionId=chat_request.sessionId or self._generate_session_id(),
                error={
                    "message": f"Error processing request: {str(e)}",
                    "type": type(e).__name__
                }
            )

    def _generate_session_id(self) -> str:
        """Generate a simple session ID for tracking conversations."""
        import uuid
        return str(uuid.uuid4())