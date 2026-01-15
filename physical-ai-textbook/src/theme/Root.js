import React, { useState } from 'react';
import ChatBotIcon from '../components/ChatBotIcon';
import ChatBox from '../components/ChatBox';

// Default implementation, that you can customize
function Root({ children }) {
  const [isChatOpen, setIsChatOpen] = useState(false);

  const handleChatIconClick = () => {
    setIsChatOpen(true);
  };

  const handleCloseChat = () => {
    setIsChatOpen(false);
  };

  const handleSendMessage = async (message) => {
    try {
      // Send message to backend API
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: message,
          sessionId: localStorage.getItem('chatSessionId') || Date.now().toString()
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(`HTTP error! status: ${response.status}, message: ${errorData.detail || response.statusText}`);
      }

      const data = await response.json();

      // Store session ID for continuity
      if (data.sessionId) {
        localStorage.setItem('chatSessionId', data.sessionId);
      }

      // Log the response for debugging
      console.log('Chat response:', data);

      return data;
    } catch (error) {
      console.error('Error sending message:', error);
      // Return an error response structure that the frontend can handle
      return {
        response: "Sorry, I'm having trouble connecting to the AI service right now. Please try again later.",
        sources: [],
        sessionId: localStorage.getItem('chatSessionId') || Date.now().toString(),
        error: {
          message: error.message,
          type: 'connection_error'
        }
      };
    }
  };

  return (
    <>
      {children}
      <ChatBotIcon onClick={handleChatIconClick} />
      <ChatBox
        isOpen={isChatOpen}
        onClose={handleCloseChat}
        onSendMessage={handleSendMessage}
      />
    </>
  );
}

export default Root;