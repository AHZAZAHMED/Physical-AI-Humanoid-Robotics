import React, { useState, useRef, useEffect } from 'react';
import './ChatBox.css';

const ChatBox = ({ isOpen, onClose, onSendMessage }) => {
  const [messages, setMessages] = useState([]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    // Use a safer scroll method to avoid ResizeObserver issues
    if (messagesEndRef.current) {
      // Use instant scrolling instead of smooth to prevent layout thrashing
      messagesEndRef.current.scrollIntoView({ behavior: 'instant' });
    }
  };

  const handleSend = async () => {
    if (!inputText.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputText,
      sender: 'user',
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Send to backend API
      const response = await onSendMessage(inputText);

      // Add bot response
      const botMessage = {
        id: Date.now() + 1,
        text: response.response || '',
        sources: response.sources || [],
        error: response.error, // Include error if present
        sender: 'bot',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      // Add error message
      const errorMessage = {
        id: Date.now() + 1,
        text: 'Sorry, I encountered an error. Please try again.',
        sender: 'bot',
        isError: true,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  if (!isOpen) return null;

  return (
    <div className="chatbox-overlay-non-modal">
      <div className="chatbox-container" role="document">
        <div className="chatbox-header">
          <h3 id="chatbox-title">AI Assistant</h3>
          <button
            className="chatbox-close-btn"
            onClick={onClose}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>

        <div className="chatbox-messages">
          {messages.length === 0 ? (
            <div className="chatbox-welcome">
              <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook. How can I help you today?</p>
            </div>
          ) : (
            messages.map((message) => (
              <div
                key={message.id}
                className={`chatbox-message ${
                  message.sender === 'user' ? 'user-message' : 'bot-message'
                } ${message.isError ? 'error-message' : ''}`}
              >
                <div className="message-content">
                  <span className="sender-indicator">
                    {message.sender === 'user' ? 'You:' : 'AI:'}
                  </span>
                  <div className="message-text">
                    {message.text || (message.error ? message.error.message || 'An error occurred' : 'No response text')}
                  </div>

                  {message.sources && message.sources.length > 0 && (
                    <div className="message-sources">
                      <strong>Sources:</strong>
                      <ul>
                        {message.sources.slice(0, 3).map((source, idx) => (
                          <li key={idx}>
                            {source.metadata?.book_title || 'Reference'} - {source.metadata?.section || 'Section'}
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}

                  {message.error && (
                    <div className="error-details">
                      <strong>Error:</strong> {message.error.message}
                    </div>
                  )}
                </div>
              </div>
            ))
          )}

          {isLoading && (
            <div className="chatbox-message bot-message">
              <div className="message-content">
                <span className="sender-indicator">AI:</span>
                <div className="loading-indicator">AI is thinking...</div>
              </div>
            </div>
          )}

          <div ref={messagesEndRef} />
        </div>

        <div className="chatbox-input-area">
          <textarea
            id="chat-input"
            value={inputText}
            onChange={(e) => setInputText(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Type your question about the textbook..."
            disabled={isLoading}
            rows={2}
            aria-label="Type your message"
            aria-describedby="chat-input-help"
          />
          <button
            onClick={handleSend}
            disabled={!inputText.trim() || isLoading}
            className="send-button"
            aria-label="Send message"
            aria-describedby="chat-input"
          >
            {isLoading ? 'Sending...' : 'Send'}
          </button>
          <div id="chat-input-help" className="sr-only">
            Press Enter to send, Shift+Enter for new line
          </div>
        </div>
      </div>
    </div>
  );
};

export default ChatBox;