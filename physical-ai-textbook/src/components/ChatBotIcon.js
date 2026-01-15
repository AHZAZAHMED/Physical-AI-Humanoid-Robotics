import React, { useState } from 'react';
import './ChatBotIcon.css';

const ChatBotIcon = ({ onClick }) => {
  const [isVisible, setIsVisible] = useState(true);

  const handleClick = () => {
    onClick();
  };

  return (
    <div className={`chatbot-icon ${isVisible ? 'visible' : 'hidden'}`}>
      <button
        className="chatbot-button"
        onClick={handleClick}
        aria-label="Open chatbot"
        title="Chat with our AI assistant"
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          className="chatbot-icon-svg"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
        </svg>
      </button>
    </div>
  );
};

export default ChatBotIcon;