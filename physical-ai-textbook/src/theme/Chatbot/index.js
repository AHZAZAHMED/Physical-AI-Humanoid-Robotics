import React, { useState, useEffect, useCallback } from 'react';

import styles from './styles.module.css';

// Helper function to get current color mode from localStorage or system preference
const getColorMode = () => {
  if (typeof window !== 'undefined') {
    const storedPreference = localStorage.getItem('theme');
    if (storedPreference) {
      return storedPreference === 'dark';
    }
    return window.matchMedia('(prefers-color-scheme: dark)').matches;
  }
  return false; // default to light theme
};

const ChatbotInner = ({ initialIsDarkTheme }) => {
  const [isDarkTheme, setIsDarkTheme] = useState(initialIsDarkTheme);
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Listen for color mode changes
  useEffect(() => {
    const updateColorMode = () => {
      const newIsDarkTheme = getColorMode();
      setIsDarkTheme(newIsDarkTheme);
    };

    // Listen for storage changes (when theme is changed elsewhere)
    const handleStorageChange = (e) => {
      if (e.key === 'theme') {
        updateColorMode();
      }
    };

    // Listen for system preference changes
    const mediaQuery = window.matchMedia('(prefers-color-scheme: dark)');
    const handleSystemChange = () => {
      // Only update if no user preference is set (i.e., theme is not in localStorage)
      if (!localStorage.getItem('theme')) {
        updateColorMode();
      }
    };

    // Set up listeners
    window.addEventListener('storage', handleStorageChange);
    mediaQuery.addEventListener('change', handleSystemChange);

    // Clean up listeners
    return () => {
      window.removeEventListener('storage', handleStorageChange);
      mediaQuery.removeEventListener('change', handleSystemChange);
    };
  }, []);

  // Initialize with a welcome message
  useEffect(() => {
    setMessages([
      {
        id: 1,
        text: "Hello! I'm your Physical AI & Humanoid Robotics assistant. Ask me anything about the textbook content.",
        isBot: true,
      }
    ]);
  }, []);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      text: inputValue,
      isBot: false,
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // In a real implementation, this would call your backend API
      // For now, we'll simulate a response
      const response = await simulateBackendResponse(inputValue);

      const botMessage = {
        id: Date.now() + 1,
        text: response,
        isBot: true,
      };

      setMessages((prev) => [...prev, botMessage]);
    } catch (error) {
      const errorMessage = {
        id: Date.now() + 1,
        text: "Sorry, I encountered an error processing your request. Please try again.",
        isBot: true,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  // Simulate backend response - in real implementation, call your API
  const simulateBackendResponse = async (question) => {
    // Simulate API delay
    await new Promise(resolve => setTimeout(resolve, 1000));

    // Simple response logic - in real implementation, call your backend API
    const responses = [
      "Based on the textbook content, this topic covers important concepts in humanoid robotics.",
      "That's an interesting question about Physical AI! The textbook covers this in detail.",
      "I found relevant information in the textbook: This topic is covered in Module 2 on Simulation.",
      "According to the Physical AI & Humanoid Robotics textbook, this concept is fundamental to understanding robot perception.",
      "Great question! The textbook explains this concept in Module 3: NVIDIA Isaac Platform."
    ];

    return responses[Math.floor(Math.random() * responses.length)];
  };

  return (
    <>
      {/* Floating Chat Button */}
      {!isOpen && (
        <button
          className={`${styles.chatButton} ${isDarkTheme ? styles.dark : styles.light}`}
          onClick={toggleChat}
          aria-label="Open chat"
        >
          <svg
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 24 24"
            fill="currentColor"
            className={styles.chatIcon}
          >
            <path d="M4.913 2.658c2.075-.27 4.19-.408 6.337-.408 2.147 0 4.262.139 6.337.408 1.922.25 3.291 1.861 3.405 3.727a4.403 4.403 0 00-1.032-.211 50.89 50.89 0 00-8.42 0c-2.358.196-4.04 2.19-4.04 4.434v4.286a4.47 4.47 0 002.433 3.984L7.28 21.53A.75.75 0 016 21v-4.03a48.527 48.527 0 01-1.087-.128C2.905 16.58 1.5 14.833 1.5 12.862V6.638c0-1.97 1.405-3.718 3.413-3.979z" />
            <path d="M15.75 7.5c-1.376 0-2.739.057-4.086.169C10.124 7.797 9 9.103 9 10.609v4.285c0 1.507 1.128 2.814 2.67 2.94 1.243.102 2.5.157 3.768.165l2.782 2.781a.75.75 0 001.28-.53v-2.39l.33-.026c1.542-.125 2.67-1.433 2.67-2.94v-4.286c0-1.505-1.125-2.811-2.664-2.94A49.392 49.392 0 0015.75 7.5z" />
          </svg>
        </button>
      )}

      {/* Chat Window */}
      {isOpen && (
        <div className={`${styles.chatContainer} ${isDarkTheme ? styles.dark : styles.light}`}>
          <div className={styles.chatHeader}>
            <h3>Physical AI Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${
                  message.isBot ? styles.botMessage : styles.userMessage
                }`}
              >
                <div className={styles.messageText}>{message.text}</div>
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.botMessage}`}>
                <div className={styles.messageText}>
                  <div className={styles.typingIndicator}>
                    <div className={styles.dot}></div>
                    <div className={styles.dot}></div>
                    <div className={styles.dot}></div>
                  </div>
                </div>
              </div>
            )}
          </div>

          <div className={styles.chatInputContainer}>
            <textarea
              value={inputValue}
              onChange={handleInputChange}
              onKeyPress={handleKeyPress}
              placeholder="Ask about Physical AI & Humanoid Robotics..."
              className={styles.chatInput}
              rows="1"
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              className={`${styles.sendButton} ${
                !inputValue.trim() || isLoading ? styles.disabled : ''
              }`}
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                viewBox="0 0 24 24"
                fill="currentColor"
                className={styles.sendIcon}
              >
                <path d="M3.478 2.405a.75.75 0 00-.926.94l2.432 7.905H13.5a.75.75 0 010 1.5H4.984l-2.432 7.905a.75.75 0 00.926.94 60.519 60.519 0 0018.445-8.986.75.75 0 000-1.218A60.517 60.517 0 003.478 2.405z" />
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
};

// For use in Root.js where the color mode context is not available
// This version uses direct DOM access and event listeners to track theme changes
const Chatbot = () => {
  const initialIsDarkTheme = getColorMode();
  return <ChatbotInner initialIsDarkTheme={initialIsDarkTheme} />;
};

export default Chatbot;