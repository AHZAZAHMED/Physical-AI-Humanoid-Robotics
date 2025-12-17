import React, { useState, useEffect } from 'react';

// Difficulty toggle component for content personalization
const DifficultyToggle = ({ onDifficultyChange }) => {
  const [difficulty, setDifficulty] = useState(() => {
    // Get saved difficulty from localStorage, default to 'intermediate'
    return typeof window !== 'undefined'
      ? localStorage.getItem('userDifficulty') || 'intermediate'
      : 'intermediate';
  });

  useEffect(() => {
    // Save difficulty to localStorage whenever it changes
    if (typeof window !== 'undefined') {
      localStorage.setItem('userDifficulty', difficulty);
    }

    // Notify parent component of difficulty change
    if (onDifficultyChange) {
      onDifficultyChange(difficulty);
    }
  }, [difficulty, onDifficultyChange]);

  const handleDifficultyChange = (newDifficulty) => {
    setDifficulty(newDifficulty);
  };

  return (
    <div className="difficulty-toggle">
      <label>Select your expertise level:</label>
      <div className="difficulty-buttons">
        <button
          className={`difficulty-btn ${difficulty === 'beginner' ? 'active' : ''}`}
          onClick={() => handleDifficultyChange('beginner')}
        >
          Beginner
        </button>
        <button
          className={`difficulty-btn ${difficulty === 'intermediate' ? 'active' : ''}`}
          onClick={() => handleDifficultyChange('intermediate')}
        >
          Intermediate
        </button>
        <button
          className={`difficulty-btn ${difficulty === 'advanced' ? 'active' : ''}`}
          onClick={() => handleDifficultyChange('advanced')}
        >
          Advanced
        </button>
      </div>
      <style jsx>{`
        .difficulty-toggle {
          display: flex;
          flex-direction: column;
          align-items: flex-start;
          margin: 1rem 0;
          padding: 1rem;
          border: 1px solid #ddd;
          border-radius: 4px;
          background-color: #f9f9f9;
        }

        .difficulty-buttons {
          display: flex;
          gap: 0.5rem;
          margin-top: 0.5rem;
        }

        .difficulty-btn {
          padding: 0.5rem 1rem;
          border: 1px solid #ccc;
          background-color: #fff;
          cursor: pointer;
          border-radius: 4px;
        }

        .difficulty-btn:hover {
          background-color: #e9e9e9;
        }

        .difficulty-btn.active {
          background-color: #007cba;
          color: white;
          border-color: #007cba;
        }
      `}</style>
    </div>
  );
};

export default DifficultyToggle;