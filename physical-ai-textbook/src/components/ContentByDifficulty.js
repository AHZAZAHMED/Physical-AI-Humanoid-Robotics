import React, { useState, useEffect } from 'react';

// Component that shows content based on user's selected difficulty level
const ContentByDifficulty = ({ children }) => {
  const [userDifficulty, setUserDifficulty] = useState('intermediate');

  useEffect(() => {
    // Get user's selected difficulty from localStorage
    const savedDifficulty = typeof window !== 'undefined'
      ? localStorage.getItem('userDifficulty') || 'intermediate'
      : 'intermediate';

    setUserDifficulty(savedDifficulty);

    // Set up a listener for changes to the difficulty in localStorage
    const handleStorageChange = (e) => {
      if (e.key === 'userDifficulty') {
        setUserDifficulty(e.newValue || 'intermediate');
      }
    };

    window.addEventListener('storage', handleStorageChange);
    return () => window.removeEventListener('storage', handleStorageChange);
  }, []);

  // Function to determine if content should be shown based on difficulty
  const shouldShowContent = (contentDifficulty) => {
    if (!contentDifficulty) return true; // If no difficulty specified, always show

    const difficultyLevels = {
      'beginner': 1,
      'intermediate': 2,
      'advanced': 3
    };

    const userLevel = difficultyLevels[userDifficulty] || 2; // Default to intermediate
    const contentLevel = difficultyLevels[contentDifficulty] || 2; // Default to intermediate

    // Users can see content at their level or below
    return contentLevel <= userLevel;
  };

  // Filter children based on their difficulty attribute
  const filteredChildren = React.Children.map(children, (child) => {
    if (React.isValidElement(child)) {
      const contentDifficulty = child.props.difficulty;
      if (shouldShowContent(contentDifficulty)) {
        return child;
      }
      return null;
    }
    return child;
  });

  return <div className="content-by-difficulty">{filteredChildren}</div>;
};

// Sub-component for content that has a specific difficulty level
export const DifficultyContent = ({ difficulty, children }) => {
  return <div data-difficulty={difficulty}>{children}</div>;
};

export default ContentByDifficulty;