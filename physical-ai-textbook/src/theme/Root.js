import React from 'react';
import Chatbot from './Chatbot';

// Default implementation, that you can customize
function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}

export default Root;