import React, { useState, useEffect } from 'react';

// Urdu translation component with caching
const UrduTranslation = ({ content, contentId }) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  // Check if translation is already cached
  useEffect(() => {
    if (contentId) {
      const cachedTranslation = localStorage.getItem(`urdu-translation-${contentId}`);
      if (cachedTranslation) {
        setTranslatedContent(cachedTranslation);
        setIsTranslated(true);
      }
    }
  }, [contentId]);

  const translateToUrdu = async () => {
    if (isTranslated) {
      // Toggle back to original content
      setIsTranslated(false);
      return;
    }

    setIsLoading(true);

    try {
      // Check cache first
      if (contentId) {
        const cached = localStorage.getItem(`urdu-translation-${contentId}`);
        if (cached) {
          setTranslatedContent(cached);
          setIsTranslated(true);
          setIsLoading(false);
          return;
        }
      }

      // In a real implementation, this would call an AI API to translate the content
      // For this example, I'll simulate a translation API call
      // In production, you would use something like OpenAI API, Google Translate API, etc.

      // Simulate API call delay
      await new Promise(resolve => setTimeout(resolve, 1000));

      // This is a placeholder - in real implementation, we would call an actual translation API
      // For now, I'll return a message indicating it would be translated
      const mockTranslation = `یہ مشینی ترجمہ ہے: ${content.substring(0, 100)}... [مکمل مواد کے لیے اصل مواد دیکھیں]`;

      // Cache the translation if contentId is provided
      if (contentId) {
        localStorage.setItem(`urdu-translation-${contentId}`, mockTranslation);
      }

      setTranslatedContent(mockTranslation);
      setIsTranslated(true);
    } catch (error) {
      console.error('Translation error:', error);
      alert('Translation failed. Please try again later.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="urdu-translation">
      <button
        className="translate-btn"
        onClick={translateToUrdu}
        disabled={isLoading}
      >
        {isLoading ? 'Translating...' :
         isTranslated ? 'Show Original' : '.Translate to Urdu'}
      </button>

      {isTranslated && (
        <div className="translated-content" dir="rtl">
          {translatedContent}
        </div>
      )}

      {!isTranslated && (
        <div className="original-content">
          {content}
        </div>
      )}

      <style jsx>{`
        .urdu-translation {
          margin: 1rem 0;
        }

        .translate-btn {
          background-color: #007cba;
          color: white;
          border: none;
          padding: 0.5rem 1rem;
          border-radius: 4px;
          cursor: pointer;
          margin-bottom: 1rem;
        }

        .translate-btn:hover:not(:disabled) {
          background-color: #005a87;
        }

        .translate-btn:disabled {
          background-color: #cccccc;
          cursor: not-allowed;
        }

        .translated-content {
          padding: 1rem;
          border: 1px solid #ddd;
          border-radius: 4px;
          background-color: #f0f8ff;
          font-size: 1.1em;
        }

        .original-content {
          padding: 1rem;
          border: 1px solid #ddd;
          border-radius: 4px;
        }
      `}</style>
    </div>
  );
};

export default UrduTranslation;