import React, { useState } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Placeholder authentication page with registration questionnaire
const AuthPage = () => {
  const [isLogin, setIsLogin] = useState(true);
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    softwareBackground: '',
    hardwareBackground: ''
  });

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!formData.softwareBackground || !formData.hardwareBackground) {
      if (!isLogin) {
        alert("Please complete the required background questionnaire");
        return;
      }
    }

    if (isLogin) {
      // Handle login
      try {
        const response = await fetch('http://localhost:8000/auth/login', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email: formData.email,
            password: formData.password,
          }),
        });

        const data = await response.json();

        if (response.ok) {
          alert('Login successful!');
          console.log('Login data:', data);
          // In a real app, you would store the token and redirect
        } else {
          alert(data.detail || 'Login failed');
        }
      } catch (error) {
        console.error('Login error:', error);
        alert('An error occurred during login');
      }
    } else {
      // Handle registration with required background information
      try {
        const response = await fetch('http://localhost:8000/auth/register', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({
            email: formData.email,
            password: formData.password,
            softwareBackground: formData.softwareBackground,
            hardwareBackground: formData.hardwareBackground,
          }),
        });

        const data = await response.json();

        if (response.ok) {
          alert('Registration successful!');
          console.log('Registration data:', data);
          // In a real app, you would store the token and redirect
        } else {
          alert(data.detail || 'Registration failed');
        }
      } catch (error) {
        console.error('Registration error:', error);
        alert('An error occurred during registration');
      }
    }
  };

  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Authentication - ${siteConfig.title}`}
      description="Login or register for the Physical AI & Humanoid Robotics textbook">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>{isLogin ? 'Login' : 'Create Account'}</h2>
                <div className="auth-toggle">
                  <button
                    className={isLogin ? 'active' : ''}
                    onClick={() => setIsLogin(true)}
                  >
                    Login
                  </button>
                  <button
                    className={!isLogin ? 'active' : ''}
                    onClick={() => setIsLogin(false)}
                  >
                    Register
                  </button>
                </div>
              </div>
              <div className="card__body">
                <form onSubmit={handleSubmit}>
                  <div className="form-group">
                    <label htmlFor="email">Email:</label>
                    <input
                      type="email"
                      id="email"
                      name="email"
                      value={formData.email}
                      onChange={handleInputChange}
                      required
                    />
                  </div>

                  <div className="form-group">
                    <label htmlFor="password">Password:</label>
                    <input
                      type="password"
                      id="password"
                      name="password"
                      value={formData.password}
                      onChange={handleInputChange}
                      required
                    />
                  </div>

                  {!isLogin && (
                    <>
                      <div className="form-group">
                        <label htmlFor="softwareBackground">Software Background:</label>
                        <select
                          id="softwareBackground"
                          name="softwareBackground"
                          value={formData.softwareBackground}
                          onChange={handleInputChange}
                          required
                        >
                          <option value="">Select your software background</option>
                          <option value="none">None/Beginner</option>
                          <option value="python">Python Programming</option>
                          <option value="cpp">C++ Programming</option>
                          <option value="robotics">Robotics Programming</option>
                          <option value="ai_ml">AI/ML Experience</option>
                          <option value="advanced">Advanced Software Development</option>
                        </select>
                        <small>This helps us personalize your learning experience</small>
                      </div>

                      <div className="form-group">
                        <label htmlFor="hardwareBackground">Hardware Background:</label>
                        <select
                          id="hardwareBackground"
                          name="hardwareBackground"
                          value={formData.hardwareBackground}
                          onChange={handleInputChange}
                          required
                        >
                          <option value="">Select your hardware background</option>
                          <option value="none">None/Beginner</option>
                          <option value="electronics">Electronics Fundamentals</option>
                          <option value="microcontrollers">Microcontrollers (Arduino, Raspberry Pi)</option>
                          <option value="robotics">Robotics Hardware</option>
                          <option value="sensors">Sensor Integration</option>
                          <option value="advanced">Advanced Hardware Engineering</option>
                        </select>
                        <small>This helps us personalize your learning experience</small>
                      </div>
                    </>
                  )}

                  <button type="submit" className="button button--primary button--block">
                    {isLogin ? 'Login' : 'Register'}
                  </button>
                </form>
              </div>
            </div>
          </div>
        </div>
      </div>

      <style jsx>{`
        .auth-toggle {
          display: flex;
          margin-top: 1rem;
        }

        .auth-toggle button {
          flex: 1;
          padding: 0.5rem;
          border: 1px solid #ccc;
          background: #f5f5f5;
          cursor: pointer;
        }

        .auth-toggle button.active {
          background: #007cba;
          color: white;
          border-color: #007cba;
        }

        .form-group {
          margin-bottom: 1.5rem;
        }

        .form-group label {
          display: block;
          margin-bottom: 0.5rem;
          font-weight: bold;
        }

        .form-group input,
        .form-group select {
          width: 100%;
          padding: 0.5rem;
          border: 1px solid #ccc;
          border-radius: 4px;
        }

        .form-group small {
          display: block;
          margin-top: 0.25rem;
          color: #666;
          font-size: 0.85rem;
        }
      `}</style>
    </Layout>
  );
};

export default AuthPage;