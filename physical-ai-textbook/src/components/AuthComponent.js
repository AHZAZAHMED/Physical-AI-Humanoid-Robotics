import React, { useState } from 'react';
import { useAuth } from 'better-auth/react';

// Simple placeholder component for authentication
// This will be replaced with actual Better-Auth components when installed
const AuthComponent = () => {
  const { signIn, signOut, user } = useAuth();
  const [showRegistrationForm, setShowRegistrationForm] = useState(false);
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

  const handleRegister = async (e) => {
    e.preventDefault();

    // This would be the actual registration with Better-Auth
    // For now, this is a placeholder
    try {
      // In a real implementation, we would call the Better-Auth registration
      // with the additional fields for software and hardware background
      console.log("Registration data:", formData);
      // signIn('credentials', { ...formData });
    } catch (error) {
      console.error("Registration error:", error);
    }
  };

  if (showRegistrationForm) {
    return (
      <div className="auth-modal">
        <h3>Register</h3>
        <form onSubmit={handleRegister}>
          <input
            type="email"
            name="email"
            placeholder="Email"
            value={formData.email}
            onChange={handleInputChange}
            required
          />
          <input
            type="password"
            name="password"
            placeholder="Password"
            value={formData.password}
            onChange={handleInputChange}
            required
          />
          <select
            name="softwareBackground"
            value={formData.softwareBackground}
            onChange={handleInputChange}
            required
          >
            <option value="">Select Software Background</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
            <option value="expert">Expert</option>
          </select>
          <select
            name="hardwareBackground"
            value={formData.hardwareBackground}
            onChange={handleInputChange}
            required
          >
            <option value="">Select Hardware Background</option>
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
            <option value="expert">Expert</option>
          </select>
          <button type="submit">Register</button>
          <button type="button" onClick={() => setShowRegistrationForm(false)}>Cancel</button>
        </form>
      </div>
    );
  }

  if (user) {
    return (
      <div className="auth-component">
        <span>Welcome, {user.email}</span>
        <button onClick={() => signOut()}>Logout</button>
      </div>
    );
  }

  return (
    <div className="auth-component">
      <button onClick={() => setShowRegistrationForm(true)}>Register</button>
      <button onClick={() => signIn()}>Login</button>
    </div>
  );
};

export default AuthComponent;