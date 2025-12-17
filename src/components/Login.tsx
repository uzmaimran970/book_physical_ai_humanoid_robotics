/**
 * Login Component
 * User authentication form with session management
 */
import React, { useState } from 'react';
import styles from './Auth.module.css';

interface LoginFormData {
  email: string;
  password: string;
}

interface LoginProps {
  onSuccess?: () => void;
  onSwitchToSignUp?: () => void;
}

const Login: React.FC<LoginProps> = ({ onSuccess, onSwitchToSignUp }) => {
  const [formData, setFormData] = useState<LoginFormData>({
    email: '',
    password: '',
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [validationErrors, setValidationErrors] = useState<Partial<LoginFormData>>({});

  const validateForm = (): boolean => {
    const errors: Partial<LoginFormData> = {};

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!formData.email) {
      errors.email = 'Email is required';
    } else if (!emailRegex.test(formData.email)) {
      errors.email = 'Invalid email format';
    }

    // Password validation
    if (!formData.password) {
      errors.password = 'Password is required';
    }

    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  const handleChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    // Clear validation error for this field
    setValidationErrors((prev) => ({ ...prev, [name]: undefined }));
    setError(null);
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateForm()) {
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      // Mock implementation - no backend available
      throw new Error('Authentication backend is not available. This is a UI demonstration only.');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred during login');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.authTitle}>Welcome Back</h2>
        <p className={styles.authSubtitle}>
          Log in to continue your learning journey
        </p>

        <form onSubmit={handleSubmit} className={styles.authForm}>
          {error && (
            <div className={styles.errorAlert} role="alert">
              {error}
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="email" className={styles.formLabel}>
              Email
            </label>
            <input
              type="email"
              id="email"
              name="email"
              value={formData.email}
              onChange={handleChange}
              className={`${styles.formInput} ${validationErrors.email ? styles.inputError : ''}`}
              placeholder="Enter your email"
              disabled={isLoading}
              autoComplete="email"
            />
            {validationErrors.email && (
              <span className={styles.errorText}>{validationErrors.email}</span>
            )}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="password" className={styles.formLabel}>
              Password
            </label>
            <input
              type="password"
              id="password"
              name="password"
              value={formData.password}
              onChange={handleChange}
              className={`${styles.formInput} ${validationErrors.password ? styles.inputError : ''}`}
              placeholder="Enter your password"
              disabled={isLoading}
              autoComplete="current-password"
            />
            {validationErrors.password && (
              <span className={styles.errorText}>{validationErrors.password}</span>
            )}
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Logging In...' : 'Log In'}
          </button>
        </form>

        {onSwitchToSignUp && (
          <div className={styles.authFooter}>
            <p>
              Don't have an account?{' '}
              <button
                type="button"
                onClick={onSwitchToSignUp}
                className={styles.linkButton}
                disabled={isLoading}
              >
                Sign Up
              </button>
            </p>
          </div>
        )}
      </div>
    </div>
  );
};

export default Login;
