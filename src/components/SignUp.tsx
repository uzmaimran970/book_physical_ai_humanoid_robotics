/**
 * SignUp Component
 * User registration form with validation and API integration
 */
import React, { useState } from 'react';
import styles from './Auth.module.css';

interface SignUpFormData {
  email: string;
  password: string;
  confirmPassword: string;
  fullName: string;
}

interface SignUpProps {
  onSuccess?: () => void;
  onSwitchToLogin?: () => void;
}

const SignUp: React.FC<SignUpProps> = ({ onSuccess, onSwitchToLogin }) => {
  const [formData, setFormData] = useState<SignUpFormData>({
    email: '',
    password: '',
    confirmPassword: '',
    fullName: '',
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [validationErrors, setValidationErrors] = useState<Partial<SignUpFormData>>({});

  const validateForm = (): boolean => {
    const errors: Partial<SignUpFormData> = {};

    // Email validation
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!formData.email) {
      errors.email = 'Email is required';
    } else if (!emailRegex.test(formData.email)) {
      errors.email = 'Invalid email format';
    }

    // Full name validation
    if (!formData.fullName || formData.fullName.trim().length === 0) {
      errors.fullName = 'Full name is required';
    }

    // Password validation
    if (!formData.password) {
      errors.password = 'Password is required';
    } else if (formData.password.length < 8) {
      errors.password = 'Password must be at least 8 characters';
    } else if (formData.password.length > 100) {
      errors.password = 'Password must be less than 100 characters';
    }

    // Confirm password validation
    if (formData.password !== formData.confirmPassword) {
      errors.confirmPassword = 'Passwords do not match';
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
      setError(err instanceof Error ? err.message : 'An error occurred during signup');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2 className={styles.authTitle}>Create Account</h2>
        <p className={styles.authSubtitle}>
          Join to access personalized learning features
        </p>

        <form onSubmit={handleSubmit} className={styles.authForm}>
          {error && (
            <div className={styles.errorAlert} role="alert">
              {error}
            </div>
          )}

          <div className={styles.formGroup}>
            <label htmlFor="fullName" className={styles.formLabel}>
              Full Name
            </label>
            <input
              type="text"
              id="fullName"
              name="fullName"
              value={formData.fullName}
              onChange={handleChange}
              className={`${styles.formInput} ${validationErrors.fullName ? styles.inputError : ''}`}
              placeholder="Enter your full name"
              disabled={isLoading}
            />
            {validationErrors.fullName && (
              <span className={styles.errorText}>{validationErrors.fullName}</span>
            )}
          </div>

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
              placeholder="At least 8 characters"
              disabled={isLoading}
            />
            {validationErrors.password && (
              <span className={styles.errorText}>{validationErrors.password}</span>
            )}
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="confirmPassword" className={styles.formLabel}>
              Confirm Password
            </label>
            <input
              type="password"
              id="confirmPassword"
              name="confirmPassword"
              value={formData.confirmPassword}
              onChange={handleChange}
              className={`${styles.formInput} ${validationErrors.confirmPassword ? styles.inputError : ''}`}
              placeholder="Re-enter your password"
              disabled={isLoading}
            />
            {validationErrors.confirmPassword && (
              <span className={styles.errorText}>{validationErrors.confirmPassword}</span>
            )}
          </div>

          <button
            type="submit"
            className={styles.submitButton}
            disabled={isLoading}
          >
            {isLoading ? 'Creating Account...' : 'Sign Up'}
          </button>
        </form>

        {onSwitchToLogin && (
          <div className={styles.authFooter}>
            <p>
              Already have an account?{' '}
              <button
                type="button"
                onClick={onSwitchToLogin}
                className={styles.linkButton}
                disabled={isLoading}
              >
                Log In
              </button>
            </p>
          </div>
        )}
      </div>
    </div>
  );
};

export default SignUp;
