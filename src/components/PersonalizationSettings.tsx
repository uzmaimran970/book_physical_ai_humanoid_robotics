/**
 * Personalization Settings Component
 * Allows users to configure their learning preferences
 */
import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './Personalization.module.css';

interface ProfileData {
  experience_level: string;
  focus_areas: string[];
  learning_goals: string;
}

const FOCUS_AREA_OPTIONS = [
  'Locomotion & Walking',
  'Sensors & Perception',
  'Control Systems',
  'Machine Learning',
  'Reinforcement Learning',
  'Sim-to-Real Transfer',
  'Hardware Design',
  'Ethics & Safety',
];

const PersonalizationSettings: React.FC = () => {
  const { user, isAuthenticated } = useAuth();
  const [profile, setProfile] = useState<ProfileData>({
    experience_level: 'beginner',
    focus_areas: [],
    learning_goals: '',
  });
  const [isLoading, setIsLoading] = useState(true);
  const [isSaving, setIsSaving] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  useEffect(() => {
    if (isAuthenticated) {
      fetchProfile();
    }
  }, [isAuthenticated]);

  const fetchProfile = async () => {
    // Mock implementation - load default static data
    setTimeout(() => {
      setProfile({
        experience_level: 'beginner',
        focus_areas: [],
        learning_goals: '',
      });
      setIsLoading(false);
    }, 500);
  };

  const handleSave = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsSaving(true);
    setMessage(null);

    // Mock implementation - simulate save with delay
    setTimeout(() => {
      setMessage({
        type: 'error',
        text: 'Backend not available. This is a UI demonstration only. Preferences are not saved.',
      });
      setIsSaving(false);
    }, 1000);
  };

  const handleFocusAreaToggle = (area: string) => {
    setProfile((prev) => ({
      ...prev,
      focus_areas: prev.focus_areas.includes(area)
        ? prev.focus_areas.filter((a) => a !== area)
        : [...prev.focus_areas, area],
    }));
  };

  if (!isAuthenticated) {
    return (
      <div className={styles.container}>
        <p>Please log in to access personalization settings.</p>
      </div>
    );
  }

  if (isLoading) {
    return (
      <div className={styles.container}>
        <p>Loading...</p>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <div className={styles.card}>
        <h2 className={styles.title}>Personalization Settings</h2>
        <p className={styles.subtitle}>
          Customize your learning experience to match your goals and interests
        </p>

        <form onSubmit={handleSave} className={styles.form}>
          {message && (
            <div className={`${styles.message} ${styles[message.type]}`}>
              {message.text}
            </div>
          )}

          <div className={styles.formGroup}>
            <label className={styles.label}>Experience Level</label>
            <div className={styles.radioGroup}>
              {['beginner', 'intermediate', 'advanced'].map((level) => (
                <label key={level} className={styles.radioLabel}>
                  <input
                    type="radio"
                    name="experience_level"
                    value={level}
                    checked={profile.experience_level === level}
                    onChange={(e) =>
                      setProfile({ ...profile, experience_level: e.target.value })
                    }
                    className={styles.radioInput}
                  />
                  <span className={styles.radioText}>
                    {level.charAt(0).toUpperCase() + level.slice(1)}
                  </span>
                </label>
              ))}
            </div>
            <p className={styles.helpText}>
              {profile.experience_level === 'beginner' &&
                'New to Physical AI and Robotics. Focus on fundamentals.'}
              {profile.experience_level === 'intermediate' &&
                'Some knowledge of robotics. Ready for deeper concepts.'}
              {profile.experience_level === 'advanced' &&
                'Experienced in robotics. Interested in cutting-edge techniques.'}
            </p>
          </div>

          <div className={styles.formGroup}>
            <label className={styles.label}>Focus Areas</label>
            <p className={styles.helpText}>Select topics you're most interested in</p>
            <div className={styles.checkboxGrid}>
              {FOCUS_AREA_OPTIONS.map((area) => (
                <label key={area} className={styles.checkboxLabel}>
                  <input
                    type="checkbox"
                    checked={profile.focus_areas.includes(area)}
                    onChange={() => handleFocusAreaToggle(area)}
                    className={styles.checkbox}
                  />
                  <span className={styles.checkboxText}>{area}</span>
                </label>
              ))}
            </div>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="learning_goals" className={styles.label}>
              Learning Goals
            </label>
            <textarea
              id="learning_goals"
              value={profile.learning_goals}
              onChange={(e) => setProfile({ ...profile, learning_goals: e.target.value })}
              placeholder="What do you want to achieve with this textbook? (e.g., 'I want to build my own humanoid robot' or 'I'm preparing for a career in robotics')"
              className={styles.textarea}
              rows={4}
              maxLength={500}
            />
            <p className={styles.charCount}>
              {profile.learning_goals.length}/500 characters
            </p>
          </div>

          <button type="submit" className={styles.saveButton} disabled={isSaving}>
            {isSaving ? 'Saving...' : 'Save Preferences'}
          </button>
        </form>
      </div>
    </div>
  );
};

export default PersonalizationSettings;
