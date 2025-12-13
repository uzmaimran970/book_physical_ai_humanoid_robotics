/**
 * Recommendations Widget Component
 * Displays personalized chapter recommendations
 */
import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './Personalization.module.css';

interface ChapterRecommendation {
  chapter: {
    id: number;
    title: string;
    slug: string;
    description: string;
    difficulty: string;
    reading_time_minutes: number;
  };
  relevance_score: number;
  in_progress: boolean;
}

interface RecommendationsWidgetProps {
  limit?: number;
}

const RecommendationsWidget: React.FC<RecommendationsWidgetProps> = ({ limit = 3 }) => {
  const { isAuthenticated } = useAuth();
  const [recommendations, setRecommendations] = useState<ChapterRecommendation[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (isAuthenticated) {
      fetchRecommendations();
    } else {
      setIsLoading(false);
    }
  }, [isAuthenticated]);

  const fetchRecommendations = async () => {
    // Mock implementation - show message that backend is not available
    setTimeout(() => {
      setError('Backend not available. This is a UI demonstration only.');
      setIsLoading(false);
    }, 500);
  };

  if (!isAuthenticated) {
    return (
      <div className={styles.widgetCard}>
        <h3 className={styles.widgetTitle}>Recommended for You</h3>
        <p className={styles.widgetText}>
          Log in to get personalized chapter recommendations based on your learning goals.
        </p>
        <a href="/login" className={styles.widgetLink}>
          Log In
        </a>
      </div>
    );
  }

  if (isLoading) {
    return (
      <div className={styles.widgetCard}>
        <h3 className={styles.widgetTitle}>Recommended for You</h3>
        <p className={styles.widgetText}>Loading recommendations...</p>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.widgetCard}>
        <h3 className={styles.widgetTitle}>Recommended for You</h3>
        <p className={styles.widgetError}>{error}</p>
      </div>
    );
  }

  if (recommendations.length === 0) {
    return (
      <div className={styles.widgetCard}>
        <h3 className={styles.widgetTitle}>Recommended for You</h3>
        <p className={styles.widgetText}>
          Complete your profile settings to get personalized recommendations.
        </p>
        <a href="/personalization" className={styles.widgetLink}>
          Update Preferences
        </a>
      </div>
    );
  }

  return (
    <div className={styles.widgetCard}>
      <h3 className={styles.widgetTitle}>Recommended for You</h3>
      <div className={styles.recommendationsList}>
        {recommendations.map((rec, index) => (
          <a
            key={rec.chapter.id}
            href={`/docs/${rec.chapter.slug}`}
            className={styles.recommendationItem}
          >
            <div className={styles.recommendationHeader}>
              <span className={styles.recommendationRank}>#{index + 1}</span>
              <span className={styles.recommendationBadge}>
                {Math.round(rec.relevance_score * 100)}% match
              </span>
            </div>
            <h4 className={styles.recommendationTitle}>{rec.chapter.title}</h4>
            <p className={styles.recommendationDescription}>
              {rec.chapter.description}
            </p>
            <div className={styles.recommendationMeta}>
              <span className={styles.difficultyBadge}>
                {rec.chapter.difficulty}
              </span>
              <span className={styles.readingTime}>
                {rec.chapter.reading_time_minutes} min read
              </span>
              {rec.in_progress && (
                <span className={styles.inProgressBadge}>In Progress</span>
              )}
            </div>
          </a>
        ))}
      </div>
      <a href="/learning-path" className={styles.widgetLink}>
        View Full Learning Path →
      </a>
    </div>
  );
};

export default RecommendationsWidget;
