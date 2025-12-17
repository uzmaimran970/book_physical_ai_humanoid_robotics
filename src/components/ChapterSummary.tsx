import React, { useState, useEffect } from 'react';
import styles from './ChapterSummary.module.css';

interface KeyTerm {
  term: string;
  definition: string;
}

interface SummaryData {
  id: number;
  chapter_id: number;
  overview: string;
  key_points: string[];
  learning_objectives: string[];
  key_terms: KeyTerm[];
  language: string;
}

interface ChapterSummaryProps {
  chapterId: number;
  language?: string;
}

const ChapterSummary: React.FC<ChapterSummaryProps> = ({ chapterId, language = 'en' }) => {
  const [summary, setSummary] = useState<SummaryData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [expanded, setExpanded] = useState(true);

  useEffect(() => {
    fetchSummary();
  }, [chapterId, language]);

  const fetchSummary = async () => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch(
        `http://localhost:8000/api/summaries/${chapterId}?language=${language}`
      );

      if (!response.ok) {
        if (response.status === 404) {
          setError('Summary not available for this chapter yet.');
        } else {
          throw new Error('Failed to load summary');
        }
        return;
      }

      const data = await response.json();
      setSummary(data);
    } catch (err) {
      setError('Error loading summary. Please try again later.');
      console.error('Error fetching summary:', err);
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return (
      <div className={styles.summaryContainer}>
        <div className={styles.loading}>Loading summary...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.summaryContainer}>
        <div className={styles.error}>{error}</div>
      </div>
    );
  }

  if (!summary) {
    return null;
  }

  return (
    <div className={styles.summaryContainer}>
      <div className={styles.header}>
        <h2 className={styles.title}>
          {language === 'ur' ? '📝 باب کا خلاصہ' : '📝 Chapter Summary'}
        </h2>
        <button
          className={styles.toggleButton}
          onClick={() => setExpanded(!expanded)}
          aria-label={expanded ? 'Collapse' : 'Expand'}
        >
          {expanded ? '−' : '+'}
        </button>
      </div>

      {expanded && (
        <div className={styles.content}>
          {/* Overview */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>
              {language === 'ur' ? 'جائزہ' : 'Overview'}
            </h3>
            <p className={styles.overview}>{summary.overview}</p>
          </div>

          {/* Key Points */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>
              {language === 'ur' ? 'کلیدی نکات' : 'Key Points'}
            </h3>
            <ul className={styles.keyPoints}>
              {summary.key_points.map((point, index) => (
                <li key={index} className={styles.keyPoint}>
                  {point}
                </li>
              ))}
            </ul>
          </div>

          {/* Learning Objectives */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>
              {language === 'ur' ? 'سیکھنے کے مقاصد' : 'Learning Objectives'}
            </h3>
            <ul className={styles.objectives}>
              {summary.learning_objectives.map((objective, index) => (
                <li key={index} className={styles.objective}>
                  {objective}
                </li>
              ))}
            </ul>
          </div>

          {/* Key Terms */}
          <div className={styles.section}>
            <h3 className={styles.sectionTitle}>
              {language === 'ur' ? 'کلیدی اصطلاحات' : 'Key Terms'}
            </h3>
            <div className={styles.keyTerms}>
              {summary.key_terms.map((term, index) => (
                <div key={index} className={styles.keyTerm}>
                  <strong className={styles.termName}>{term.term}:</strong>{' '}
                  <span className={styles.termDefinition}>{term.definition}</span>
                </div>
              ))}
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChapterSummary;
