import React, { useState, useEffect } from 'react';
import styles from './ChapterQuiz.module.css';

interface Question {
  question: string;
  options: string[];
  correct_answer?: number;
  explanation?: string;
  difficulty: 'easy' | 'medium' | 'hard';
}

interface QuizData {
  id: number;
  chapter_id: number;
  questions: Question[];
  language: string;
  total_questions: number;
}

interface QuizResult {
  score: number;
  percentage: number;
  feedback: Array<{
    question_index: number;
    user_answer: number;
    correct_answer: number;
    is_correct: boolean;
    explanation: string;
  }>;
}

interface ChapterQuizProps {
  chapterId: number;
  language?: string;
}

const ChapterQuiz: React.FC<ChapterQuizProps> = ({ chapterId, language = 'en' }) => {
  const [quiz, setQuiz] = useState<QuizData | null>(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [userAnswers, setUserAnswers] = useState<number[]>([]);
  const [showResults, setShowResults] = useState(false);
  const [result, setResult] = useState<QuizResult | null>(null);
  const [startTime, setStartTime] = useState<number>(Date.now());

  useEffect(() => {
    fetchQuiz();
  }, [chapterId, language]);

  const fetchQuiz = async () => {
    setLoading(true);
    setError(null);

    // Simulate loading delay
    setTimeout(() => {
      // Static sample quiz data for demonstration
      const sampleQuiz: QuizData = {
        id: chapterId,
        chapter_id: chapterId,
        language: language,
        total_questions: 3,
        questions: [
          {
            question: language === 'ur'
              ? 'ہیومنائیڈ روبوٹکس میں ZMP کا کیا مطلب ہے؟'
              : 'What does ZMP stand for in humanoid robotics?',
            options: language === 'ur'
              ? ['زیرو موشن پوائنٹ', 'زیرو مومنٹ پوائنٹ', 'زون موشن پیرامیٹر', 'زونل میکانکل پوزیشن']
              : ['Zero Motion Point', 'Zero Moment Point', 'Zone Motion Parameter', 'Zonal Mechanical Position'],
            correct_answer: 1,
            explanation: language === 'ur'
              ? 'ZMP کا مطلب Zero Moment Point ہے، جو توازن اور چلنے کے کنٹرول میں اہم ہے۔'
              : 'ZMP stands for Zero Moment Point, which is crucial for balance and walking control.',
            difficulty: 'easy',
          },
          {
            question: language === 'ur'
              ? 'ہیومنائیڈ روبوٹس کون سے سینسرز استعمال کرتے ہیں؟'
              : 'Which sensors do humanoid robots commonly use?',
            options: language === 'ur'
              ? ['صرف کیمرہ', 'IMU اور فورس سینسرز', 'صرف لیزر', 'کوئی سینسرز نہیں']
              : ['Camera only', 'IMU and force sensors', 'Laser only', 'No sensors'],
            correct_answer: 1,
            explanation: language === 'ur'
              ? 'ہیومنائیڈ روبوٹس IMU (Inertial Measurement Units) اور فورس سینسرز استعمال کرتے ہیں۔'
              : 'Humanoid robots use IMU (Inertial Measurement Units) and force sensors for balance and movement.',
            difficulty: 'medium',
          },
          {
            question: language === 'ur'
              ? 'سِم ٹو ریئل ٹرانسفر کا کیا مقصد ہے؟'
              : 'What is the purpose of sim-to-real transfer?',
            options: language === 'ur'
              ? ['سمیولیشن میں تربیت یافتہ ماڈلز کو حقیقی روبوٹس میں منتقل کرنا', 'روبوٹس کو تیزی سے چلانا', 'سینسرز کو کیلیبریٹ کرنا', 'بجلی بچانا']
              : ['Transfer simulation-trained models to real robots', 'Make robots move faster', 'Calibrate sensors', 'Save power'],
            correct_answer: 0,
            explanation: language === 'ur'
              ? 'سِم ٹو ریئل ٹرانسفر سمیولیشن میں سیکھے گئے ماڈلز کو حقیقی دنیا کے روبوٹس میں منتقل کرتا ہے۔'
              : 'Sim-to-real transfer applies models learned in simulation to real-world robots.',
            difficulty: 'hard',
          },
        ],
      };

      setQuiz(sampleQuiz);
      setUserAnswers(new Array(sampleQuiz.total_questions).fill(-1));
      setStartTime(Date.now());
      setLoading(false);
    }, 500);
  };

  const handleAnswerSelect = (answerIndex: number) => {
    const newAnswers = [...userAnswers];
    newAnswers[currentQuestion] = answerIndex;
    setUserAnswers(newAnswers);
  };

  const handleNext = () => {
    if (currentQuestion < quiz!.total_questions - 1) {
      setCurrentQuestion(currentQuestion + 1);
    }
  };

  const handlePrevious = () => {
    if (currentQuestion > 0) {
      setCurrentQuestion(currentQuestion - 1);
    }
  };

  const handleSubmit = async () => {
    if (!quiz) return;

    // Check if all questions are answered
    if (userAnswers.some(a => a === -1)) {
      alert(language === 'ur'
        ? 'براہ کرم تمام سوالات کے جوابات دیں'
        : 'Please answer all questions before submitting');
      return;
    }

    // Calculate results locally (no backend required)
    const feedback = userAnswers.map((userAnswer, index) => {
      const question = quiz.questions[index];
      const correctAnswer = question.correct_answer || 0;
      const isCorrect = userAnswer === correctAnswer;

      return {
        question_index: index,
        user_answer: userAnswer,
        correct_answer: correctAnswer,
        is_correct: isCorrect,
        explanation: question.explanation || '',
      };
    });

    const score = feedback.filter(f => f.is_correct).length;
    const percentage = Math.round((score / quiz.total_questions) * 100);

    const resultData: QuizResult = {
      score,
      percentage,
      feedback,
    };

    setResult(resultData);
    setShowResults(true);
  };

  const handleRetry = () => {
    setUserAnswers(new Array(quiz!.total_questions).fill(-1));
    setCurrentQuestion(0);
    setShowResults(false);
    setResult(null);
    setStartTime(Date.now());
  };

  if (loading) {
    return (
      <div className={styles.quizContainer}>
        <div className={styles.loading}>
          {language === 'ur' ? 'کوئز لوڈ ہو رہا ہے...' : 'Loading quiz...'}
        </div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.quizContainer}>
        <div className={styles.error}>{error}</div>
      </div>
    );
  }

  if (!quiz) {
    return null;
  }

  // Show results
  if (showResults && result) {
    const passPercentage = 70;
    const passed = result.percentage >= passPercentage;

    return (
      <div className={styles.quizContainer}>
        <div className={styles.resultsContainer}>
          <h2 className={styles.resultsTitle}>
            {language === 'ur' ? '🎉 کوئز مکمل!' : '🎉 Quiz Complete!'}
          </h2>

          <div className={`${styles.scoreCard} ${passed ? styles.passed : styles.failed}`}>
            <div className={styles.scoreNumber}>{result.percentage}%</div>
            <div className={styles.scoreText}>
              {result.score} / {quiz.total_questions}{' '}
              {language === 'ur' ? 'درست' : 'Correct'}
            </div>
            <div className={styles.scoreStatus}>
              {passed
                ? (language === 'ur' ? '✓ کامیاب' : '✓ Passed')
                : (language === 'ur' ? '✗ ناکام' : '✗ Failed')}
            </div>
          </div>

          <div className={styles.feedback}>
            <h3 className={styles.feedbackTitle}>
              {language === 'ur' ? 'تفصیلی Feedback' : 'Detailed Feedback'}
            </h3>
            {result.feedback.map((item, index) => (
              <div
                key={index}
                className={`${styles.feedbackItem} ${
                  item.is_correct ? styles.correct : styles.incorrect
                }`}
              >
                <div className={styles.feedbackHeader}>
                  <span className={styles.questionNumber}>
                    {language === 'ur' ? `سوال ${index + 1}` : `Question ${index + 1}`}
                  </span>
                  <span className={styles.feedbackIcon}>
                    {item.is_correct ? '✓' : '✗'}
                  </span>
                </div>
                <p className={styles.feedbackQuestion}>
                  {quiz.questions[index].question}
                </p>
                <p className={styles.feedbackAnswer}>
                  <strong>
                    {language === 'ur' ? 'آپ کا جواب: ' : 'Your answer: '}
                  </strong>
                  {quiz.questions[index].options[item.user_answer]}
                </p>
                {!item.is_correct && (
                  <p className={styles.feedbackCorrect}>
                    <strong>
                      {language === 'ur' ? 'درست جواب: ' : 'Correct answer: '}
                    </strong>
                    {quiz.questions[index].options[item.correct_answer]}
                  </p>
                )}
                <p className={styles.feedbackExplanation}>{item.explanation}</p>
              </div>
            ))}
          </div>

          <button className={styles.retryButton} onClick={handleRetry}>
            {language === 'ur' ? '🔄 دوبارہ کوشش کریں' : '🔄 Try Again'}
          </button>
        </div>
      </div>
    );
  }

  // Show quiz questions
  const question = quiz.questions[currentQuestion];
  const progress = ((currentQuestion + 1) / quiz.total_questions) * 100;

  return (
    <div className={styles.quizContainer}>
      <div className={styles.header}>
        <h2 className={styles.title}>
          {language === 'ur' ? '📝 کوئز' : '📝 Chapter Quiz'}
        </h2>
        <div className={styles.progress}>
          <div className={styles.progressBar} style={{ width: `${progress}%` }} />
        </div>
        <div className={styles.progressText}>
          {language === 'ur'
            ? `سوال ${currentQuestion + 1} از ${quiz.total_questions}`
            : `Question ${currentQuestion + 1} of ${quiz.total_questions}`}
        </div>
      </div>

      <div className={styles.questionContainer}>
        <div className={styles.difficultyBadge}>
          {question.difficulty === 'easy' && (language === 'ur' ? '🟢 آسان' : '🟢 Easy')}
          {question.difficulty === 'medium' && (language === 'ur' ? '🟡 درمیانی' : '🟡 Medium')}
          {question.difficulty === 'hard' && (language === 'ur' ? '🔴 مشکل' : '🔴 Hard')}
        </div>

        <h3 className={styles.question}>{question.question}</h3>

        <div className={styles.options}>
          {question.options.map((option, index) => (
            <button
              key={index}
              className={`${styles.option} ${
                userAnswers[currentQuestion] === index ? styles.selected : ''
              }`}
              onClick={() => handleAnswerSelect(index)}
            >
              <span className={styles.optionLetter}>
                {String.fromCharCode(65 + index)}
              </span>
              <span className={styles.optionText}>{option}</span>
            </button>
          ))}
        </div>
      </div>

      <div className={styles.navigation}>
        <button
          className={styles.navButton}
          onClick={handlePrevious}
          disabled={currentQuestion === 0}
        >
          {language === 'ur' ? '← پچھلا' : '← Previous'}
        </button>

        {currentQuestion === quiz.total_questions - 1 ? (
          <button className={styles.submitButton} onClick={handleSubmit}>
            {language === 'ur' ? 'جمع کرائیں ✓' : 'Submit ✓'}
          </button>
        ) : (
          <button className={styles.navButton} onClick={handleNext}>
            {language === 'ur' ? 'اگلا →' : 'Next →'}
          </button>
        )}
      </div>
    </div>
  );
};

export default ChapterQuiz;
