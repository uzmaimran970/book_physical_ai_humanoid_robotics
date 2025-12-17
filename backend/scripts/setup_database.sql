-- Database schema for RAG Chatbot query logging
-- Execute this script in your Neon database

-- Create extension for UUID generation
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Queries table
CREATE TABLE IF NOT EXISTS queries (
    query_id UUID PRIMARY KEY,
    query_text TEXT NOT NULL,
    top_k INTEGER NOT NULL DEFAULT 3,
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),

    CONSTRAINT query_text_length CHECK (char_length(query_text) BETWEEN 1 AND 2000)
);

-- Responses table
CREATE TABLE IF NOT EXISTS responses (
    response_id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    query_id UUID NOT NULL REFERENCES queries(query_id) ON DELETE CASCADE,
    response_text TEXT NOT NULL,
    citations JSONB DEFAULT '[]'::jsonb,
    latency_ms INTEGER NOT NULL,
    status VARCHAR(20) NOT NULL CHECK (status IN ('success', 'partial', 'cannot_answer')),

    CONSTRAINT latency_positive CHECK (latency_ms > 0)
);

-- Retrieved chunks junction table
CREATE TABLE IF NOT EXISTS retrieved_chunks (
    id SERIAL PRIMARY KEY,
    query_id UUID NOT NULL REFERENCES queries(query_id) ON DELETE CASCADE,
    chunk_id UUID NOT NULL,
    relevance_score FLOAT NOT NULL,
    rank INTEGER NOT NULL,

    CONSTRAINT relevance_score_range CHECK (relevance_score BETWEEN 0.0 AND 1.0),
    CONSTRAINT rank_positive CHECK (rank > 0),
    UNIQUE (query_id, chunk_id)
);

-- Error logs table
CREATE TABLE IF NOT EXISTS error_logs (
    error_id UUID PRIMARY KEY,
    query_id UUID REFERENCES queries(query_id) ON DELETE SET NULL,
    error_code VARCHAR(50) NOT NULL,
    error_message TEXT NOT NULL,
    timestamp TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Books table (for metadata)
CREATE TABLE IF NOT EXISTS books (
    book_id UUID PRIMARY KEY,
    title VARCHAR(255) NOT NULL,
    author VARCHAR(255) NOT NULL,
    total_chapters INTEGER,
    ingestion_date TIMESTAMP DEFAULT NOW()
);

-- Indexes for performance
CREATE INDEX IF NOT EXISTS idx_queries_timestamp ON queries(timestamp);
CREATE INDEX IF NOT EXISTS idx_responses_query_id ON responses(query_id);
CREATE INDEX IF NOT EXISTS idx_responses_status ON responses(status);
CREATE INDEX IF NOT EXISTS idx_retrieved_chunks_query_id ON retrieved_chunks(query_id);
CREATE INDEX IF NOT EXISTS idx_retrieved_chunks_chunk_id ON retrieved_chunks(chunk_id);
CREATE INDEX IF NOT EXISTS idx_error_logs_timestamp ON error_logs(timestamp);
CREATE INDEX IF NOT EXISTS idx_error_logs_error_code ON error_logs(error_code);
CREATE INDEX IF NOT EXISTS idx_responses_latency ON responses(latency_ms);

-- Views for analytics

-- Query success rate view
CREATE OR REPLACE VIEW query_success_rate AS
SELECT
    DATE(q.timestamp) as date,
    COUNT(*) as total_queries,
    SUM(CASE WHEN r.status = 'success' THEN 1 ELSE 0 END) as successful_queries,
    ROUND(
        100.0 * SUM(CASE WHEN r.status = 'success' THEN 1 ELSE 0 END) / COUNT(*),
        2
    ) as success_rate_pct
FROM queries q
LEFT JOIN responses r ON q.query_id = r.query_id
GROUP BY DATE(q.timestamp)
ORDER BY date DESC;

-- Average latency by status view
CREATE OR REPLACE VIEW avg_latency_by_status AS
SELECT
    status,
    COUNT(*) as count,
    ROUND(AVG(latency_ms), 2) as avg_latency_ms,
    ROUND(MIN(latency_ms), 2) as min_latency_ms,
    ROUND(MAX(latency_ms), 2) as max_latency_ms,
    ROUND(PERCENTILE_CONT(0.5) WITHIN GROUP (ORDER BY latency_ms), 2) as p50_latency_ms,
    ROUND(PERCENTILE_CONT(0.90) WITHIN GROUP (ORDER BY latency_ms), 2) as p90_latency_ms,
    ROUND(PERCENTILE_CONT(0.95) WITHIN GROUP (ORDER BY latency_ms), 2) as p95_latency_ms
FROM responses
GROUP BY status
ORDER BY status;

-- Top errors view
CREATE OR REPLACE VIEW top_errors AS
SELECT
    error_code,
    COUNT(*) as occurrence_count,
    MAX(timestamp) as last_occurrence
FROM error_logs
GROUP BY error_code
ORDER BY occurrence_count DESC;

-- Stored function for complete query logging
CREATE OR REPLACE FUNCTION log_query_complete(
    p_query_id UUID,
    p_query_text TEXT,
    p_top_k INTEGER,
    p_response_text TEXT,
    p_citations JSONB,
    p_latency_ms INTEGER,
    p_status VARCHAR(20)
) RETURNS UUID AS $$
DECLARE
    v_response_id UUID;
BEGIN
    -- Insert query
    INSERT INTO queries (query_id, query_text, top_k, timestamp)
    VALUES (p_query_id, p_query_text, p_top_k, NOW());

    -- Insert response
    v_response_id := uuid_generate_v4();
    INSERT INTO responses (response_id, query_id, response_text, citations, latency_ms, status)
    VALUES (v_response_id, p_query_id, p_response_text, p_citations, p_latency_ms, p_status);

    RETURN v_response_id;
END;
$$ LANGUAGE plpgsql;

-- Sample data for testing (optional)
-- Uncomment to insert test data

/*
-- Sample book
INSERT INTO books (book_id, title, author, total_chapters)
VALUES (
    uuid_generate_v4(),
    'Physical AI & Humanoid Robotics',
    'Test Author',
    10
) ON CONFLICT DO NOTHING;

-- Sample query
DO $$
DECLARE
    test_query_id UUID := uuid_generate_v4();
    test_response_id UUID := uuid_generate_v4();
BEGIN
    INSERT INTO queries (query_id, query_text, top_k)
    VALUES (test_query_id, 'What are IMU sensors?', 3);

    INSERT INTO responses (response_id, query_id, response_text, citations, latency_ms, status)
    VALUES (
        test_response_id,
        test_query_id,
        'IMU sensors are inertial measurement units...',
        '[{"chapter": 3, "section": "Sensors", "page": 45}]'::jsonb,
        1250,
        'success'
    );

    INSERT INTO retrieved_chunks (query_id, chunk_id, relevance_score, rank)
    VALUES
        (test_query_id, uuid_generate_v4(), 0.89, 1),
        (test_query_id, uuid_generate_v4(), 0.76, 2),
        (test_query_id, uuid_generate_v4(), 0.65, 3);
END $$;
*/

-- Grant permissions (adjust user as needed)
-- GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO your_user;
-- GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO your_user;
