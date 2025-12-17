# Local Testing Guide - Frontend + Backend

## Issue: Vercel frontend can't access localhost backend

Vercel (deployed frontend) cannot connect to your localhost backend. You have 3 options:

## Option 1: Test Locally (Recommended First)

### Step 1: Keep Backend Running
Your backend is already running at `http://localhost:8000`

### Step 2: Run Frontend Locally
```bash
# Go to your frontend directory
cd /mnt/c/Users/pc/Desktop/hackathon_2025/frontend

# Install dependencies (if not done)
npm install

# Run locally
npm run dev
```

Frontend will run at `http://localhost:3000` and can now connect to backend at `http://localhost:8000`

### Step 3: Update Frontend API URL
In your frontend code, set:
```javascript
const API_URL = 'http://localhost:8000';
```

### Step 4: Test Query
Open `http://localhost:3000` and try asking a question!

---

## Option 2: Use ngrok (Expose localhost publicly)

### Step 1: Install ngrok
```bash
# For Linux/WSL
wget https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-amd64.tgz
tar -xzf ngrok-v3-stable-linux-amd64.tgz
sudo mv ngrok /usr/local/bin/

# For Windows
# Download from: https://ngrok.com/download
```

### Step 2: Setup ngrok
```bash
# Sign up at https://dashboard.ngrok.com/get-started/setup
# Get your authtoken

ngrok config add-authtoken YOUR_AUTH_TOKEN
```

### Step 3: Expose Backend
```bash
ngrok http 8000
```

You'll get a public URL like:
```
https://abc123.ngrok-free.app
```

### Step 4: Update Frontend
Use this ngrok URL in your Vercel frontend:
```javascript
const API_URL = 'https://abc123.ngrok-free.app';
```

**Note**: Free ngrok URLs change every time you restart. For permanent URL, use ngrok paid plan or deploy backend.

---

## Option 3: Deploy Backend (Production Solution)

### Deploy to Render (Free Tier)

#### Step 1: Create Render Account
Go to https://render.com and sign up

#### Step 2: Create `render.yaml`
```yaml
services:
  - type: web
    name: rag-chatbot-backend
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn src.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: QDRANT_URL
        value: https://d8265a1b-bab5-4cfb-9435-cfb8385b9cb9.us-east4-0.gcp.cloud.qdrant.io
      - key: QDRANT_API_KEY
        sync: false  # Add in Render dashboard
      - key: COHERE_API_KEY
        sync: false
      - key: ANTHROPIC_API_KEY
        sync: false
      - key: NEON_DB_URL
        sync: false
```

#### Step 3: Deploy
```bash
# Push to GitHub
git add .
git commit -m "Add Render config"
git push

# Connect GitHub repo to Render
# Render will auto-deploy and give you a URL like:
# https://rag-chatbot-backend.onrender.com
```

#### Step 4: Update Frontend
```javascript
const API_URL = 'https://rag-chatbot-backend.onrender.com';
```

---

## Quick Test Without Frontend

Test backend directly to make sure it works:

```bash
# Check health
curl http://localhost:8000/api/v1/health

# Test ask endpoint (after ingesting book)
curl -X POST http://localhost:8000/api/v1/ask \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are IMU sensors?",
    "top_k": 3
  }'
```

---

## Recommended Flow for Hackathon Demo

### For Quick Demo (5 minutes):
1. Use **ngrok** to expose localhost
2. Update frontend API URL to ngrok URL
3. Demo ready!

### For Production (30 minutes):
1. Deploy backend to **Render** (free)
2. Update frontend to use Render URL
3. Redeploy frontend to Vercel
4. Production ready!

### For Local Testing (1 minute):
1. Run frontend locally: `npm run dev`
2. Backend already running on localhost:8000
3. Test locally on http://localhost:3000

---

## Current Setup Status

✅ Backend running: `http://localhost:8000`
✅ All credentials configured
✅ Qdrant collection created
✅ CORS enabled
✅ `/ask` endpoint ready

❌ Backend not publicly accessible (localhost only)
❌ Vercel frontend can't reach localhost

**Choose one option above to fix! 👆**

---

## Troubleshooting

### "Connection refused" error
- Backend not running → Start with `python -m uvicorn src.main:app --reload --host 0.0.0.0 --port 8000`

### CORS error
- Update `allowed_origins` in `src/main.py`
- Add your ngrok/Render URL

### "Collection doesn't exist" error
- Run `python scripts/setup_qdrant.py`
- Ingest book data with `python scripts/ingest_book.py`

---

**Need help? Let me know which option you want to use!**
