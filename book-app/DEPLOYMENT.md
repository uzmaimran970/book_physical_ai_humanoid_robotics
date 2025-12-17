# Deployment Guide

Deploy your educational book to various platforms.

## 📦 Build for Production

First, create an optimized production build:

```bash
npm run build
```

This creates a `dist/` folder with optimized static files.

## 🚀 Deployment Options

### 1. Vercel (Recommended)

**Easiest deployment - Zero configuration needed!**

#### Via Vercel CLI:
```bash
# Install Vercel CLI
npm install -g vercel

# Deploy
cd book-app
vercel
```

#### Via GitHub:
1. Push your code to GitHub
2. Go to [vercel.com](https://vercel.com)
3. Import your repository
4. Vercel auto-detects Vite and deploys!

### 2. Netlify

#### Via Netlify CLI:
```bash
# Install Netlify CLI
npm install -g netlify-cli

# Build and deploy
npm run build
netlify deploy --prod
```

#### Via Drag & Drop:
1. Build: `npm run build`
2. Go to [app.netlify.com/drop](https://app.netlify.com/drop)
3. Drag `dist/` folder

#### Via GitHub:
1. Push code to GitHub
2. Connect repository in Netlify
3. Build settings:
   - **Build command**: `npm run build`
   - **Publish directory**: `dist`

### 3. GitHub Pages

#### Method 1: GitHub Actions (Automated)

1. Create `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'

      - name: Install dependencies
        working-directory: ./book-app
        run: npm ci

      - name: Build
        working-directory: ./book-app
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./book-app/dist
```

2. Enable GitHub Pages in repository settings
3. Push to main branch

#### Method 2: Manual Deploy

```bash
# Install gh-pages
npm install -g gh-pages

# Build
npm run build

# Deploy
gh-pages -d dist
```

Update `vite.config.js` for GitHub Pages:

```javascript
export default defineConfig({
  plugins: [react()],
  base: '/repository-name/', // Your repo name
})
```

### 4. AWS S3 + CloudFront

```bash
# Build
npm run build

# Install AWS CLI
# Configure: aws configure

# Sync to S3
aws s3 sync dist/ s3://your-bucket-name

# Invalidate CloudFront cache
aws cloudfront create-invalidation --distribution-id YOUR_DIST_ID --paths "/*"
```

### 5. Firebase Hosting

```bash
# Install Firebase CLI
npm install -g firebase-tools

# Login
firebase login

# Initialize
firebase init hosting

# Build
npm run build

# Deploy
firebase deploy
```

**firebase.json**:
```json
{
  "hosting": {
    "public": "dist",
    "ignore": ["firebase.json", "**/.*", "**/node_modules/**"],
    "rewrites": [
      {
        "source": "**",
        "destination": "/index.html"
      }
    ]
  }
}
```

### 6. Docker

**Dockerfile**:
```dockerfile
# Build stage
FROM node:18-alpine as build
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
RUN npm run build

# Production stage
FROM nginx:alpine
COPY --from=build /app/dist /usr/share/nginx/html
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
```

**nginx.conf**:
```nginx
server {
    listen 80;
    server_name localhost;
    root /usr/share/nginx/html;
    index index.html;

    location / {
        try_files $uri $uri/ /index.html;
    }
}
```

**Build and run**:
```bash
docker build -t book-app .
docker run -p 8080:80 book-app
```

## ⚙️ Environment Variables

If you need environment variables:

1. Create `.env.production`:
```env
VITE_API_URL=https://api.example.com
VITE_ANALYTICS_ID=UA-XXXXX-X
```

2. Access in code:
```javascript
const apiUrl = import.meta.env.VITE_API_URL;
```

## 🔧 Build Optimizations

### Analyze Bundle Size

```bash
# Install analyzer
npm install -D rollup-plugin-visualizer

# Add to vite.config.js
import { visualizer } from 'rollup-plugin-visualizer';

export default defineConfig({
  plugins: [
    react(),
    visualizer({ open: true })
  ]
});

# Build and analyze
npm run build
```

### Enable Compression

Most platforms auto-enable gzip/brotli. For custom servers:

**nginx.conf**:
```nginx
gzip on;
gzip_types text/plain text/css application/json application/javascript;
gzip_min_length 1000;
```

## 🌐 Custom Domain

### Vercel
1. Add domain in Vercel dashboard
2. Update DNS records as instructed

### Netlify
1. Add domain in Netlify dashboard
2. Update DNS to Netlify nameservers

### GitHub Pages
1. Add `CNAME` file to `public/` with your domain
2. Update DNS:
   - Type: `A`
   - Name: `@`
   - Value: GitHub Pages IPs

## ✅ Pre-Deploy Checklist

- [ ] Run `npm run build` locally to verify
- [ ] Test the production build with `npm run preview`
- [ ] Check all routes work correctly
- [ ] Verify dark/light mode switching
- [ ] Test mobile responsiveness
- [ ] Ensure all images load
- [ ] Check console for errors
- [ ] Update meta tags in `index.html`
- [ ] Add favicon
- [ ] Set up analytics (optional)

## 📊 Analytics (Optional)

### Google Analytics

Add to `index.html`:
```html
<script async src="https://www.googletagmanager.com/gtag/js?id=GA_MEASUREMENT_ID"></script>
<script>
  window.dataLayer = window.dataLayer || [];
  function gtag(){dataLayer.push(arguments);}
  gtag('js', new Date());
  gtag('config', 'GA_MEASUREMENT_ID');
</script>
```

### Plausible Analytics

```html
<script defer data-domain="yourdomain.com" src="https://plausible.io/js/script.js"></script>
```

## 🔒 Security Headers

Add to your hosting platform:

```
X-Frame-Options: DENY
X-Content-Type-Options: nosniff
Referrer-Policy: strict-origin-when-cross-origin
Permissions-Policy: camera=(), microphone=(), geolocation=()
```

## 🎯 Performance Tips

1. **Lazy load images**: Use `loading="lazy"`
2. **Code splitting**: React Router does this automatically
3. **Optimize images**: Use WebP format
4. **Enable caching**: Set cache headers
5. **Use CDN**: Most platforms provide this

## 📈 Monitoring

Use these tools to monitor your deployment:
- [Google Lighthouse](https://developers.google.com/web/tools/lighthouse)
- [WebPageTest](https://www.webpagetest.org/)
- [GTmetrix](https://gtmetrix.com/)

---

**Your book is ready to share with the world! 🌍**
