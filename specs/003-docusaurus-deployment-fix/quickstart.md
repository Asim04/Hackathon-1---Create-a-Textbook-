# Quickstart: Docusaurus Deployment Fix Workflow

**Feature**: 003-docusaurus-deployment-fix
**Purpose**: Step-by-step workflow for fixing Docusaurus routing, navigation, and GitHub Pages deployment
**Target Audience**: Developers implementing the fixes

---

## Prerequisites

Before starting, ensure you have:
- [ ] Node.js 18+ installed (`node --version`)
- [ ] npm 8+ installed (`npm --version`)
- [ ] Git repository access
- [ ] Text editor (VS Code recommended)
- [ ] GitHub repository name and username available

---

## Workflow Steps

### Step 1: Gather GitHub Repository Information

**Time**: 2 minutes

Determine your GitHub deployment details:

```bash
# Check current git remote
cd book-ai/
git remote get-url origin

# Example output: https://github.com/johndoe/Humanoid-Robotics-Textbook.git
# Extract: username = "johndoe", repo = "Humanoid-Robotics-Textbook"
```

**Record**:
- GitHub username: `_______________`
- Repository name: `_______________`

---

### Step 2: Create Static Assets Directory

**Time**: 5 minutes

```bash
# Create directory structure
mkdir -p static/img

# Create simple SVG logo (placeholder)
cat > static/img/logo.svg << 'EOF'
<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" width="40" height="40">
  <circle cx="50" cy="50" r="40" fill="#4CAF50"/>
  <text x="50" y="65" font-size="40" text-anchor="middle" fill="white" font-family="Arial">AI</text>
</svg>
EOF

# Create placeholder favicon (16x16 transparent PNG as .ico)
# Note: For production, use actual .ico file or online generator
# Placeholder: touch static/img/favicon.ico

echo "Static assets directory created"
```

**Verification**:
```bash
ls -la static/img/
# Should show: favicon.ico, logo.svg
```

---

### Step 3: Update docusaurus.config.js

**Time**: 10 minutes

Open `docusaurus.config.js` and make the following changes:

**Change 1: Add routeBasePath** (line 42)
```javascript
// BEFORE:
docs: {
  sidebarPath: './sidebars.js',
  editUrl: 'https://github.com/your-org/book-ai/tree/main/',
},

// AFTER:
docs: {
  routeBasePath: '/',  // KEY FIX: Serve docs at root
  sidebarPath: './sidebars.js',
  editUrl: 'https://github.com/[USERNAME]/[REPO_NAME]/tree/main/',
},
```

**Change 2: Update GitHub URLs** (lines 16, 23-24, 47, 76, 103)
```javascript
// Replace [USERNAME] with your GitHub username
// Replace [REPO_NAME] with your repository name

url: 'https://[USERNAME].github.io',
baseUrl: '/[REPO_NAME]/',
organizationName: '[USERNAME]',
projectName: '[REPO_NAME]',
editUrl: 'https://github.com/[USERNAME]/[REPO_NAME]/tree/main/',
href: 'https://github.com/[USERNAME]/[REPO_NAME]',
```

**Change 3: Fix footer link** (line 90)
```javascript
// BEFORE:
to: '/docs/module-1-ros2',

// AFTER:
to: '/module-1-ros2/index',  // Removed /docs/ prefix
```

**Change 4: Add Module 2 footer link** (after line 92)
```javascript
{
  label: 'Module 2: Digital Twin',
  to: '/module-2-digital-twin/index',
},
```

**Verification**:
```bash
# Check syntax (should not error)
node -c docusaurus.config.js
```

---

### Step 4: Create Homepage Redirect

**Time**: 3 minutes

```bash
# Create pages directory if it doesn't exist
mkdir -p src/pages

# Create redirect component
cat > src/pages/index.jsx << 'EOF'
import React from 'react';
import { Redirect } from '@docusaurus/router';

export default function Home() {
  return <Redirect to="/intro" />;
}
EOF

echo "Homepage redirect created"
```

**Verification**:
```bash
cat src/pages/index.jsx
# Should show React redirect component
```

---

### Step 5: Review Sidebar Configuration

**Time**: 5 minutes

Open `sidebars.js` and verify:

**Check 1: Module 1 IDs** (lines 35-55)
```javascript
// CORRECT PATTERN (already implemented):
id: 'module-1-ros2/ros2-fundamentals',      // No '01-' prefix
// Corresponds to file: docs/module-1-ros2/01-ros2-fundamentals.md
```

**Check 2: Module 2 IDs** (lines 72-92)
```javascript
// CORRECT PATTERN (already implemented):
id: 'module-2-digital-twin/gazebo-physics', // No '01-' prefix
// Corresponds to file: docs/module-2-digital-twin/01-gazebo-physics.md
```

**If IDs already match this pattern**: No changes needed ✓

**If IDs include numeric prefixes**: Remove them (e.g., change `'01-gazebo-physics'` to `'gazebo-physics'`)

---

### Step 6: Install Dependencies and Test Locally

**Time**: 5-10 minutes

```bash
# Install all dependencies
npm install

# Expected: Dependencies install without errors
# If errors occur: Check Node.js version (must be 18+)

# Start development server
npm start

# Expected: Server starts at http://localhost:3000
```

**Testing Checklist**:
- [ ] Access `http://localhost:3000/book-ai/` → should redirect to `/book-ai/intro`
- [ ] Access `http://localhost:3000/book-ai/intro` → introduction page loads
- [ ] Click Module 1 sidebar items → all chapters load without 404
- [ ] Click Module 2 sidebar items → all chapters load without 404
- [ ] Check browser console → no errors (except optional missing social card)
- [ ] Verify logo appears in navbar
- [ ] Verify favicon appears in browser tab

---

### Step 7: Test Production Build

**Time**: 5 minutes

```bash
# Build for production
npm run build

# Expected: Build completes in <2 minutes
# Expected: No errors or warnings
# Expected: build/ directory created
```

**Verification**:
```bash
# Check build output
ls -la build/

# Verify intro page exists at correct path
ls -la build/book-ai/intro/index.html
# OR (if baseUrl is /Humanoid-Robotics-Textbook/)
ls -la build/Humanoid-Robotics-Textbook/intro/index.html

# Serve build locally to test
npx serve build -p 5000

# Access: http://localhost:5000/book-ai/ (or your baseUrl)
# Verify navigation works with production paths
```

---

### Step 8: Deploy to GitHub Pages

**Time**: 10 minutes (first time), 2 minutes (subsequent)

**Option A: Manual Deployment**
```bash
# Build the site
npm run build

# Deploy build/ directory to gh-pages branch
npm run deploy
# OR use gh-pages package:
# npx gh-pages -d build
```

**Option B: GitHub Actions** (if configured)
```bash
# Push changes to main branch
git add .
git commit -m "Fix Docusaurus routing and navigation"
git push origin main

# GitHub Actions automatically builds and deploys
# Check Actions tab in GitHub repository
```

**Verification**:
- Access `https://[USERNAME].github.io/[REPO_NAME]/`
- Verify homepage redirects to intro
- Test all navigation links
- Check browser console for errors
- Verify images load

---

### Step 9: Validate All Requirements

**Time**: 10 minutes

Run through the acceptance scenarios from spec.md:

**User Story 1 - Local Development**:
- [x] npm install completes without errors
- [x] npm start serves at port 3000
- [x] http://localhost:3000/book-ai/ redirects to intro
- [x] All sidebar links work without 404

**User Story 2 - GitHub Pages**:
- [x] npm run build completes successfully
- [x] Deployed site loads at GitHub Pages URL
- [x] All navigation works in production
- [x] All images load correctly

**User Story 3 - Navigation**:
- [x] All sidebar items clickable and functional
- [x] Navbar "Textbook" link works
- [x] Navbar "GitHub" link opens repository
- [x] Internal markdown links work
- [x] Sidebar IDs match filename patterns

---

## Troubleshooting

### Issue 1: npm install fails with EBADF or ENOMEM errors

**Solution**:
```bash
# Clear npm cache
npm cache clean --force

# Retry with force flag
npm install --force

# OR use yarn instead
npm install -g yarn
yarn install
```

### Issue 2: 404 error at http://localhost:3000/book-ai/

**Solution**:
- Verify `routeBasePath: '/'` is set in docusaurus.config.js docs preset
- Verify `src/pages/index.jsx` exists with redirect to `/intro`
- Access direct URL: `http://localhost:3000/book-ai/intro`

### Issue 3: Sidebar links don't work

**Solution**:
- Check sidebar IDs match file paths in docs/ directory
- Verify no typos in sidebar ID strings
- Ensure filenames use correct slugs (no spaces, lowercase with hyphens)

### Issue 4: Images don't load

**Solution**:
- Verify `static/img/` directory exists
- Check image filenames match references in config and markdown
- Use leading slash for static images: `/img/file.png` not `img/file.png`

### Issue 5: GitHub Pages shows 404 for all pages

**Solution**:
- Verify `baseUrl` in docusaurus.config.js matches repository name exactly
- Check GitHub Pages is enabled in repository settings
- Ensure gh-pages branch has build output
- Wait 2-3 minutes for GitHub Pages to update after deployment

---

## Success Metrics

After completing all steps, you should achieve:
- ✅ Zero 404 errors on local development
- ✅ Zero 404 errors on GitHub Pages production
- ✅ Build completes in <2 minutes
- ✅ All navigation links functional
- ✅ All images display correctly
- ✅ No console errors

---

**Quickstart Version**: 1.0.0
**Last Updated**: 2025-12-19
**Estimated Total Time**: 45-60 minutes (first-time setup)
