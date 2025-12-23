# Research Documentation: Docusaurus Deployment and Navigation Fix

**Feature**: 003-docusaurus-deployment-fix
**Created**: 2025-12-19
**Purpose**: Technical research for fixing Docusaurus routing, navigation, and GitHub Pages deployment issues

---

## Decision 1: Routing Configuration - Docs at Root vs /docs Prefix

**Options**:
- **A**: Keep docs at `/docs/` path (default Docusaurus behavior)
- **B**: Move docs to root `/` path using `routeBasePath: '/'`

**Decision**: **B - Move docs to root** (`routeBasePath: '/'`)

**Rationale**:
- User requirement FR-003: "Site homepage MUST be accessible at root URL (`/`) without requiring `/docs` prefix"
- User reported 404 error at `http://localhost:3000/book-ai/` suggests expectation of root-level access
- Textbook is the primary content (no blog, no landing page), so docs should be at root for simplicity
- Eliminates confusion between `/book-ai/` (base URL) and `/book-ai/docs/` (docs path)

**Implementation**:
```javascript
// docusaurus.config.js
presets: [
  [
    'classic',
    {
      docs: {
        routeBasePath: '/',  // Serve docs at root instead of /docs/
        sidebarPath: './sidebars.js',
        editUrl: 'https://github.com/your-org/book-ai/tree/main/',
      },
      blog: false,
      theme: {
        customCss: './src/css/custom.css',
      },
    },
  ],
],
```

**Trade-offs**:
- **Pro**: Simpler URLs (`/intro` vs `/docs/intro`)
- **Pro**: Homepage (`/`) directly accesses docs
- **Con**: Cannot use `/` for custom landing page (pages conflict)
- **Mitigation**: If custom landing needed later, move docs back to `/docs/` and create `src/pages/index.jsx`

**Sources**: [Docusaurus Docs Plugin Configuration](https://docusaurus.io/docs/api/plugins/@docusaurus/plugin-content-docs#routeBasePath)

---

## Decision 2: Base URL for Local Development vs GitHub Pages

**Options**:
- **A**: Always use `/book-ai/` for both local and production
- **B**: Use `/` for local, `/book-ai/` for production via environment variables
- **C**: Use `/book-ai/` for production, developers access `http://localhost:3000/book-ai/intro`

**Decision**: **A - Always use `/book-ai/`** (keep current config)

**Rationale**:
- Current `docusaurus.config.js` has `baseUrl: '/book-ai/'` (line 19)
- This matches GitHub Pages project repository pattern: `https://[user].github.io/book-ai/`
- Keeping same base URL for local and production ensures routing consistency
- Developers can access local site at `http://localhost:3000/book-ai/intro` (matches production URL structure)
- No environment variable complexity needed

**Implementation**:
```javascript
// docusaurus.config.js (KEEP AS-IS)
const config = {
  url: 'https://your-org.github.io',   // Production URL
  baseUrl: '/book-ai/',                  // Matches GitHub repo name
  organizationName: 'your-org',          // Update with actual GitHub username
  projectName: 'book-ai',                // Matches repository name
};
```

**Trade-offs**:
- **Con**: Local URLs require `/book-ai/` prefix (e.g., `http://localhost:3000/book-ai/intro`)
- **Pro**: Exact parity between local and production (catches base URL issues early)
- **Pro**: No complex environment switching logic

**Alternative Considered**: Environment-based base URL (rejected for simplicity)

**Sources**: [Docusaurus GitHub Pages Deployment Guide](https://docusaurus.io/docs/deployment#deploying-to-github-pages)

---

## Decision 3: Sidebar ID Naming Convention

**Options**:
- **A**: Match filenames exactly including `01-` prefixes (e.g., `id: 'module-2-digital-twin/01-gazebo-physics'`)
- **B**: Use simplified IDs without numeric prefixes (e.g., `id: 'module-2-digital-twin/gazebo-physics'`)

**Decision**: **B - Simplified IDs** (already implemented in current sidebars.js)

**Rationale**:
- Current `sidebars.js` lines 72-87 already use simplified IDs: `gazebo-physics`, `unity-visualization`, `sensor-simulation`, `lab-digital-twin`
- This is Docusaurus best practice: IDs are semantic references, filenames are organizational
- Numeric prefixes (`01-`, `02-`) are implementation details for file sorting
- Simplified IDs are more maintainable (renumbering files doesn't break sidebar config)

**Current Implementation** (already correct):
```javascript
// sidebars.js (lines 72-87) - KEEP AS-IS
{
  type: 'doc',
  id: 'module-2-digital-twin/gazebo-physics',      // Simplified ID
  label: 'Chapter 1: Gazebo Physics Simulation',
},
// Corresponding file: docs/module-2-digital-twin/01-gazebo-physics.md
```

**Mapping**:
- **Filename**: `01-gazebo-physics.md` (numeric prefix for alphabetical sorting)
- **Sidebar ID**: `gazebo-physics` (clean, semantic reference)
- **Docusaurus auto-derives ID** from filename (strips numeric prefixes automatically)

**Trade-offs**:
- **Pro**: Semantic IDs independent of file ordering
- **Pro**: Renumbering chapters doesn't require sidebar updates
- **Con**: Requires understanding that IDs != filenames (documented via comments)

**Sources**: [Docusaurus Sidebar Items](https://docusaurus.io/docs/sidebar/items)

---

## Decision 4: Static Assets Directory Structure

**Options**:
- **A**: Create `static/img/` with minimal assets (favicon only)
- **B**: Create `static/img/` with full asset set (favicon, logo, social card)

**Decision**: **B - Full asset set** (favicon, logo, social card)

**Rationale**:
- Current `docusaurus.config.js` references 3 assets:
  - Line 13: `favicon: 'img/favicon.ico'`
  - Line 61: `image: 'img/docusaurus-social-card.jpg'` (social preview)
  - Line 66: `src: 'img/logo.svg'` (navbar logo)
- All 3 are currently missing (no `static/` directory exists)
- Missing assets cause console errors and broken UI (user-reported issue)
- Creating all 3 provides complete professional appearance

**Directory Structure**:
```
static/
└── img/
    ├── favicon.ico              # Browser tab icon
    ├── logo.svg                 # Navbar logo
    └── docusaurus-social-card.jpg  # Social media preview image
```

**Trade-offs**:
- **Pro**: Eliminates all console errors for missing assets
- **Pro**: Professional appearance with logo and favicon
- **Con**: Requires creating/sourcing 3 asset files (can use placeholders)
- **Mitigation**: Create simple SVG logo programmatically, use generic favicon, skip social card initially

**Sources**: [Docusaurus Static Assets](https://docusaurus.io/docs/static-assets)

---

## Decision 5: Homepage Redirect Strategy

**Options**:
- **A**: No redirect - let Docusaurus 404 at `/` and require users to access `/intro`
- **B**: Create `src/pages/index.jsx` redirecting `/` to `/intro`
- **C**: Create landing page at `src/pages/index.jsx` with custom content

**Decision**: **B - Redirect page** (`src/pages/index.jsx` → `/intro`)

**Rationale**:
- User requirement FR-003: "Site homepage MUST be accessible at root URL"
- With `routeBasePath: '/'`, docs path `/intro` exists, but `/` (exact root) does not
- Simple redirect ensures users accessing `/` land on introduction page
- Avoids creating duplicate landing page content (intro.md already serves as introduction)

**Implementation**:
```javascript
// src/pages/index.jsx
import React from 'react';
import { Redirect } from '@docusaurus/router';

export default function Home() {
  return <Redirect to="/intro" />;
}
```

**Trade-offs**:
- **Pro**: Seamless user experience - `/` works immediately
- **Pro**: Minimal code (3-line redirect component)
- **Con**: Users don't see a flashy landing page (acceptable for textbook use case)

**Sources**: [Docusaurus Pages](https://docusaurus.io/docs/creating-pages)

---

## Decision 6: GitHub Repository URL Configuration

**Options**:
- **A**: Keep placeholder "your-org" URLs (requires manual replacement later)
- **B**: Determine actual GitHub username/repo and update config
- **C**: Use repository variables to auto-detect at build time

**Decision**: **B - Update with actual values** (requires user clarification)

**Rationale**:
- Current config has 6 placeholder URLs with "your-org" (lines 16, 23-24, 47, 76, 90, 103)
- These cause broken GitHub links in navbar and footer
- Need actual GitHub username and repository name for correct deployment
- **[NEEDS CLARIFICATION: GitHub username and repository name]**

**Placeholder Locations**:
1. Line 16: `url: 'https://your-org.github.io'`
2. Line 23: `organizationName: 'your-org'`
3. Line 24: `projectName: 'book-ai'` (may need update to match actual repo)
4. Line 47: `editUrl: 'https://github.com/your-org/book-ai/tree/main/'`
5. Line 76: `href: 'https://github.com/your-org/book-ai'`
6. Line 90: `to: '/docs/module-1-ros2'` (needs update after routeBasePath change)
7. Line 103: `href: 'https://github.com/your-org/book-ai'`

**Implementation** (pending clarification):
```javascript
// Example if GitHub user is "johndoe" and repo is "Humanoid-Robotics-Textbook"
const config = {
  url: 'https://johndoe.github.io',
  baseUrl: '/Humanoid-Robotics-Textbook/',
  organizationName: 'johndoe',
  projectName: 'Humanoid-Robotics-Textbook',
};
```

**Trade-offs**:
- **Pro**: Functional GitHub links throughout site
- **Con**: Requires knowing actual GitHub details
- **Mitigation**: Query user for GitHub username and repository name

---

## Technical Decisions Summary

| Decision | Choice | Status | Notes |
|----------|--------|--------|-------|
| Routing Path | Docs at root `/` | ✅ Decided | Use `routeBasePath: '/'` |
| Base URL | `/book-ai/` consistent | ✅ Decided | Keep for local+production |
| Sidebar IDs | Simplified (no `01-`) | ✅ Already correct | Keep current pattern |
| Static Assets | Full set (favicon, logo, social) | ✅ Decided | Create `static/img/` |
| Homepage | Redirect `/` → `/intro` | ✅ Decided | Create `src/pages/index.jsx` |
| GitHub URLs | Update placeholders | ⚠️ Needs clarification | Require GitHub username + repo name |

---

## Configuration Files to Modify

### 1. `docusaurus.config.js` (7 changes)

| Line | Current | New | Reason |
|------|---------|-----|--------|
| 16 | `url: 'https://your-org.github.io'` | `url: 'https://[ACTUAL_USER].github.io'` | Real GitHub username |
| 19 | `baseUrl: '/book-ai/'` | `baseUrl: '/[ACTUAL_REPO]/'` | Match actual repo name |
| 23 | `organizationName: 'your-org'` | `organizationName: '[ACTUAL_USER]'` | Real GitHub username |
| 24 | `projectName: 'book-ai'` | `projectName: '[ACTUAL_REPO]'` | Match repo name |
| 42-48 | `docs: {...}` | Add `routeBasePath: '/'` | Serve docs at root |
| 47 | `editUrl: 'https://github.com/your-org/book-ai/...'` | Update with actual URL | Real repo URL |
| 76, 103 | `href: 'https://github.com/your-org/book-ai'` | Update with actual URL | Real repo URL |

### 2. `sidebars.js` (1 potential change)

| Line | Current | New | Reason |
|------|---------|-----|--------|
| 35-55 | Module 1 IDs without `01-` prefix | KEEP AS-IS ✓ | Already follows best practice |
| 72-92 | Module 2 IDs without `01-` prefix | KEEP AS-IS ✓ | Already follows best practice |

**Conclusion**: Sidebar IDs are already correctly simplified. No changes needed.

### 3. Footer Links (1 change)

| Line | Current | New | Reason |
|------|---------|-----|--------|
| 90 | `to: '/docs/module-1-ros2'` | `to: '/module-1-ros2/index'` | Remove `/docs/` prefix after routeBasePath change |

---

## Files to Create

### 1. `src/pages/index.jsx`

**Purpose**: Redirect root `/` to `/intro` page
**Content**: React component using `@docusaurus/router` Redirect

### 2. `static/img/favicon.ico`

**Purpose**: Browser tab icon
**Source**: Can use simple generated icon or placeholder

### 3. `static/img/logo.svg`

**Purpose**: Navbar logo
**Source**: Simple SVG with textbook initials or robot icon

### 4. `static/img/docusaurus-social-card.jpg` (Optional)

**Purpose**: Social media preview image
**Source**: Can skip initially (not critical for functionality)

---

## Validation Steps

### Local Development Testing
1. Run `npm install` - verify completes without errors
2. Run `npm start` - verify server starts on port 3000
3. Access `http://localhost:3000/book-ai/` - should redirect to `/book-ai/intro`
4. Access `http://localhost:3000/book-ai/intro` - should load introduction page
5. Click all sidebar links - verify no 404 errors
6. Check browser console - verify no missing asset errors

### Production Build Testing
1. Run `npm run build` - verify completes in <2 minutes
2. Check `build/` directory contains HTML files at correct paths
3. Verify `build/book-ai/intro/index.html` exists (baseURL applied)
4. Serve build locally: `npx serve build -p 5000`
5. Access `http://localhost:5000/book-ai/` - test routing with production paths

### GitHub Pages Deployment Testing
1. Deploy `build/` directory to GitHub Pages (gh-pages branch or repo settings)
2. Access `https://[user].github.io/[repo]/` - verify homepage redirects
3. Navigate all sidebar items - verify no 404 errors
4. Check browser console on deployed site - verify assets load

---

**Research Version**: 1.0.0
**Last Updated**: 2025-12-19
**Clarifications Needed**: GitHub username and repository name for URL configuration
