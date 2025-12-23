# Implementation Plan: Docusaurus Deployment and Navigation Fix

**Branch**: `003-docusaurus-deployment-fix` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-docusaurus-deployment-fix/spec.md`

## Summary

This plan addresses critical Docusaurus deployment and navigation issues causing 404 errors in both local development and GitHub Pages production. The primary requirement is to fix routing configuration so that `http://localhost:3000/` serves the introduction page (not 404), configure correct base URL for GitHub Pages deployment, standardize sidebar navigation to match file naming patterns, and create missing static assets directory for images.

**Technical Approach**: Modify `docusaurus.config.js` to set `routeBasePath: '/'` in docs preset configuration (eliminating `/docs/` prefix), update `url` and `baseUrl` for GitHub Pages deployment, standardize sidebar IDs in `sidebars.js` to match filename patterns (`01-gazebo-physics` format), create `static/img/` directory structure for assets, and audit markdown files for hardcoded URLs requiring relative path fixes.

## Technical Context

**Language/Version**: JavaScript (ES6+), Node.js 18+, npm 9+
**Primary Dependencies**: Docusaurus 3.x (@docusaurus/core, @docusaurus/preset-classic), React 18, prism-react-renderer
**Storage**: File-based (markdown in docs/, static assets in static/img/, build artifacts in build/)
**Testing**: Manual validation (npm start, npm run build, browser testing), link checking via Docusaurus build
**Target Platform**: Static site (local development via webpack-dev-server, production via GitHub Pages)
**Project Type**: Static documentation site (Docusaurus-based textbook)
**Performance Goals**: Build time <2 minutes, dev server start <10 seconds, page load <1 second
**Constraints**: Must work with standard Docusaurus workflows (no webpack ejecting), GitHub Pages deployment compatibility, relative path markdown links
**Scale/Scope**: 2 modules (Module 1, Module 2), 10+ chapters total, 15+ code examples, navigation hierarchy 3 levels deep

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Compliance | Evidence/Plan |
|-----------|------------|---------------|
| **I. Accuracy** | ✅ PASS | Configuration changes based on official Docusaurus 3.x documentation. No factual claims about Docusaurus behavior—only standard configuration patterns. |
| **II. Clarity** | ✅ PASS | Implementation plan includes inline comments in config files explaining each setting. Technical decisions documented with rationale. |
| **III. Reproducibility** | ✅ PASS | All changes are deterministic: same configuration → same routing behavior. Testing steps provided for both local (npm start) and production (npm run build). |
| **IV. Rigor** | ✅ PASS | Deployment configuration follows official Docusaurus documentation and GitHub Pages deployment guide. No speculation—all patterns validated. |
| **V. Originality** | ✅ PASS | Configuration changes are standard Docusaurus setup, not copied code. Inline comments original. |
| **VI. Modularity** | ✅ PASS | Fixes are self-contained configuration changes. Does not affect Module 1 or Module 2 content. Backward compatible with existing markdown files. |

**Status**: All 6 constitution principles satisfied. No violations. Proceed to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/003-docusaurus-deployment-fix/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (Docusaurus configuration research)
├── data-model.md        # Phase 1 output (Configuration entities)
├── quickstart.md        # Phase 1 output (Deployment workflow)
├── contracts/           # Phase 1 output (Expected configurations)
│   ├── docusaurus-config-contract.js  # Template for correct config
│   └── sidebar-config-contract.js     # Template for sidebar IDs
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Docusaurus Project Structure (existing)

```text
book-ai/ (repository root)
├── docs/                         # Markdown content (Module 1, Module 2)
│   ├── intro.md                  # Introduction page
│   ├── module-1-ros2/            # Module 1 chapters
│   └── module-2-digital-twin/    # Module 2 chapters
├── src/                          # Docusaurus source (CSS, pages)
│   ├── css/
│   │   └── custom.css
│   └── pages/                    # Custom pages (homepage redirect)
│       └── index.jsx             # TO BE CREATED (homepage redirect)
├── static/                       # TO BE CREATED (static assets)
│   └── img/                      # TO BE CREATED (favicon, logo, images)
│       ├── favicon.ico           # TO BE CREATED
│       └── logo.svg              # TO BE CREATED
├── docusaurus.config.js          # TO BE MODIFIED (routing, baseURL)
├── sidebars.js                   # TO BE REVIEWED (IDs match files)
├── package.json                  # Docusaurus dependencies
└── node_modules/                 # TO BE CREATED (npm install)
```

**Structure Decision**: This is a Docusaurus static site configuration fix, not a source code project. The "source code" is the Docusaurus configuration files (`docusaurus.config.js`, `sidebars.js`) and optional redirect page (`src/pages/index.jsx`). No backend, frontend, or mobile components. Focus is on configuration file modifications and directory structure creation.

## Complexity Tracking

**Status**: Not applicable - no constitution violations. All configuration changes follow standard Docusaurus patterns.

---

## Implementation Phases

### Phase 1: Static Assets Setup (US1 partial)

**Dependencies**: None
**Duration**: 10 minutes

**Tasks**:
1. Create `static/img/` directory structure
2. Create `static/img/logo.svg` (simple SVG with textbook initials or robot icon)
3. Create or download `static/img/favicon.ico` (16x16 icon file)
4. Optionally create `static/img/docusaurus-social-card.jpg` (1200x630 social preview)

**Acceptance**: Directory exists with at minimum favicon.ico and logo.svg files

---

### Phase 2: Configuration Files Update (US1, US2, US3)

**Dependencies**: Phase 1 complete
**Duration**: 20 minutes

**Tasks**:
1. Update `docusaurus.config.js`:
   - Add `routeBasePath: '/'` to docs preset configuration (line ~42)
   - Replace all "your-org" placeholders with actual GitHub username (lines 16, 23, 47, 76, 103)
   - Replace "book-ai" with actual repository name if different (lines 19, 24)
   - Fix footer module link: change `/docs/module-1-ros2` to `/module-1-ros2/index` (line 90)
   - Add Module 2 footer link (after line 92)
   - Add inline comments explaining each configuration setting

2. Review `sidebars.js`:
   - Verify all IDs follow simplified pattern (no `01-` prefixes)
   - Confirm all IDs have corresponding markdown files
   - No changes needed if already using simplified IDs

3. Create `src/pages/index.jsx`:
   - Implement redirect from `/` to `/intro`
   - Use `@docusaurus/router` Redirect component

**Acceptance**: Configuration files updated with inline comments, homepage redirect created

---

### Phase 3: Local Development Testing (US1)

**Dependencies**: Phase 2 complete
**Duration**: 15 minutes

**Tasks**:
1. Install dependencies: `npm install`
2. Start dev server: `npm start`
3. Test root URL: Access `http://localhost:3000/[baseUrl]/` → should redirect to `/intro`
4. Test direct intro: Access `http://localhost:3000/[baseUrl]/intro` → should load introduction
5. Test sidebar navigation: Click all Module 1 and Module 2 chapters → verify no 404 errors
6. Verify images: Check logo in navbar, favicon in browser tab
7. Check console: Verify no missing asset errors

**Acceptance**: All User Story 1 acceptance scenarios pass (4/4)

---

### Phase 4: Production Build Testing (US2)

**Dependencies**: Phase 3 complete (local dev working)
**Duration**: 10 minutes

**Tasks**:
1. Run production build: `npm run build`
2. Verify build completes without errors (<2 minutes)
3. Check build output: Verify `build/[baseUrl]/intro/index.html` exists
4. Test build locally: `npx serve build -p 5000`
5. Access `http://localhost:5000/[baseUrl]/` → test with production paths
6. Verify all navigation and assets work in build

**Acceptance**: Build succeeds, all URLs resolve correctly with baseUrl prefix

---

### Phase 5: GitHub Pages Deployment (US2)

**Dependencies**: Phase 4 complete (build tested)
**Duration**: 10-15 minutes

**Tasks**:
1. Deploy to GitHub Pages:
   - Manual: Copy `build/` contents to gh-pages branch
   - Automated: Run `npm run deploy` (if configured)
   - Actions: Push to main and wait for workflow

2. Verify GitHub Pages settings:
   - Repository Settings → Pages
   - Source: gh-pages branch (or main branch /docs folder)
   - Custom domain: None (use default GitHub Pages URL)

3. Test deployed site:
   - Access `https://[USERNAME].github.io/[REPO_NAME]/`
   - Verify homepage redirects to intro
   - Test all navigation links
   - Check all images load
   - Verify no console errors

**Acceptance**: All User Story 2 acceptance scenarios pass (4/4)

---

### Phase 6: Navigation Validation (US3)

**Dependencies**: Phase 5 complete (deployed site working)
**Duration**: 15 minutes

**Tasks**:
1. Create navigation test checklist:
   - List all sidebar items (intro, Module 1 chapters, Module 2 chapters, references)
   - List all navbar items ("Textbook", "GitHub")
   - List all footer links (Module 1, Module 2, GitHub, ROS 2 docs)

2. Manual testing:
   - Click each sidebar item → verify page loads
   - Click each navbar item → verify destination correct
   - Click each footer link → verify target opens
   - Test internal markdown links in chapters → verify relative paths work

3. Automated link checking (optional):
   - Install: `npm install -g linkinator`
   - Run: `linkinator http://localhost:3000/book-ai/ --recurse`
   - Review output for broken links

**Acceptance**: All User Story 3 acceptance scenarios pass (5/5), 100% links functional

---

## Summary

**Total Phases**: 6 (Static Assets → Config Update → Local Test → Build Test → Deploy → Navigation Validation)
**Estimated Duration**: 60-80 minutes
**Critical Path**: Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5 → Phase 6 (sequential)

**Key Files Modified**:
- `docusaurus.config.js` (7 changes: routeBasePath, GitHub URLs, footer links)
- `src/pages/index.jsx` (new file: homepage redirect)
- `static/img/` (new directory: favicon, logo assets)

**Key Files Reviewed** (no changes if already correct):
- `sidebars.js` (verify simplified ID pattern)

**Risks Mitigated**:
- Base URL confusion (inline comments added)
- Sidebar ID mismatch (validation against filenames)
- Missing assets (static/img/ created with required files)
- Hardcoded URLs (footer links updated for routeBasePath change)

---

## Next Steps

1. **Immediate**: Run `/sp.tasks` to generate detailed task breakdown from this plan
2. **After tasks created**: Run `/sp.implement` to execute all configuration fixes
3. **Manual action required**: Provide actual GitHub username and repository name for URL updates

---

**Plan Version**: 1.0.0
**Last Updated**: 2025-12-19
**Status**: ✅ READY FOR TASK GENERATION (`/sp.tasks`)

