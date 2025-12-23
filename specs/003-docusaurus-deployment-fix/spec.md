# Feature Specification: Docusaurus Deployment and Navigation Fix

**Feature Branch**: `003-docusaurus-deployment-fix`
**Created**: 2025-12-19
**Status**: Draft
**Input**: User description: "Fix Docusaurus installation, routing, navigation, images, and GitHub Pages deployment so the site shows no 'Page Not Found' errors on localhost or production."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Local Development Access (Priority: P1)

A developer runs `npm start` and can immediately access the textbook at `http://localhost:3000/` (root) or `http://localhost:3000/intro` without encountering 404 errors. The homepage displays correctly with working navigation, images, and links to all modules.

**Why this priority**: Local development is the foundation for all content creation and testing. Without a working local environment, no further development or fixes can proceed. This is a blocking issue affecting all contributors.

**Independent Test**: Can be tested by running `npm install && npm start`, then accessing `http://localhost:3000/` in a browser. Success means the site loads without 404 errors and displays the introduction page with navigation sidebar.

**Acceptance Scenarios**:

1. **Given** a fresh clone of the repository, **When** developer runs `npm install` and `npm start`, **Then** server starts on port 3000 without errors
2. **Given** the dev server is running, **When** user navigates to `http://localhost:3000/`, **Then** the introduction page loads without 404 errors
3. **Given** the site is loaded locally, **When** user clicks any sidebar navigation link, **Then** the corresponding page loads without 404 errors
4. **Given** markdown files contain relative image paths, **When** pages render, **Then** all images display correctly from `static/img/` directory

---

### User Story 2 - GitHub Pages Production Deployment (Priority: P2)

A maintainer runs `npm run build` to generate production assets, and the site deploys successfully to GitHub Pages at the correct URL path (`/book-ai/` or `/Humanoid-Robotics-Textbook/`). All navigation, images, and links work correctly in the deployed environment.

**Why this priority**: Production deployment is essential for sharing the textbook with students and instructors. While local development is more critical, a broken production deployment prevents actual use of the textbook.

**Independent Test**: Can be tested by running `npm run build`, verifying the build succeeds without errors, then deploying to GitHub Pages and accessing the live URL. Success means all pages load, navigation works, and no console errors appear.

**Acceptance Scenarios**:

1. **Given** the codebase is ready for deployment, **When** developer runs `npm run build`, **Then** build completes successfully without errors and generates `build/` directory
2. **Given** the site is deployed to GitHub Pages, **When** user accesses `https://[username].github.io/[repo-name]/`, **Then** the homepage loads correctly with correct base URL applied to all assets
3. **Given** the deployed site, **When** user navigates between pages, **Then** all internal links work with correct base path (`/book-ai/` or repo name)
4. **Given** the deployed site, **When** user views pages with images, **Then** all images load correctly with proper base URL paths

---

### User Story 3 - Complete Navigation System (Priority: P3)

A student or instructor accesses the textbook and can easily navigate between modules, chapters, and references using the sidebar. The navbar provides quick access to docs, GitHub repository, and textbook title. All links are functional and point to correct locations.

**Why this priority**: While not blocking development or deployment, broken navigation severely degrades user experience. Users should be able to explore the full textbook structure without encountering dead links.

**Independent Test**: Can be tested by manually clicking through all navigation elements (sidebar items, navbar links, internal markdown links) and verifying each destination loads correctly. Success means 100% of navigation elements work.

**Acceptance Scenarios**:

1. **Given** a user is on any page, **When** they click a sidebar item (Module 1, Module 2, chapter, references), **Then** the target page loads without 404 errors
2. **Given** a user views the navbar, **When** they click the "Docs" link, **Then** they navigate to the textbook introduction
3. **Given** a user views the navbar, **When** they click the GitHub link, **Then** a new tab opens to the correct repository URL
4. **Given** a user reads a chapter, **When** they click internal markdown links (e.g., `[Chapter 2](./02-unity-visualization.md)`), **Then** the linked page loads correctly
5. **Given** Module 1 and Module 2 files are named with `01-`, `02-` prefixes, **When** sidebars.js is configured, **Then** sidebar IDs match filename patterns for consistency

---

### Edge Cases

- **Missing `static/` directory**: What happens when `static/img/` doesn't exist? (Expected: Build fails or images don't load, fix requires creating directory structure)
- **Base URL mismatch**: What if `baseUrl` is set to `/book-ai/` but GitHub Pages uses different repo name? (Expected: 404 errors on all assets, requires updating `docusaurus.config.js`)
- **File not found at root**: What happens when user accesses `/` with `baseUrl: '/book-ai/'`? (Expected: 404 error, fix requires setting `routeBasePath: '/'` in docs config)
- **Hardcoded `/docs` URLs**: What if markdown files contain hardcoded `/docs/` paths? (Expected: 404 errors in production, requires fixing relative paths)
- **Missing favicon**: What happens when `static/img/favicon.ico` is missing? (Expected: Console error but site still works, optional fix)
- **Inconsistent sidebar IDs**: What if sidebars.js uses `gazebo-physics` but filename is `01-gazebo-physics.md`? (Expected: Docusaurus may fail to match files, requires ID consistency)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Docusaurus MUST be correctly installed with all dependencies via `npm install` completing without errors
- **FR-002**: Local development server MUST start successfully with `npm start` and serve site at `http://localhost:3000/`
- **FR-003**: Site homepage MUST be accessible at root URL (`/`) in local development without requiring `/docs` prefix
- **FR-004**: Docs configuration MUST use `routeBasePath: '/'` to eliminate `/docs/` prefix from URLs
- **FR-005**: `docusaurus.config.js` MUST have correct `url` and `baseUrl` configured for GitHub Pages deployment target
- **FR-006**: Base URL MUST match the GitHub Pages repository name (e.g., `/book-ai/` or `/Humanoid-Robotics-Textbook/`)
- **FR-007**: Navbar MUST include functional links: "Docs" (to intro), "GitHub" (to repository), and textbook title
- **FR-008**: Sidebar MUST include all markdown files from `docs/` directory organized by module structure
- **FR-009**: Sidebar IDs in `sidebars.js` MUST match actual filename patterns (with or without `01-`, `02-` prefixes consistently)
- **FR-010**: Static assets directory `static/img/` MUST exist for logo, favicon, and markdown images
- **FR-011**: All relative image paths in markdown files MUST resolve correctly from `static/img/` directory
- **FR-012**: Internal markdown links (e.g., `./02-unity-visualization.md`) MUST not contain hardcoded `/docs/` prefixes
- **FR-013**: Production build MUST complete successfully with `npm run build` without errors or warnings
- **FR-014**: Deployed site on GitHub Pages MUST have all assets load with correct base URL paths
- **FR-015**: All internal navigation links (sidebar, navbar, markdown) MUST work without producing 404 errors

### Key Entities

- **Docusaurus Configuration**: `docusaurus.config.js` file containing site metadata (`url`, `baseUrl`, `organizationName`, `projectName`), navbar configuration, and preset options including docs `routeBasePath`
- **Sidebar Configuration**: `sidebars.js` file defining navigation structure with document IDs matching markdown file paths
- **Static Assets**: Files in `static/` directory including logo, favicon, and images referenced from markdown
- **Markdown Content**: Chapter files in `docs/` directory with front matter (`sidebar_position`, `title`, `description`) and internal links

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Developer can run `npm install && npm start` and access the site at `http://localhost:3000/` within 30 seconds without 404 errors
- **SC-002**: 100% of sidebar navigation links work in both local development and production deployment (zero 404 errors)
- **SC-003**: Production build (`npm run build`) completes in under 2 minutes without errors or warnings
- **SC-004**: All images display correctly in both local and production environments (100% image load success rate)
- **SC-005**: Site deploys to GitHub Pages without manual URL path corrections (base URL automatically applied)
- **SC-006**: Zero console errors related to missing files, incorrect paths, or broken links when accessing any page
- **SC-007**: Users can navigate from introduction to any module chapter and back using sidebar within 3 clicks
- **SC-008**: Internal markdown links between chapters work without requiring manual URL edits (relative paths resolve correctly)

## Dependencies *(mandatory)*

### Prerequisites

- **Node.js 18+**: Required for Docusaurus 3.x compatibility
- **npm 8+**: Package manager for installing Docusaurus dependencies
- **Git**: Version control system for branch management and GitHub Pages deployment
- **GitHub Repository**: Existing repository with appropriate access for GitHub Pages deployment

### External Dependencies

- **Docusaurus 3.x**: Static site generator framework
- **@docusaurus/preset-classic**: Standard Docusaurus preset including docs and theme
- **prism-react-renderer**: Syntax highlighting for code blocks (included in preset)
- **GitHub Pages**: Hosting platform for production deployment

## Assumptions *(mandatory)*

- **Repository is already initialized**: Git repository exists with commit history
- **Content structure is finalized**: `docs/` directory structure with Module 1 and Module 2 content is complete
- **GitHub Pages is enabled**: Repository settings allow GitHub Pages deployment from `gh-pages` branch or `/docs` folder
- **Base URL is known**: GitHub repository name is determined (either `/book-ai/` or `/Humanoid-Robotics-Textbook/`)
- **Node.js environment is available**: Developer has Node.js 18+ installed locally
- **No custom domain**: Deployment uses standard GitHub Pages URL pattern (`https://[username].github.io/[repo]`)
- **Standard Docusaurus structure**: Project follows default Docusaurus file organization conventions
- **Markdown links follow conventions**: Internal links use relative paths (e.g., `./file.md`) not absolute paths

## Out of Scope *(mandatory)*

- **Custom domain configuration**: Setting up `CNAME` or custom DNS for vanity URL
- **CI/CD automation**: GitHub Actions workflows for automatic deployment (manual deployment assumed)
- **Search functionality**: Algolia DocSearch integration or local search plugin
- **Multi-language support**: Internationalization (i18n) for translated content
- **Dark mode theming**: Custom color schemes beyond default Docusaurus theme
- **Advanced Docusaurus features**: Versioning, blog, landing pages, custom plugins
- **Content creation**: Writing or editing module/chapter markdown content
- **SEO optimization**: Meta tags, structured data, sitemap configuration beyond Docusaurus defaults
- **Analytics integration**: Google Analytics, Plausible, or other tracking tools
- **Performance optimization**: Bundle size reduction, image optimization, lazy loading

## Non-Functional Requirements *(if applicable)*

- **NFR-001**: Site build time MUST be under 2 minutes for production builds on standard hardware
- **NFR-002**: Local development server MUST start in under 10 seconds after `npm start`
- **NFR-003**: Configuration changes in `docusaurus.config.js` or `sidebars.js` MUST not require complex workarounds (simple, maintainable solutions)
- **NFR-004**: All configuration files MUST include inline comments explaining key settings (especially `url`, `baseUrl`, `routeBasePath`)
- **NFR-005**: Solution MUST work with standard Docusaurus workflows without custom webpack configurations or ejecting
- **NFR-006**: Documentation of fixes MUST be embedded as code comments for future maintainability

## Risks & Mitigations *(if applicable)*

### Risk 1: Base URL Configuration Confusion
**Impact**: High - Incorrect `baseUrl` causes all assets to fail loading in production
**Mitigation**: Add inline comments in `docusaurus.config.js` explaining when to use `/book-ai/` vs `/` and document testing procedure for both local and production

### Risk 2: Sidebar ID Mismatch
**Impact**: Medium - Inconsistent sidebar IDs cause navigation failures or build errors
**Mitigation**: Standardize all sidebar IDs to match filename patterns (either remove `01-` prefixes from IDs or add them consistently), document the chosen convention

### Risk 3: Missing Static Assets Directory
**Impact**: Medium - Images fail to load, causing poor user experience
**Mitigation**: Create `static/img/` directory structure, add placeholder favicon, document required directory structure in setup guide

### Risk 4: Hardcoded URLs in Markdown
**Impact**: Low-Medium - Internal links break when `baseUrl` changes
**Mitigation**: Audit all markdown files for hardcoded `/docs/` paths, replace with relative paths (e.g., `../module-1-ros2/index.md`)
