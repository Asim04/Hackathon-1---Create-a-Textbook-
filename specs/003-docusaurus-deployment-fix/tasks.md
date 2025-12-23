# Tasks: Docusaurus Deployment and Navigation Fix

**Input**: Design documents from `/specs/003-docusaurus-deployment-fix/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No automated tests requested - manual validation only

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus static site configuration fix. Key paths:
- **Configuration**: `docusaurus.config.js`, `sidebars.js` at repository root
- **Static assets**: `static/img/` directory
- **Pages**: `src/pages/` directory
- **Documentation content**: `docs/` directory

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Gather prerequisites and create required directory structure

- [X] T001 Determine GitHub repository information (username and repo name) from `git remote get-url origin` - NO REMOTE CONFIGURED YET (will use placeholders)
- [X] T002 [P] Create `static/img/` directory structure
- [X] T003 [P] Create `static/img/logo.svg` with simple placeholder SVG (40x40, textbook/robot icon)
- [X] T004 [P] Create `static/img/favicon.ico` (16x16 icon file, can use placeholder initially) - Created favicon.svg instead (modern browsers support SVG favicons)

**Checkpoint**: Static assets directory created with minimum required files (favicon, logo)

---

## Phase 2: User Story 1 - Local Development Access (Priority: P1) 🎯 MVP

**Goal**: Fix routing configuration so developers can run `npm start` and access the site at root URL without 404 errors

**Independent Test**: Run `npm install && npm start`, access `http://localhost:3000/book-ai/`, verify intro page loads and all sidebar navigation works

### Configuration Updates for US1

- [X] T005 [US1] Update `docusaurus.config.js` line ~42-48: Add `routeBasePath: '/'` to docs preset configuration
- [X] T006 [US1] Update `docusaurus.config.js` line 16: Replace `url: 'https://your-org.github.io'` with actual GitHub username - ADDED TODO COMMENTS (no remote configured yet)
- [X] T007 [US1] Update `docusaurus.config.js` line 19: Update `baseUrl: '/book-ai/'` to match actual repository name if different - KEPT AS '/book-ai/' with documentation
- [X] T008 [US1] Update `docusaurus.config.js` line 23: Replace `organizationName: 'your-org'` with actual GitHub username - ADDED TODO COMMENTS (no remote configured yet)
- [X] T009 [US1] Update `docusaurus.config.js` line 24: Replace `projectName: 'book-ai'` with actual repository name - ADDED TODO COMMENTS (no remote configured yet)
- [X] T010 [US1] Update `docusaurus.config.js` line 47: Replace `editUrl: 'https://github.com/your-org/book-ai/tree/main/'` with actual GitHub URL - KEPT PLACEHOLDER with TODO
- [X] T011 [US1] Update `docusaurus.config.js` line 90: Change footer link from `to: '/docs/module-1-ros2'` to `to: '/module-1-ros2/index'`
- [X] T012 [US1] Update `docusaurus.config.js` after line 92: Add Module 2 footer link `{ label: 'Module 2: Digital Twin', to: '/module-2-digital-twin/index' }`
- [X] T013 [US1] Add inline comments in `docusaurus.config.js` explaining `routeBasePath`, `url`, `baseUrl`, and footer changes
- [X] T014 [US1] Create `src/pages/index.jsx` with redirect component from `/` to `/intro` using `@docusaurus/router`
- [X] T015 [US1] Verify `sidebars.js` uses simplified IDs (no `01-` prefixes) - already correct, document pattern if needed - VERIFIED: All IDs use simplified format

### Local Testing for US1

- [ ] T016 [US1] Run `npm install` and verify dependencies install without errors
- [ ] T017 [US1] Run `npm start` and verify dev server starts on port 3000
- [ ] T018 [US1] Test root URL access: `http://localhost:3000/book-ai/` should redirect to `/book-ai/intro`
- [ ] T019 [US1] Test direct intro access: `http://localhost:3000/book-ai/intro` should load introduction page
- [ ] T020 [US1] Test Module 1 sidebar navigation: Click all 4 chapters and verify no 404 errors
- [ ] T021 [US1] Test Module 2 sidebar navigation: Click all 4 chapters and verify no 404 errors
- [ ] T022 [US1] Verify navbar logo displays correctly from `static/img/logo.svg`
- [ ] T023 [US1] Verify favicon displays in browser tab from `static/img/favicon.ico`
- [ ] T024 [US1] Check browser console for missing asset errors (should be zero or only social card warning)

**Checkpoint**: User Story 1 complete - local development environment fully functional with zero 404 errors

---

## Phase 3: User Story 2 - GitHub Pages Production Deployment (Priority: P2)

**Goal**: Configure and test production build for GitHub Pages deployment with correct base URL paths

**Independent Test**: Run `npm run build`, deploy to GitHub Pages, access live URL, verify all navigation and assets work

### Production Build Testing for US2

- [X] T025 [US2] Run `npm run build` and verify build completes successfully in <2 minutes - COMPLETED in ~42 seconds
- [X] T026 [US2] Verify build output: Check that `build/` directory exists with expected structure - VERIFIED: build/ directory created with proper structure
- [X] T027 [US2] Verify intro page exists at correct path: `build/[baseUrl]/intro/index.html` - VERIFIED: build/intro/index.html exists (baseUrl applied at serve time)
- [ ] T028 [US2] Test build locally: Run `npx serve build -p 5000` and verify site loads - REQUIRES MANUAL TESTING
- [ ] T029 [US2] Access `http://localhost:5000/[baseUrl]/` and verify navigation works with production paths - REQUIRES MANUAL TESTING
- [ ] T030 [US2] Verify all static assets (logo, favicon) load correctly in production build - REQUIRES MANUAL TESTING

**Additional Fix Applied**: Fixed broken links in footer and Module 2 markdown files
- Updated footer links from `/module-1-ros2/index` to `/module-1-ros2/`
- Fixed Module 2 chapter links from `./01-gazebo-physics` to `./gazebo-physics` format
- All builds now pass without broken link errors

### GitHub Pages Deployment for US2

- [ ] T031 [US2] Update `docusaurus.config.js` line 76: Replace navbar GitHub link `href: 'https://github.com/your-org/book-ai'` with actual repo URL
- [ ] T032 [US2] Update `docusaurus.config.js` line 103: Replace footer GitHub link `href: 'https://github.com/your-org/book-ai'` with actual repo URL
- [ ] T033 [US2] Deploy to GitHub Pages: Run `npm run deploy` or manually copy `build/` to gh-pages branch
- [ ] T034 [US2] Verify GitHub Pages settings: Repository Settings → Pages, confirm source branch
- [ ] T035 [US2] Access deployed site at `https://[username].github.io/[repo-name]/` and verify homepage redirects to intro
- [ ] T036 [US2] Test all Module 1 navigation links on deployed site
- [ ] T037 [US2] Test all Module 2 navigation links on deployed site
- [ ] T038 [US2] Verify all images load correctly with proper base URL paths on deployed site
- [ ] T039 [US2] Check browser console on deployed site for errors (should be zero)

**Checkpoint**: User Story 2 complete - production site deployed and fully functional on GitHub Pages

---

## Phase 4: User Story 3 - Complete Navigation System (Priority: P3)

**Goal**: Validate all navigation elements (sidebar, navbar, footer, internal markdown links) work correctly

**Independent Test**: Manually test all navigation elements and verify 100% functionality

### Navigation Validation for US3

- [X] T040 [P] [US3] Create navigation test checklist: List all sidebar items (intro + Module 1 + Module 2 chapters + references) - COMPLETED: Created NAVIGATION_TEST_CHECKLIST.md with 29 items
- [X] T041 [P] [US3] Document navbar items to test: "Textbook" link, "GitHub" link - COMPLETED: Documented 4 navbar items
- [X] T042 [P] [US3] Document footer links to test: Module 1, Module 2, GitHub, external docs - COMPLETED: Documented 4 footer links
- [ ] T043 [US3] Test all sidebar items: Click each Module 1 chapter and verify page loads (4 chapters) - REQUIRES MANUAL BROWSER TESTING
- [ ] T044 [US3] Test all sidebar items: Click each Module 2 chapter and verify page loads (4 chapters) - REQUIRES MANUAL BROWSER TESTING
- [ ] T045 [US3] Test sidebar items: Click intro and references pages, verify they load - REQUIRES MANUAL BROWSER TESTING
- [ ] T046 [US3] Test navbar "Textbook" link: Verify it navigates to correct location - REQUIRES MANUAL BROWSER TESTING
- [ ] T047 [US3] Test navbar "GitHub" link: Verify it opens correct repository in new tab - REQUIRES MANUAL BROWSER TESTING
- [ ] T048 [US3] Test footer Module 1 link: Verify `to: '/module-1-ros2/'` navigates correctly - REQUIRES MANUAL BROWSER TESTING (already fixed in Phase 3)
- [ ] T049 [US3] Test footer Module 2 link: Verify `to: '/module-2-digital-twin/'` navigates correctly - REQUIRES MANUAL BROWSER TESTING (already fixed in Phase 3)
- [ ] T050 [US3] Test footer GitHub link: Verify it opens correct repository - REQUIRES MANUAL BROWSER TESTING
- [X] T051 [US3] Audit markdown files in `docs/module-1-ros2/` for hardcoded `/docs/` URLs (should use relative paths) - COMPLETED: 0 hardcoded URLs found ✓
- [X] T052 [US3] Audit markdown files in `docs/module-2-digital-twin/` for hardcoded `/docs/` URLs (should use relative paths) - COMPLETED: 0 hardcoded URLs found (1 external GitHub URL acceptable) ✓
- [ ] T053 [US3] Test internal markdown links: Click links within chapters and verify relative paths resolve correctly - REQUIRES MANUAL BROWSER TESTING
- [X] T054 [US3] Verify sidebar IDs in `sidebars.js` match filename patterns consistently (simplified format) - COMPLETED: All 13 IDs verified as simplified format ✓
- [ ] T055 [US3] Optional: Install linkinator (`npm install -g linkinator`) and run automated link checking - OPTIONAL
- [ ] T056 [US3] Optional: Run `linkinator http://localhost:3000/book-ai/ --recurse` and review output for broken links - OPTIONAL

**Automated Tasks Complete**: 6/17 tasks (T040-T042, T051-T052, T054)
**Manual Testing Required**: 8 tasks (T043-T050, T053)
**Optional Tasks**: 2 tasks (T055-T056)

**Checkpoint**: User Story 3 complete - 100% of navigation links functional and verified

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Final documentation, cleanup, and validation

- [ ] T057 [P] Document GitHub repository configuration in README.md (username, repo name, deployment URL)
- [ ] T058 [P] Create troubleshooting guide based on quickstart.md Issue 1-5 sections
- [ ] T059 [P] Add code comments in `docusaurus.config.js` explaining configuration decisions from research.md
- [ ] T060 Validate all 15 functional requirements from spec.md are met
- [ ] T061 Validate all 8 success criteria from spec.md are achieved
- [ ] T062 Run final acceptance test: Complete all scenarios from User Story 1, 2, 3
- [ ] T063 Optional: Create `static/img/docusaurus-social-card.jpg` (1200x630 social preview image)
- [ ] T064 Final verification: Check that both local dev and production have zero 404 errors and zero console errors

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
  - Requires GitHub repository information from user
- **User Story 1 (Phase 2)**: Depends on Setup (Phase 1) completion
  - Blocking story: Local development MUST work before production can be tested
- **User Story 2 (Phase 3)**: Depends on User Story 1 completion
  - Cannot test production build until local development is verified working
- **User Story 3 (Phase 4)**: Depends on User Story 2 completion
  - Navigation validation requires both local and production environments working
- **Polish (Phase 5)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Phase 1 - No dependencies on other stories ✅ MVP
- **User Story 2 (P2)**: Depends on User Story 1 - Cannot deploy to production until local works
- **User Story 3 (P3)**: Depends on User Story 2 - Navigation testing requires both environments

### Within Each User Story

**User Story 1**:
1. Configuration updates (T005-T015) can mostly be done in parallel
2. Local testing (T016-T024) must be done sequentially after configuration

**User Story 2**:
1. Production build testing (T025-T030) must complete before deployment
2. GitHub Pages deployment (T031-T039) must be done sequentially

**User Story 3**:
1. Test checklist creation (T040-T042) can be done in parallel
2. Manual testing (T043-T056) should be done systematically

### Parallel Opportunities

- Phase 1 Setup: T002, T003, T004 (all [P]) can create assets in parallel
- User Story 1: Most config changes in `docusaurus.config.js` can be made simultaneously
- User Story 3: Checklist creation (T040, T041, T042) can be done in parallel
- Polish: T057, T058, T059 (all [P]) can be done in parallel

---

## Parallel Example: Phase 1 Setup

```bash
# Launch all static asset creation tasks together:
Task: "Create static/img/ directory structure"
Task: "Create static/img/logo.svg with placeholder SVG"
Task: "Create static/img/favicon.ico placeholder"
```

## Parallel Example: User Story 3 Checklist

```bash
# Launch all checklist documentation tasks together:
Task: "Create navigation test checklist for sidebar items"
Task: "Document navbar items to test"
Task: "Document footer links to test"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (gather GitHub info, create assets)
2. Complete Phase 2: User Story 1 (fix routing, test local dev)
3. **STOP and VALIDATE**: Local development fully functional
4. This MVP enables all content development work to proceed

### Incremental Delivery

1. **Foundation**: Phase 1 (Setup) → Assets ready
2. **MVP**: Phase 2 (US1) → Local dev works → Team can create content ✅
3. **Production**: Phase 3 (US2) → GitHub Pages deployed → Students can access textbook ✅
4. **Polish**: Phase 4 (US3) → Navigation validated → Professional UX ✅
5. **Complete**: Phase 5 (Polish) → Documentation and final validation

### Sequential Execution (Recommended)

This feature requires sequential execution due to dependencies:

1. **Day 1**: Phase 1 + Phase 2 (US1) - Get local dev working first
2. **Day 2**: Phase 3 (US2) - Deploy to production once local is verified
3. **Day 3**: Phase 4 (US3) + Phase 5 - Validate navigation and polish

---

## Notes

- **GitHub Info Required**: Tasks T001, T006-T010, T031-T032 require actual GitHub username and repository name
- **Base URL**: Current config uses `/book-ai/` - update to match actual repository name if different
- **Sidebar IDs**: Already correct (simplified format) - T015, T054 are verification only
- **Manual Testing**: All acceptance scenarios require manual browser testing (no automated tests)
- **Console Errors**: Only acceptable error is missing social card (optional asset)
- Commit after each phase for clear rollback points
- Stop at each checkpoint to validate independently
- Estimated total time: 60-80 minutes for complete implementation
