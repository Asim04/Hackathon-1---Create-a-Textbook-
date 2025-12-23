# Navigation Test Checklist - Docusaurus Deployment Fix

**Feature**: 003-docusaurus-deployment-fix
**Created**: 2025-12-20
**Purpose**: Comprehensive checklist for testing all navigation elements (sidebar, navbar, footer, internal markdown links)

---

## Sidebar Navigation Items

### Introduction
- [ ] **Intro** - Click "Introduction" in sidebar → Should navigate to `/intro`

### Module 1: The Robotic Nervous System (ROS 2)
- [ ] **Module 1 Overview** - Click "Module Overview" → Should navigate to `/module-1-ros2/`
- [ ] **Chapter 1: ROS 2 Fundamentals** - Click chapter 1 → Should navigate to `/module-1-ros2/ros2-fundamentals`
- [ ] **Chapter 2: Python Agents & ROS Integration** - Click chapter 2 → Should navigate to `/module-1-ros2/python-agents-ros`
- [ ] **Chapter 3: URDF for Humanoids** - Click chapter 3 → Should navigate to `/module-1-ros2/urdf-humanoids`
- [ ] **Chapter 4: Lab - Building a ROS 2 Robot** - Click chapter 4 → Should navigate to `/module-1-ros2/lab-building-ros2-robot`
- [ ] **Module 1 References** - Click "References" → Should navigate to `/module-1-ros2/references`

### Module 2: The Digital Twin (Gazebo & Unity)
- [ ] **Module 2 Overview** - Click "Module Overview" → Should navigate to `/module-2-digital-twin/`
- [ ] **Chapter 1: Gazebo Physics Simulation** - Click chapter 1 → Should navigate to `/module-2-digital-twin/gazebo-physics`
- [ ] **Chapter 2: Unity for High-Fidelity Simulation** - Click chapter 2 → Should navigate to `/module-2-digital-twin/unity-visualization`
- [ ] **Chapter 3: Simulating Sensors** - Click chapter 3 → Should navigate to `/module-2-digital-twin/sensor-simulation`
- [ ] **Chapter 4: Lab - Building a Digital Twin** - Click chapter 4 → Should navigate to `/module-2-digital-twin/lab-digital-twin`
- [ ] **Module 2 References** - Click "References" → Should navigate to `/module-2-digital-twin/references`

**Total Sidebar Items**: 13 (1 intro + 6 Module 1 + 6 Module 2)

---

## Navbar Navigation Items

- [ ] **Site Title** ("Physical AI & Humanoid Robotics") - Click title → Should navigate to homepage (/)
- [ ] **Logo** (left side) - Click logo → Should navigate to homepage (/)
- [ ] **"Textbook" Link** (left nav) - Click → Should open sidebar or navigate to docs
- [ ] **"GitHub" Link** (right nav) - Click → Should open https://github.com/your-org/book-ai in new tab

**Total Navbar Items**: 4

---

## Footer Navigation Links

### Modules Section
- [ ] **Module 1: ROS 2** - Click → Should navigate to `/module-1-ros2/`
- [ ] **Module 2: Digital Twin** - Click → Should navigate to `/module-2-digital-twin/`

### Resources Section
- [ ] **ROS 2 Documentation** - Click → Should open https://docs.ros.org/en/humble/ in new tab
- [ ] **GitHub** - Click → Should open https://github.com/your-org/book-ai in new tab

**Total Footer Items**: 4

---

## Internal Markdown Links

### Module 2 Index Page (`docs/module-2-digital-twin/index.md`)
- [ ] **Chapter 1 link** (line 187) - `[Chapter 1: Gazebo Physics Simulation](./gazebo-physics)` → Should navigate correctly
- [ ] **Chapter 2 link** (line 188) - `[Chapter 2: Unity for High-Fidelity Simulation](./unity-visualization)` → Should navigate correctly
- [ ] **Chapter 3 link** (line 189) - `[Chapter 3: Simulating Sensors](./sensor-simulation)` → Should navigate correctly
- [ ] **Chapter 4 link** (line 190) - `[Chapter 4: Lab - Building a Digital Twin](./lab-digital-twin)` → Should navigate correctly
- [ ] **"Next" link** (line 246) - `[Chapter 1: Gazebo Physics Simulation](./gazebo-physics)` → Should navigate correctly

### Module 2 Chapter "Next" Links
- [ ] **01-gazebo-physics.md** (line 253) - `[Chapter 2: Unity for High-Fidelity Simulation](./unity-visualization)` → Should navigate correctly
- [ ] **02-unity-visualization.md** (line 307) - `[Chapter 3: Simulating Sensors](./sensor-simulation)` → Should navigate correctly
- [ ] **03-sensor-simulation.md** (line 370) - `[Chapter 4: Lab - Building a Digital Twin](./lab-digital-twin)` → Should navigate correctly

**Total Internal Links Checked**: 8

---

## Hardcoded `/docs/` URL Audit

### Module 1 Files
- [ ] **module-1-ros2/index.md** - No hardcoded `/docs/` URLs found ✓
- [ ] **module-1-ros2/01-ros2-fundamentals.md** - Check for `/docs/` patterns
- [ ] **module-1-ros2/02-python-agents-ros.md** - Check for `/docs/` patterns
- [ ] **module-1-ros2/03-urdf-humanoids.md** - Check for `/docs/` patterns
- [ ] **module-1-ros2/04-lab-building-ros2-robot.md** - Check for `/docs/` patterns
- [ ] **module-1-ros2/references.md** - Check for `/docs/` patterns

### Module 2 Files
- [ ] **module-2-digital-twin/index.md** - Already fixed ✓ (lines 187-190, 246 use relative paths)
- [ ] **module-2-digital-twin/01-gazebo-physics.md** - Already fixed ✓ (line 253 uses relative path)
- [ ] **module-2-digital-twin/02-unity-visualization.md** - Already fixed ✓ (line 307 uses relative path)
- [ ] **module-2-digital-twin/03-sensor-simulation.md** - Already fixed ✓ (line 370 uses relative path)
- [ ] **module-2-digital-twin/04-lab-digital-twin.md** - Check for `/docs/` patterns
- [ ] **module-2-digital-twin/references.md** - Check for `/docs/` patterns

**Status**: Module 2 links already fixed to use relative paths without `/docs/` prefix

---

## Sidebar ID Consistency Check

### Current Pattern (from sidebars.js)
Sidebar IDs use **simplified format** (no numeric prefixes):
- `intro`
- `module-1-ros2/index`
- `module-1-ros2/ros2-fundamentals` (file: `01-ros2-fundamentals.md`)
- `module-1-ros2/python-agents-ros` (file: `02-python-agents-ros.md`)
- `module-1-ros2/urdf-humanoids` (file: `03-urdf-humanoids.md`)
- `module-1-ros2/lab-building-ros2-robot` (file: `04-lab-building-ros2-robot.md`)
- `module-1-ros2/references`
- `module-2-digital-twin/index`
- `module-2-digital-twin/gazebo-physics` (file: `01-gazebo-physics.md`)
- `module-2-digital-twin/unity-visualization` (file: `02-unity-visualization.md`)
- `module-2-digital-twin/sensor-simulation` (file: `03-sensor-simulation.md`)
- `module-2-digital-twin/lab-digital-twin` (file: `04-lab-digital-twin.md`)
- `module-2-digital-twin/references`

**Consistency**: ✅ VERIFIED - All sidebar IDs use simplified format without numeric prefixes, matching Docusaurus best practices

---

## Testing URLs

### Local Development
- **Base URL**: `http://localhost:3000/book-ai/`
- **Dev Server**: Start with `npm start`
- **Test**: Access base URL → should redirect to `/book-ai/intro`

### Production Build (Local)
- **Base URL**: `http://localhost:5000/`
- **Serve Command**: `npx serve build -p 5000`
- **Test**: Access `/` → should display site with baseUrl applied

### GitHub Pages (Production)
- **Base URL**: `https://[username].github.io/[repo-name]/`
- **Deploy**: Run `npm run deploy` or push to gh-pages branch
- **Test**: Access base URL → should redirect to intro with all navigation working

---

## Expected Results

### Success Criteria
- ✅ 0/13 sidebar navigation items result in 404 errors (100% success rate)
- ✅ 0/4 navbar items result in 404 errors (100% success rate)
- ✅ 0/4 footer links result in 404 errors (100% success rate)
- ✅ 0/8 internal markdown links broken (100% success rate)
- ✅ All sidebar IDs match filename patterns consistently
- ✅ No hardcoded `/docs/` URLs in markdown files
- ✅ Browser console shows zero navigation-related errors

### Performance Targets
- Page load time: <1 second per page
- Navigation response: Instant (client-side routing)
- No loading spinners or delays

---

## Common Issues and Fixes

### Issue 1: 404 Error on Root URL
**Symptom**: Accessing `http://localhost:3000/book-ai/` shows 404
**Fix**: Verify `routeBasePath: '/'` set in docusaurus.config.js and `src/pages/index.jsx` exists with redirect

### Issue 2: Footer Links 404
**Symptom**: Clicking Module 1 or Module 2 footer links results in 404
**Fix**: Ensure footer links use trailing slash format (`/module-1-ros2/` not `/module-1-ros2/index`)

### Issue 3: Internal Chapter Links 404
**Symptom**: Clicking chapter links in Module 2 index shows 404
**Fix**: Use simplified format matching sidebar IDs (`./gazebo-physics` not `./01-gazebo-physics`)

### Issue 4: Sidebar Items Don't Match Files
**Symptom**: Clicking sidebar items results in "Page not found"
**Fix**: Verify sidebar IDs match markdown file slugs (without numeric prefixes)

---

**Checklist Version**: 1.0.0
**Last Updated**: 2025-12-20
**Total Items to Test**: 29 (13 sidebar + 4 navbar + 4 footer + 8 internal links)
