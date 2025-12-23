# Implementation Plan: Module 1 - The Robotic Nervous System (ROS 2)

**Branch**: `001-ros2-nervous-system` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ros2-nervous-system/spec.md`

**Note**: This plan covers Module 1 implementation. The broader 4-module textbook system plan is documented separately.

## Summary

Create Module 1 of the Physical AI & Humanoid Robotics textbook, covering ROS 2 fundamentals, Python AI agent integration, URDF robot modeling, and hands-on labs. Module will be delivered as Docusaurus-compatible Markdown with embedded code examples, diagrams, and reproducible experiments. Content must meet constitution standards: 5,000-7,000 words, minimum 15 peer-reviewed sources, APA citations, 100% reproducibility, and Flesch-Kincaid grade 10-12 readability.

## Technical Context

**Language/Version**: Markdown (CommonMark spec), Python 3.10+ (for code examples), XML (for URDF models)
**Primary Dependencies**: Docusaurus 3.x (static site generator), Node.js 18+ (for Docusaurus build), Mermaid.js (for diagrams)
**Storage**: File-based (Markdown files in `docs/module-1-ros2/` directory), Git version control
**Testing**: Manual proofreading, plagiarism detection (Turnitin or similar), readability analysis (Flesch-Kincaid tools), code example testing (Ubuntu 22.04 + ROS 2 Humble)
**Target Platform**: Web (GitHub Pages via Docusaurus), PDF export (for offline distribution)
**Project Type**: Documentation/Content (structured as modular educational material)
**Performance Goals**: Page load <2s, search response <500ms, mobile-responsive design
**Constraints**:
- Word count: 5,000-7,000 words total for module
- Reading level: Flesch-Kincaid grade 10-12
- Citation requirement: Minimum 15 sources, 50%+ peer-reviewed
- Code reproducibility: 100% on Ubuntu 22.04 + ROS 2 Humble
**Scale/Scope**:
- 4 chapters (ROS 2 Fundamentals, Python/ROS Integration, URDF Modeling, Hands-on Lab)
- 15-20 code examples
- 6-10 diagrams
- 1 comprehensive lab guide
- Module is Part 1 of 4-module curriculum

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Core Principles Compliance

| Principle | Requirement | Compliance Plan |
|-----------|-------------|-----------------|
| **I. Accuracy** | All content factually correct, verified against primary sources, cited in APA style, 50%+ peer-reviewed sources | ✅ Phase 0 research will identify authoritative sources (ROS 2 docs, peer-reviewed robotics papers). Phase 1 draft will embed APA citations. Post-draft gate includes fact verification. |
| **II. Clarity** | Flesch-Kincaid grade 10-12, technical terminology explained, concepts broken down with examples | ✅ Phase 1 draft targets grade 10-12 readability. Technical terms defined on first use. Each concept accompanied by code example or diagram. Post-draft readability analysis required. |
| **III. Reproducibility** | All experiments step-by-step, code tested and functional, external resources accessible | ✅ All code examples tested on Ubuntu 22.04 + ROS 2 Humble (pre-publication gate). Lab guide includes materials list, procedures, expected outcomes, troubleshooting. GitHub repo with working code provided. |
| **IV. Rigor** | Peer-reviewed sources prioritized, minimum 15 sources, claims evidenced, speculation labeled | ✅ Pre-draft gate requires 15+ sources identified and verified. At least 8 peer-reviewed papers (robotics middleware, humanoid robots, simulation). Official ROS 2/URDF documentation cited. |
| **V. Originality** | Zero plagiarism, proper attribution, licensed content only | ✅ All content written from scratch based on research. Paraphrased concepts cited. Post-draft plagiarism check (Turnitin or equivalent) required before publication. |
| **VI. Modularity** | Self-contained module, clear objectives/prerequisites, Spec-Kit Plus compatible, Claude Code compatible | ✅ Module includes introduction with learning objectives and prerequisites section. Can be used independently or as part of 4-module curriculum. Markdown structure compatible with automated tooling. |

### Content Standards Compliance

| Standard | Requirement | Compliance Plan |
|----------|-------------|-----------------|
| **Word Count** | 5,000-7,000 words per module | ✅ Chapter word count targets: Ch1=1,500w, Ch2=1,200w, Ch3=1,300w, Ch4=2,000w. Total ≈ 6,000 words. Post-draft word count validation. |
| **Structure** | Introduction, objectives, core content, examples, summary, references, exercises | ✅ Each chapter follows: intro → learning objectives → core content → code examples → practice exercises → summary. Module-level references section at end. |
| **Citations** | APA style, minimum 15 sources, 50%+ peer-reviewed, inline + references section | ✅ Inline citations in text (Author, Year). References section at module end in APA format. Pre-draft gate: 15+ sources verified (8+ peer-reviewed). |
| **Format** | PDF + Docusaurus-compatible, GitHub Pages deployment, RAG chatbot support | ✅ Primary: Docusaurus Markdown. Export: PDF via Docusaurus plugin. Headings structured for RAG embedding. Metadata tags for search. |
| **Practical Components** | Experiments with objectives, materials, procedures, outcomes, troubleshooting | ✅ Chapter 4 hands-on lab includes: objectives, prerequisites, materials (software), step-by-step procedures, expected outputs, common errors + fixes. |

**Constitution Check Status**: ✅ PASS - All principles and standards have defined compliance plans

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-nervous-system/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output: Source research and technical decisions
├── data-model.md        # Phase 1 output: Content structure and chapter outlines
├── quickstart.md        # Phase 1 output: Getting started guide for content creation
├── contracts/           # Phase 1 output: Chapter templates and style guides
│   ├── chapter-template.md
│   ├── code-example-template.md
│   └── lab-guide-template.md
├── checklists/          # Quality validation checklists
│   └── requirements.md  # Spec quality checklist (already created)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docs/
├── module-1-ros2/
│   ├── index.md                      # Module overview, objectives, prerequisites
│   ├── chapter-1-fundamentals.md     # ROS 2 nodes, topics, services, parameters
│   ├── chapter-2-python-integration.md  # rclpy, AI agents, debugging
│   ├── chapter-3-urdf-modeling.md    # URDF syntax, links, joints, sensors
│   ├── chapter-4-hands-on-lab.md     # Complete lab: build 6-DOF humanoid system
│   ├── references.md                 # APA-formatted bibliography
│   └── assets/
│       ├── diagrams/                 # Mermaid diagrams, architecture visuals
│       ├── code-examples/            # Python scripts, URDF files, launch files
│       └── images/                   # Screenshots, robot visualizations
├── module-2-digital-twin/            # Future: Module 2 content (not in this plan)
├── module-3-ai-robot-brain/          # Future: Module 3 content (not in this plan)
└── module-4-vla/                     # Future: Module 4 content (not in this plan)

code-examples/
└── module-1-ros2/
    ├── chapter-1/
    │   ├── publisher_node.py         # Simple publisher example
    │   ├── subscriber_node.py        # Simple subscriber example
    │   └── README.md                 # Setup and run instructions
    ├── chapter-2/
    │   ├── ai_agent_template.py      # rclpy AI agent template
    │   ├── sensor_actuator_agent.py  # Sensor-to-actuator example
    │   └── README.md
    ├── chapter-3/
    │   ├── simple_humanoid.urdf      # 6-DOF humanoid URDF
    │   ├── visualize_robot.launch.py # RViz launch file
    │   └── README.md
    └── chapter-4/
        ├── full_lab_solution/        # Complete working lab
        └── README.md

docusaurus.config.js                  # Docusaurus configuration
sidebars.js                           # Sidebar structure for all modules
package.json                          # Node.js dependencies
```

**Structure Decision**: **Documentation project** with Docusaurus static site generator. Content organized by module and chapter. Code examples separated from documentation for easy testing and distribution. This structure supports:
- Independent module development (Module 1 now, others later)
- Docusaurus sidebar auto-generation
- GitHub Pages deployment
- PDF export per module
- RAG chatbot embedding (clean heading hierarchy)

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations detected. All constitution principles and content standards can be met with planned approach.*

---

## Phase 0: Research & Source Identification

**Objective**: Identify authoritative sources, resolve technical unknowns, establish content foundation

### Research Tasks

#### R1: ROS 2 Official Documentation & Specifications
**Question**: What are the authoritative sources for ROS 2 Humble architecture, rclpy API, and best practices?
**Sources to Identify**:
- ROS 2 Humble official documentation (docs.ros.org)
- rclpy API reference
- ROS 2 Design principles and architecture papers
- ROS 2 tutorials and examples (official)

**Deliverable**: List of official ROS 2 resources with URLs, version compatibility notes

#### R2: URDF Format Specification & Best Practices
**Question**: What are the canonical sources for URDF syntax, Gazebo integration, and robot modeling patterns?
**Sources to Identify**:
- URDF XML specification (ROS wiki)
- Gazebo plugin documentation for sensors/actuators
- RViz visualization best practices
- Example humanoid URDF models (open source)

**Deliverable**: URDF specification documents, example models, plugin documentation links

#### R3: Peer-Reviewed Robotics Middleware Literature
**Question**: Which peer-reviewed papers cover ROS/ROS 2 architecture, humanoid robotics, and simulation?
**Sources to Identify** (target: 8-10 peer-reviewed papers):
- Papers on ROS 2 middleware architecture
- Humanoid robot design and control papers
- Robot simulation and digital twin research
- AI-robotics integration studies

**Search Databases**: IEEE Xplore, ACM Digital Library, arXiv (cs.RO category), Google Scholar

**Deliverable**: Annotated bibliography with 8-10 peer-reviewed sources, relevance notes

#### R4: Python AI Agent Patterns for Robotics
**Question**: What are established patterns for integrating AI decision-making with robot control systems?
**Sources to Identify**:
- Behavior trees and finite state machines in robotics
- Sensor fusion patterns
- Multi-agent robotics coordination (for future modules)
- Python robotics libraries and frameworks

**Deliverable**: Pattern catalog with code examples and references

#### R5: Educational Best Practices for Robotics Instruction
**Question**: How should robotics content be structured for maximum learning effectiveness?
**Sources to Identify**:
- Robotics curriculum design papers
- Hands-on lab design for remote/hybrid learning
- Assessment methods for robotics courses
- Bloom's taxonomy application to robotics education

**Deliverable**: Educational framework for module structure and learning progression

### Research Consolidation

**Output File**: `research.md` with structure:

```markdown
# Research Findings: Module 1 - ROS 2

## Sources Identified (Minimum 15, 50%+ Peer-Reviewed)

### Official Documentation (7 sources)
1. ROS 2 Humble Documentation. (2023). Open Robotics. https://docs.ros.org/...
2. URDF XML Specification. (2023). ROS Wiki. http://wiki.ros.org/urdf/XML
...

### Peer-Reviewed Literature (8 sources)
1. Macenski, S., et al. (2022). "Robot Operating System 2: Design, Architecture, and Uses in the Wild." Science Robotics...
2. [Additional papers on humanoid robotics, simulation, AI integration]
...

## Technical Decisions

### Decision: ROS 2 Humble as Target Version
**Rationale**: Long-term support (LTS) release, stable until May 2027, widely adopted in education
**Alternatives Considered**: ROS 2 Iron (newer but shorter support), ROS 1 Noetic (legacy)
**Impact**: All code examples target Humble API, Ubuntu 22.04 compatibility

### Decision: Gazebo Classic vs. Gazebo Sim
**Rationale**: Gazebo Classic (v11) has mature ROS 2 Humble integration, extensive community resources
**Alternatives Considered**: Gazebo Sim (Ignition) - newer but less educational material available
**Impact**: Simulation examples use Gazebo Classic, note migration path to Gazebo Sim in module

### Decision: Markdown + Docusaurus vs. LaTeX
**Rationale**: Docusaurus enables web deployment, search, versioning; Markdown more accessible for collaborative editing
**Alternatives Considered**: LaTeX (better for academic publishing), Jupyter Books (code-heavy but less flexible)
**Impact**: Content in Markdown, PDF export via Docusaurus plugin

## Content Structure Decisions

### Chapter Breakdown (Based on User Stories)
- **Chapter 1** (P1): ROS 2 Fundamentals → 1,500 words, 3-4 code examples
- **Chapter 2** (P2): Python AI Agents → 1,200 words, 3-4 code examples
- **Chapter 3** (P3): URDF Modeling → 1,300 words, 2-3 URDF examples
- **Chapter 4** (P4): Hands-On Lab → 2,000 words, 1 comprehensive lab

**Total**: ≈6,000 words (within 5,000-7,000 target)

## Code Example Requirements

All examples must:
- Run on Ubuntu 22.04 + ROS 2 Humble without errors
- Include inline comments explaining key concepts
- Have accompanying README with setup/run instructions
- Be tested before publication (pre-publication gate)
```

**Completion Criteria for Phase 0**:
- ✅ Minimum 15 sources identified and verified (8+ peer-reviewed)
- ✅ All technical decisions documented with rationale
- ✅ Content structure defined with word count targets
- ✅ research.md file created and complete

---

## Phase 1: Content Design & Structure

**Prerequisites**: research.md complete

**Objective**: Define content structure, create chapter outlines, establish style guides

### D1: Content Data Model (data-model.md)

Define the structure of educational content entities:

**Entities**:

1. **Module**
   - Attributes: title, overview, learning objectives (3-5 bullets), prerequisites, estimated time, word count target
   - Contains: 4 Chapters, 1 References section
   - Metadata: module number, version, last updated

2. **Chapter**
   - Attributes: number, title, learning objectives (2-3 bullets), word count target
   - Contains: Introduction, Core Content sections (3-5), Code Examples (2-4), Practice Exercises (2-3), Summary
   - Metadata: reading time estimate, prerequisite chapters

3. **Code Example**
   - Attributes: filename, language (Python/XML), purpose description, expected output
   - Contains: Source code, inline comments, setup instructions, troubleshooting notes
   - Relationships: Referenced by Chapter, stored in code-examples/ directory

4. **Lab Guide**
   - Attributes: title, objectives, estimated duration, difficulty level
   - Contains: Prerequisites check, Materials list, Procedures (numbered steps), Expected outcomes, Common errors + fixes, Extensions (optional challenges)
   - Relationships: Integrates concepts from all chapters in module

5. **Reference**
   - Attributes: author(s), year, title, publication/source, URL/DOI, citation key
   - Format: APA style
   - Types: Peer-reviewed paper, Official documentation, Technical specification, Book chapter

**Content Flow**:
```
Module Overview → Chapter 1 (P1) → Chapter 2 (P2) → Chapter 3 (P3) → Chapter 4 (P4/Lab) → References
```

**Validation Rules**:
- Module word count: 5,000-7,000 words (sum of all chapters)
- Minimum 15 references, 50%+ peer-reviewed
- All code examples tested and functional
- Reading level: Flesch-Kincaid grade 10-12

**Output File**: `data-model.md`

### D2: Chapter Contracts (contracts/)

Create templates and style guides for consistent content creation:

**contracts/chapter-template.md**:
```markdown
# Chapter [N]: [Title]

## Learning Objectives
By the end of this chapter, you will be able to:
- [Objective 1 - action verb + measurable outcome]
- [Objective 2]
- [Objective 3]

## Prerequisites
- [Prior knowledge required]
- [Software/tools needed]

## Introduction
[2-3 paragraphs: motivation, real-world relevance, connection to prior/future chapters]

## Core Content

### Section 1: [Concept Name]
[Explanation with examples, diagrams, analogies]

**Code Example 1.1**: [Brief description]
```python
# Code with explanatory comments
```

[Explanation of code behavior and key concepts]

### Section 2: [Next Concept]
...

## Practice Exercises
1. **Exercise [N].1**: [Description] (Difficulty: Beginner/Intermediate/Advanced)
   - Hint: [Optional guidance]
   - Solution reference: [Link to solutions repo]

## Summary
- Key takeaway 1
- Key takeaway 2
- Key takeaway 3

## Further Reading
- [Reference 1] - [Brief description of relevance]
- [Reference 2]

---
*Estimated reading time: [X] minutes | Code examples: [N] | Exercises: [M]*
```

**contracts/code-example-template.md**:
```markdown
# Code Example: [Descriptive Title]

**File**: `code-examples/module-1-ros2/chapter-[N]/[filename]`
**Purpose**: [What this example demonstrates]
**Prerequisites**: [Software/concepts needed]
**Estimated Time**: [Minutes to complete]

## Setup
1. [Step 1 - environment setup]
2. [Step 2 - dependencies]
3. [Step 3 - configuration]

## Code
```python
#!/usr/bin/env python3
# [Brief description]
# Prerequisites: [list]
# Usage: python3 [filename] [args]

[Full working code with inline comments]
```

## Expected Output
```
[What users should see when running code]
```

## Explanation
[Paragraph explaining key concepts demonstrated]

## Common Errors
| Error | Cause | Solution |
|-------|-------|----------|
| [Error message] | [Why it happens] | [How to fix] |

## Extensions
- Modify the code to [challenge 1]
- Experiment with [parameter X] to observe [effect]
```

**contracts/lab-guide-template.md**:
```markdown
# Hands-On Lab: [Lab Title]

**Module**: [Module name]
**Estimated Duration**: [Hours]
**Difficulty**: [Beginner/Intermediate/Advanced]

## Objectives
By completing this lab, you will:
- [Objective 1]
- [Objective 2]
- [Objective 3]

## Prerequisites
- [ ] Completed Chapters 1-3
- [ ] Ubuntu 22.04 with ROS 2 Humble installed
- [ ] [Additional software/knowledge]

## Materials
- Software: [List all required packages]
- Hardware: [If any, or "None - simulation only"]
- Starter Code: [Link to GitHub repo]

## Procedures

### Part 1: [Initial Setup]
1. [Step 1 with code/commands]
2. [Step 2]
   - Expected outcome: [What should happen]
   - ✅ Checkpoint: [How to verify success]

### Part 2: [Main Activity]
...

## Expected Outcomes
- [ ] [Outcome 1 - measurable success criterion]
- [ ] [Outcome 2]
- [ ] [Outcome 3]

## Troubleshooting
| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| [Problem] | [Diagnosis] | [Fix with commands/steps] |

## Extensions (Optional)
1. [Challenge 1 - harder task building on lab]
2. [Challenge 2]

## Submission (If graded)
- [ ] Screenshot/video of [working system]
- [ ] Code files: [list]
- [ ] Short report answering: [reflection questions]

---
*Lab tested on: Ubuntu 22.04, ROS 2 Humble, Python 3.10*
```

**Output Files**:
- `contracts/chapter-template.md`
- `contracts/code-example-template.md`
- `contracts/lab-guide-template.md`

### D3: Quickstart Guide (quickstart.md)

Create guide for content creators (human or AI) to follow when writing module content:

**quickstart.md**:
```markdown
# Quickstart: Creating Module 1 Content

## Overview
This guide explains how to create content for Module 1: The Robotic Nervous System (ROS 2) following the established templates and constitution standards.

## Prerequisites
- Access to research.md (sources and technical decisions)
- Familiarity with Markdown and Docusaurus
- Access to Ubuntu 22.04 + ROS 2 Humble for testing code examples

## Content Creation Workflow

### Step 1: Review Research & Data Model
1. Read `research.md` to understand technical decisions and sources
2. Review `data-model.md` for content structure requirements
3. Check constitution standards (word count, citations, readability)

### Step 2: Write Chapter Content
1. Copy `contracts/chapter-template.md` to `docs/module-1-ros2/chapter-[N]-[slug].md`
2. Fill in all sections following template structure
3. Target word count: Ch1=1,500w, Ch2=1,200w, Ch3=1,300w, Ch4=2,000w
4. Embed inline citations: (Author, Year) format
5. Explain technical terms on first use
6. Include 2-4 code examples per chapter (see Step 3)

### Step 3: Create Code Examples
1. Copy `contracts/code-example-template.md` for each example
2. Write working code with inline comments
3. Test code on Ubuntu 22.04 + ROS 2 Humble
4. Document setup, expected output, common errors
5. Save code to `code-examples/module-1-ros2/chapter-[N]/`
6. Link from chapter Markdown

### Step 4: Create Chapter 4 Lab Guide
1. Copy `contracts/lab-guide-template.md` to `docs/module-1-ros2/chapter-4-hands-on-lab.md`
2. Design lab that integrates Chapters 1-3 concepts
3. Write step-by-step procedures with checkpoints
4. Test full lab end-to-end
5. Document common errors and solutions

### Step 5: Compile References
1. Extract all cited sources from chapters
2. Format in APA style in `docs/module-1-ros2/references.md`
3. Verify minimum 15 sources, 50%+ peer-reviewed
4. Check all URLs/DOIs are accessible

### Step 6: Quality Validation
Run validation checks before submission:

#### Word Count Check
```bash
# Count words in all chapter files
wc -w docs/module-1-ros2/chapter-*.md
# Target: 5,000-7,000 total
```

#### Readability Check
Use online tool (e.g., Hemingway Editor) or:
```bash
# Install textstat (if using Python)
pip install textstat
python -c "import textstat; print(textstat.flesch_kincaid_grade(open('docs/module-1-ros2/chapter-1-fundamentals.md').read()))"
# Target: Grade 10-12
```

#### Citation Count
```bash
# Count APA citations (Author, Year) pattern
grep -o '([A-Z][a-z]*, [0-9]\{4\})' docs/module-1-ros2/*.md | wc -l
# Verify matches reference count (minimum 15)
```

#### Code Testing
```bash
# Test all Python examples
cd code-examples/module-1-ros2
find . -name "*.py" -exec python3 -m py_compile {} \;
# Then manually run each example and verify output
```

#### Plagiarism Check
- Run content through Turnitin or plagiarism detection tool
- Target: 100% original content (properly cited paraphrases acceptable)

## Docusaurus Integration

### Sidebar Configuration
Edit `sidebars.js`:
```javascript
module.exports = {
  docs: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-1-ros2/index',
        'module-1-ros2/chapter-1-fundamentals',
        'module-1-ros2/chapter-2-python-integration',
        'module-1-ros2/chapter-3-urdf-modeling',
        'module-1-ros2/chapter-4-hands-on-lab',
        'module-1-ros2/references',
      ],
    },
    // Future modules here
  ],
};
```

### Build and Preview
```bash
npm install  # First time only
npm run start  # Local preview at http://localhost:3000
npm run build  # Production build
```

## Common Pitfalls
- ❌ Missing inline citations (every factual claim needs citation)
- ❌ Code examples not tested (causes reader frustration)
- ❌ Technical jargon without explanation (violates Clarity principle)
- ❌ Reading level too advanced (target grade 10-12, not graduate level)
- ❌ Word count violations (check per-chapter and module total)

## Resources
- Constitution: `.specify/memory/constitution.md`
- Feature Spec: `specs/001-ros2-nervous-system/spec.md`
- Research: `specs/001-ros2-nervous-system/research.md`
- Data Model: `specs/001-ros2-nervous-system/data-model.md`
```

**Output File**: `quickstart.md`

### D4: Update Agent Context

Run agent context update script to add Module 1 technical context:

```bash
powershell.exe -ExecutionPolicy Bypass -File ".specify/scripts/powershell/update-agent-context.ps1" -AgentType claude
```

This will update `CLAUDE.md` with:
- ROS 2 Humble as target platform
- Docusaurus content structure
- Constitution principles for content creation
- Module 1 specific technical decisions

**Completion Criteria for Phase 1**:
- ✅ data-model.md defines all content entities and relationships
- ✅ contracts/ contains chapter, code example, and lab guide templates
- ✅ quickstart.md provides step-by-step content creation workflow
- ✅ Agent context updated with Module 1 technical details

---

## Phase 2: Task Generation

**Note**: Phase 2 is completed by `/sp.tasks` command, NOT `/sp.plan`. This plan provides input for task generation.

**Expected Task Categories**:
1. **Phase 0 Tasks**: Execute research per research.md (source identification, technical decisions)
2. **Content Creation Tasks**: Write chapters per chapter-template.md
3. **Code Development Tasks**: Create code examples per code-example-template.md
4. **Lab Design Tasks**: Build Chapter 4 lab per lab-guide-template.md
5. **Quality Validation Tasks**: Word count, readability, citations, plagiarism, code testing
6. **Docusaurus Integration Tasks**: Configure sidebars, build site, deploy to GitHub Pages

**Task Sequencing**:
- Phase 0 research must complete before content creation starts
- Chapters can be written in parallel after research
- Code examples written alongside or after their chapters
- Chapter 4 lab requires Chapters 1-3 completion
- Quality validation runs after all content complete
- Docusaurus integration after quality validation passes

---

## Re-Evaluation of Constitution Check

**Post-Design Validation**:

After completing Phase 1 design artifacts (data-model, contracts, quickstart), re-evaluate constitution compliance:

| Principle | Post-Design Status |
|-----------|-------------------|
| **Accuracy** | ✅ research.md ensures 15+ sources (8+ peer-reviewed) identified before writing. Inline citation requirement in chapter-template enforces claim verification. |
| **Clarity** | ✅ quickstart.md includes readability validation step (Flesch-Kincaid grade 10-12 check). chapter-template requires term definitions and examples. |
| **Reproducibility** | ✅ code-example-template includes testing requirement. quickstart.md mandates code testing before submission. Lab guide includes troubleshooting. |
| **Rigor** | ✅ research.md documents peer-reviewed sources. quickstart.md validates citation count (minimum 15). References section required in all modules. |
| **Originality** | ✅ quickstart.md includes plagiarism check step (Turnitin or equivalent). All content written from research, not copied. |
| **Modularity** | ✅ data-model.md defines self-contained module structure. Each module has intro, objectives, prerequisites. Docusaurus enables independent deployment. |

**Verdict**: ✅ All constitution principles remain satisfied post-design. No violations introduced.

---

## Next Steps

1. **Complete Phase 0**: Run research tasks to populate research.md
2. **Complete Phase 1**: Create data-model.md, contracts/, and quickstart.md files
3. **Run `/sp.tasks`**: Generate detailed task list from this plan
4. **Execute Tasks**: Implement Module 1 content following task sequence
5. **Validate & Deploy**: Quality checks, Docusaurus build, GitHub Pages deployment

**Ready for**: `/sp.tasks` command to generate actionable task list

---

**Plan Created**: 2025-12-17
**Plan Version**: 1.0
**Last Updated**: 2025-12-17
