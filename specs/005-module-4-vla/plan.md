# Implementation Plan: Module 4 – Vision-Language-Action (VLA)

**Branch**: `005-module-4-vla` | **Date**: 2025-12-21 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-module-4-vla/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Module 4 explains how Vision-Language-Action (VLA) enables humanoid robots to understand natural language commands and execute them through integrated vision perception and physical action. This conceptual module covers four key areas: (1) VLA fundamentals and pipeline architecture, (2) voice-to-action with speech recognition (Whisper), (3) language-to-plan with LLMs for task decomposition, and (4) vision-guided action with closed-loop control, culminating in a capstone autonomous humanoid workflow demonstration.

**Technical Approach**: Content-only module (no code implementation). Research-driven writing workflow: identify 15+ authoritative sources (50%+ peer-reviewed) → draft 4 chapters (5,000-7,000 words total) → embed APA citations → validate reading level (Flesch-Kincaid grade 10-12) → integrate with Docusaurus → deploy to GitHub Pages.

## Technical Context

**Content Format**: Markdown (.md files) with Docusaurus frontmatter (YAML)
**Primary Platform**: Docusaurus 3.x static site generator
**Storage**: Git repository with GitHub Pages deployment
**Citation Management**: Manual APA 7th edition inline citations + references section per chapter
**Target Audience**: Computer science and engineering students (undergraduate level)
**Project Type**: Textbook module (documentation/content, not software)
**Word Count Target**: 5,000-7,000 words across 4 chapters (~1,250-1,750 words per chapter)
**Reading Level**: Flesch-Kincaid grade 10-12 (beginner-friendly technical writing)
**Citation Requirements**: Minimum 15 sources total, 50%+ peer-reviewed (VLA papers: RT-1/RT-2/PaLM-E, Whisper, SayCan, Code as Policies, visual servoing, ROS 2 documentation)
**Integration Dependencies**: References to Modules 1-3 (ROS 2 from Module 1, Gazebo/Unity from Module 2, Isaac Sim/Isaac ROS/Nav2 from Module 3)
**Constraints**:
- Conceptual focus (no heavy code implementation)
- Beginner-friendly explanations with diagrams described in text
- All claims must be cited with peer-reviewed or official sources
- 100% original content (zero plagiarism tolerance)
- Reproducible references (stable URLs, DOIs where available)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Research Gate (Phase 0 Entry)

- ✅ **Accuracy Gate**: Module spec requires factual claims with APA citations. Plan includes research phase to identify 15+ authoritative sources before drafting.
- ✅ **Clarity Gate**: Spec mandates Flesch-Kincaid grade 10-12 reading level. Plan includes validation task to measure and adjust reading level.
- ✅ **Reproducibility Gate**: Spec requires stable, accessible references. Research.md will validate source accessibility and capture DOIs.
- ✅ **Rigor Gate**: Spec requires 15+ sources with 50%+ peer-reviewed. Research phase explicitly targets VLA papers (RT-1, RT-2, PaLM-E), Whisper paper, LLM planning literature (SayCan, Code as Policies).
- ✅ **Originality Gate**: 100% original content requirement acknowledged. No content reuse from external sources without proper attribution.
- ✅ **Modularity Gate**: 4 independent chapters with clear learning objectives per spec. Each chapter self-contained with frontmatter, objectives, content, takeaways, references.

**Result**: ✅ All gates PASS - Proceed to Phase 0 research

### Post-Design Gate (After Phase 1)

*To be validated after research.md and content design complete*

- [ ] **Accuracy**: All 15+ sources identified, verified, and documented in research.md
- [ ] **Rigor**: At least 8 sources (50%+) are peer-reviewed or official technical publications
- [ ] **Clarity**: Content outline structured for beginner-friendly progression (VLA intro → voice → language → vision/capstone)
- [ ] **Reproducibility**: All source URLs tested for accessibility, DOIs captured where available
- [ ] **Modularity**: 4 chapter outlines with learning objectives (3-5 per chapter) defined

## Project Structure

### Documentation (this feature)

```text
specs/005-module-4-vla/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output - 15+ sources with summaries
├── data-model.md        # N/A for content module (no entities/APIs)
├── quickstart.md        # N/A for content module (no setup instructions)
├── contracts/           # N/A for content module (no API contracts)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

**NOTE**: This is a **content module**, not a software project. The "source" is markdown documentation, not code.

```text
docs/module-4-vla/
├── index.md                         # Module overview with learning objectives
├── intro-vla.md                     # Chapter 1: Introduction to Vision-Language-Action
├── voice-to-action.md               # Chapter 2: Voice-to-Action Pipeline
├── language-planning.md             # Chapter 3: Language-to-Plan with LLMs
├── capstone-autonomous-humanoid.md  # Chapter 4: Vision-Guided Action and Capstone
└── references.md                    # Consolidated APA references (optional, or embedded per chapter)

# Supporting configuration (existing)
sidebars.js                          # Docusaurus navigation config (update with Module 4 entries)
docusaurus.config.js                 # Docusaurus site config (no changes needed)
```

**Structure Decision**: Content-only structure. All deliverables are markdown files in `docs/module-4-vla/`. No source code, APIs, or databases. Docusaurus compiles markdown to static HTML for GitHub Pages deployment. Integration point: `sidebars.js` must be updated to add Module 4 navigation links (following pattern from Modules 1-3).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations** - Module design fully compliant with constitutional requirements:
- Accuracy: Research phase identifies and validates all sources
- Clarity: Beginner-friendly writing with Flesch-Kincaid validation
- Reproducibility: Stable references with DOIs
- Rigor: 15+ sources with 50%+ peer-reviewed requirement
- Originality: 100% original content with proper attribution
- Modularity: 4 independent chapters with learning objectives

## Phase 0: Research & Source Identification

**Goal**: Identify, validate, and document 15+ authoritative sources (50%+ peer-reviewed) to support Module 4 content on Vision-Language-Action.

### Research Tasks

**Task Group 1: VLA Foundation Research** (identify 3-4 sources)
- Research Vision-Language-Action models and architectures (RT-1, RT-2, PaLM-E from Google Research)
- Find foundational VLA papers explaining multimodal integration for robotics
- Validate source quality (peer-reviewed conference papers: ICRA, IROS, CoRL, NeurIPS)
- Document: Paper title, authors, year, venue, DOI/URL, key contributions relevant to Module 4

**Task Group 2: Speech Recognition Research** (identify 2-3 sources)
- Research OpenAI Whisper architecture and performance (Whisper paper, OpenAI documentation)
- Find speech recognition fundamentals for audio preprocessing and neural transcription
- Validate source quality (Whisper paper is peer-reviewed; OpenAI docs are official technical documentation)
- Document: Whisper capabilities (noise robustness, multilingual support), limitations, conceptual workflow

**Task Group 3: LLM Planning Research** (identify 3-4 sources)
- Research LLM-based robot task planning (SayCan, Code as Policies, Instruct2Act from Google/academia)
- Find task decomposition literature and few-shot prompting for robotics
- Compare classical planning (PDDL, hierarchical task networks) vs LLM-based planning
- Validate source quality (peer-reviewed papers + official technical documentation)
- Document: How LLMs map language to action primitives, handling plan failures, tradeoffs

**Task Group 4: Vision-Guided Action Research** (identify 3-4 sources)
- Research object detection and 3D pose estimation for manipulation (YOLO, Mask R-CNN, 6D pose estimation papers)
- Find visual servoing and closed-loop control literature for robotics
- Reference Isaac ROS perception from Module 3 (object detection, depth processing)
- Validate source quality (IEEE Transactions on Robotics, ICRA/IROS papers, Isaac ROS docs)
- Document: Vision perception outputs (bounding boxes, 3D poses) → action planning (grasp poses, navigation waypoints)

**Task Group 5: Integration & Deployment Research** (identify 2-3 sources)
- Research ROS 2 action servers and Nav2 navigation (official ROS 2 documentation from Module 1/3)
- Find sim-to-real transfer best practices and Isaac Sim deployment workflows (NVIDIA Isaac documentation from Module 3)
- Reference manipulation planning (MoveIt2) for humanoid task execution
- Validate source quality (official ROS 2/NVIDIA documentation)
- Document: How voice → LLM → Nav2 → Isaac ROS perception → manipulation integrate in capstone workflow

**Task Group 6: Pedagogical Research** (identify 1-2 sources)
- Research beginner-friendly robotics education approaches (textbook references, education papers)
- Find examples of conceptual robotics explanations without heavy implementation details
- Validate: Bloom's taxonomy for learning objectives, diagram-driven explanations
- Document: Best practices for teaching VLA concepts to undergraduate students

### Research Consolidation

**Output**: `specs/005-module-4-vla/research.md` with structure:

```markdown
# Research Documentation: Module 4 – Vision-Language-Action (VLA)

## Summary
[Brief overview of research findings - 2-3 paragraphs]

## Source Categories

### VLA Foundation (4 sources)
- **VLA-01**: [Citation] - [Summary of key contributions relevant to Module 4]
- **VLA-02**: [Citation] - [Summary]
- ...

### Speech Recognition (3 sources)
- **SR-01**: OpenAI Whisper paper - [Summary]
- **SR-02**: Whisper documentation - [Summary]
- ...

### LLM Planning (4 sources)
- **LLM-01**: SayCan (Google) - [Summary]
- **LLM-02**: Code as Policies - [Summary]
- ...

### Vision-Guided Action (4 sources)
- **VIS-01**: Visual servoing paper - [Summary]
- **VIS-02**: Isaac ROS perception docs - [Summary]
- ...

### Integration (3 sources)
- **INT-01**: ROS 2 Nav2 documentation - [Summary]
- **INT-02**: Isaac Sim deployment guide - [Summary]
- ...

## Source Validation

- **Total Sources**: [Count, target 15+]
- **Peer-Reviewed**: [Count, target 50%+ = 8+]
- **Official Documentation**: [Count]
- **DOI Availability**: [Count with DOIs]
- **URL Accessibility**: [Validation date, all URLs tested]

## Key Findings for Content Design

1. **VLA Pipeline**: [High-level architecture from research]
2. **Whisper Workflow**: [Conceptual audio-to-text flow]
3. **LLM Task Decomposition**: [How LLMs generate action sequences]
4. **Vision-Action Integration**: [Perception outputs → planning inputs]
5. **Capstone Integration**: [Technologies from Modules 1-3 that enable VLA workflow]

## References (APA Format)

[Full APA 7th edition citations for all sources, sorted alphabetically]
```

### Success Criteria for Phase 0

- [ ] At least 15 sources identified and validated
- [ ] At least 8 sources (50%+) are peer-reviewed conference/journal papers
- [ ] All sources documented with full APA citations
- [ ] All URLs tested for accessibility (within last 7 days)
- [ ] DOIs captured for peer-reviewed sources where available
- [ ] Key findings summarized for each source (2-3 sentences per source)
- [ ] Research findings organized by category (VLA, Speech, LLM, Vision, Integration)
- [ ] No [NEEDS CLARIFICATION] markers remaining in research.md

## Phase 1: Content Design & Structure

**Goal**: Design chapter outlines, learning objectives, and section structure based on research findings. Define citation integration points.

**Prerequisites**: `research.md` complete with 15+ validated sources

### Design Tasks

**Task 1: Chapter 1 Design (intro-vla.md)**
- Define learning objectives (3-5 objectives using Bloom's taxonomy verbs: Explain, Describe, Identify)
- Outline sections:
  - Introduction (what is VLA, why it matters)
  - Three modalities (vision, language, action) with definitions
  - VLA pipeline high-level architecture (diagram described in text)
  - Classical vs VLA-enabled robotics comparison
  - Chapter summary with key takeaways (3-5 bullets)
- Map sources from research.md to sections (inline citation plan)
- Target word count: 1,250-1,500 words

**Task 2: Chapter 2 Design (voice-to-action.md)**
- Define learning objectives (3-5 objectives focused on speech recognition)
- Outline sections:
  - Introduction (role of voice in VLA pipeline)
  - Speech recognition fundamentals (audio preprocessing, neural transcription)
  - OpenAI Whisper architecture (conceptual, not implementation)
  - Voice-to-text examples with confidence scores
  - Handling noise, accents, ambiguous commands
  - Integration with LLM planning (text → intent extraction)
  - Chapter summary with key takeaways
- Map Whisper sources (SR-01, SR-02) to sections
- Target word count: 1,250-1,500 words

**Task 3: Chapter 3 Design (language-planning.md)**
- Define learning objectives (3-5 objectives focused on LLM planning)
- Outline sections:
  - Introduction (language understanding → action planning)
  - LLM-based task decomposition (few-shot prompting)
  - Mapping natural language to ROS 2 action primitives
  - Concrete examples ("Clean the room" → action sequence)
  - Plan failure handling and replanning
  - Classical planning vs LLM-based planning tradeoffs
  - Chapter summary with key takeaways
- Map LLM sources (LLM-01, LLM-02, LLM-03, LLM-04) to sections
- Target word count: 1,500-1,750 words

**Task 4: Chapter 4 Design (capstone-autonomous-humanoid.md)**
- Define learning objectives (4-5 objectives covering vision-action integration and capstone)
- Outline sections:
  - Introduction (closing the VLA loop)
  - Object detection and 3D pose estimation (reference Isaac ROS from Module 3)
  - Vision perception outputs (bounding boxes, poses, scene graph)
  - Vision-guided action (grasp planning, visual servoing)
  - Closed-loop control with real-time perception feedback
  - Capstone autonomous humanoid workflow (voice → LLM → Nav2 → Isaac ROS → manipulation)
  - Simulation-first approach (Isaac Sim deployment from Module 3)
  - Chapter summary with complete VLA pipeline review
- Map vision sources (VIS-01, VIS-02, VIS-03, VIS-04) and integration sources (INT-01, INT-02, INT-03)
- Target word count: 1,500-1,750 words

**Task 5: Module Overview Design (index.md)**
- Write module introduction (200-300 words)
- Summarize all 4 chapters with brief descriptions
- Define module-level learning objectives (5-6 high-level objectives)
- Prerequisites (Modules 1-3 completed)
- Estimated completion time (4-6 hours reading + exercises)
- Map VLA foundation sources (VLA-01, VLA-02, VLA-03, VLA-04)

**Task 6: Citation Integration Plan**
- Create citation map: Each section → specific sources from research.md
- Plan inline citation placement (conceptual claims, performance metrics, architecture descriptions)
- Decide: Consolidated references.md OR per-chapter references sections (recommend per-chapter for Docusaurus modularity)
- Validate: All 15+ sources are cited at least once across all chapters

**Task 7: Sidebar Configuration Update**
- Update `sidebars.js` to add Module 4 navigation links
- Follow pattern from Modules 1-3:
  ```javascript
  {
    type: 'category',
    label: 'Module 4: Vision-Language-Action (VLA)',
    collapsed: false,
    items: [
      { type: 'doc', id: 'module-4-vla/index', label: 'Module Overview' },
      { type: 'doc', id: 'module-4-vla/intro-vla', label: 'Chapter 1: Introduction to VLA' },
      { type: 'doc', id: 'module-4-vla/voice-to-action', label: 'Chapter 2: Voice-to-Action Pipeline' },
      { type: 'doc', id: 'module-4-vla/language-planning', label: 'Chapter 3: Language-to-Plan with LLMs' },
      { type: 'doc', id: 'module-4-vla/capstone-autonomous-humanoid', label: 'Chapter 4: Vision-Guided Action and Capstone' },
      { type: 'doc', id: 'module-4-vla/references', label: 'References' }
    ]
  }
  ```

### Design Output

**File**: `specs/005-module-4-vla/content-design.md` (custom design artifact for content modules)

```markdown
# Content Design: Module 4 – Vision-Language-Action (VLA)

## Module Overview (index.md)
**Word Count**: 250-300 words
**Learning Objectives**: 5 module-level objectives
**Citations**: VLA-01, VLA-02, VLA-03, VLA-04

## Chapter 1: Introduction to VLA (intro-vla.md)
**Word Count**: 1,250-1,500 words
**Learning Objectives**:
1. [Objective 1 using Bloom's verb]
2. [Objective 2]
3. [Objective 3]
4. [Objective 4]

**Sections**:
1. Introduction (200 words) - Citations: VLA-01, VLA-02
2. Three Modalities (300 words) - Citations: VLA-03
3. VLA Pipeline (350 words) - Citations: VLA-01, VLA-04
4. Classical vs VLA Robotics (350 words) - Citations: VLA-02, INT-01
5. Chapter Summary (100 words)

**Key Takeaways**: [3-5 bullets]

## Chapter 2: Voice-to-Action (voice-to-action.md)
[Similar structure with sections, word counts, citations]

## Chapter 3: Language-to-Plan (language-planning.md)
[Similar structure]

## Chapter 4: Capstone (capstone-autonomous-humanoid.md)
[Similar structure]

## Citation Integration Map
| Source | Chapters | Sections |
|--------|----------|----------|
| VLA-01 | Ch1, Ch4 | Ch1-Introduction, Ch1-Pipeline, Ch4-Capstone |
| SR-01  | Ch2      | Ch2-Whisper Architecture, Ch2-Examples |
| LLM-01 | Ch3      | Ch3-Task Decomposition, Ch3-Examples |
| ...    | ...      | ... |

## Total Word Count Estimate
- Module Overview: 275 words
- Chapter 1: 1,400 words
- Chapter 2: 1,350 words
- Chapter 3: 1,600 words
- Chapter 4: 1,650 words
**Total**: 6,275 words (within 5,000-7,000 target)

## Sidebar Configuration
[Sidebar.js snippet with Module 4 navigation]
```

### Success Criteria for Phase 1

- [ ] All 4 chapter outlines complete with learning objectives (3-5 per chapter)
- [ ] Section-level design with word count targets per section
- [ ] Citation integration map created (all 15+ sources mapped to specific sections)
- [ ] Total word count estimate within 5,000-7,000 target range
- [ ] Sidebar configuration snippet prepared for sidebars.js update
- [ ] content-design.md created with complete chapter structures
- [ ] Post-Design Constitution Gate re-check PASSED

## Phase 2: Task Generation (Command: /sp.tasks)

**Note**: Phase 2 is executed by the `/sp.tasks` command, not `/sp.plan`. This section documents the expected task structure for reference.

### Expected Task Phases (Preview)

**Phase 1: Setup & Configuration** (5 tasks)
- Update sidebars.js with Module 4 navigation
- Create docs/module-4-vla/ directory
- Validate Docusaurus configuration for Module 4
- Verify research.md and content-design.md artifacts
- Initialize module index.md skeleton

**Phase 2: Research & Source Validation** (8 tasks per `/sp.plan` Phase 0)
- Research Task Group 1: VLA Foundation (3-4 sources)
- Research Task Group 2: Speech Recognition (2-3 sources)
- Research Task Group 3: LLM Planning (3-4 sources)
- Research Task Group 4: Vision-Guided Action (3-4 sources)
- Research Task Group 5: Integration (2-3 sources)
- Research Task Group 6: Pedagogical (1-2 sources)
- Consolidate research.md with APA citations
- Validate source count and peer-review percentage

**Phase 3: Content Writing - Chapter 1 (5 tasks)**
- Write Chapter 1 frontmatter and learning objectives
- Write Chapter 1 sections (Introduction, Modalities, Pipeline, Classical vs VLA)
- Embed inline citations (APA format) from research.md
- Write Chapter 1 summary and key takeaways
- Validate word count (1,250-1,500 words)

**Phase 4: Content Writing - Chapter 2** (5 tasks, similar structure)
**Phase 5: Content Writing - Chapter 3** (5 tasks)
**Phase 6: Content Writing - Chapter 4** (5 tasks)
**Phase 7: Module Overview & Integration** (4 tasks)

**Phase 8: Validation & Quality Assurance** (7 tasks)
- Validate total word count (5,000-7,000 words)
- Validate citation count (15+ sources, 50%+ peer-reviewed)
- Check reading level (Flesch-Kincaid grade 10-12)
- Verify all learning objectives use Bloom's taxonomy verbs
- Test Docusaurus build (npm run build)
- Validate internal links and references
- Run plagiarism check (manual review for originality)

**Phase 9: Deployment** (3 tasks)
- Commit all module files
- Push to branch 005-module-4-vla
- Create PR for Module 4 integration

**Total Estimated Tasks**: ~50-55 tasks across 9 phases

## Implementation Strategy

### Modular Approach

Module 4 is designed as **independently deliverable chapters**:
- Each chapter (P1-P4) is a standalone user story with acceptance criteria
- Chapters can be written in priority order (P1 → P2 → P3 → P4)
- Early chapters (P1-P2) can be reviewed/validated while later chapters (P3-P4) are in progress

### Quality Validation Checkpoints

**After Phase 0 (Research)**:
- Validate: 15+ sources identified, 50%+ peer-reviewed, all URLs accessible
- Gate: Cannot proceed to writing without research complete

**After Phase 1 (Design)**:
- Validate: All chapter outlines complete, word count estimates within target range
- Gate: Cannot proceed to writing without design approval

**After Each Chapter (Phases 3-6)**:
- Validate: Chapter word count, citation count, learning objectives, key takeaways
- Gate: Chapter must pass validation before moving to next chapter

**After Phase 7 (Integration)**:
- Validate: Module overview complete, sidebar updated, total word count, total citations
- Gate: Cannot proceed to final validation without integration complete

**After Phase 8 (Validation)**:
- Validate: All constitutional requirements met (word count, citations, reading level, originality)
- Gate: Cannot deploy without validation passing

### Risk Mitigation

**Risk 1: Insufficient peer-reviewed sources for VLA (emerging field)**
- Mitigation: Supplement with official NVIDIA Isaac/ROS 2 documentation (considered authoritative)
- Target breakdown: 8 peer-reviewed papers + 7 official technical docs = 15 sources

**Risk 2: Reading level too technical (Flesch-Kincaid > grade 12)**
- Mitigation: Use beginner-friendly analogies, define technical terms on first use, validate after each chapter
- Tools: Online Flesch-Kincaid calculators, iterative simplification

**Risk 3: Conceptual explanations without examples may be too abstract**
- Mitigation: Include concrete examples ("Clean the room" task decomposition, "Grasp blue ball" vision pipeline)
- Reference: Pseudocode and workflow diagrams described in text

**Risk 4: Integration with Modules 1-3 may have broken references**
- Mitigation: Validate cross-module references during Phase 8, test all internal links with Docusaurus build

**Risk 5: Word count may exceed 7,000 target**
- Mitigation: Chapter-level word count validation, adjust section lengths during writing, consolidate redundant explanations

## Success Metrics

### Phase 0 Success (Research)
- ✅ 15+ sources documented in research.md
- ✅ 50%+ sources are peer-reviewed (8+ papers)
- ✅ All sources have full APA citations
- ✅ All URLs tested and accessible
- ✅ Key findings summarized for each source

### Phase 1 Success (Design)
- ✅ 4 chapter outlines with learning objectives (3-5 per chapter)
- ✅ Citation integration map complete
- ✅ Word count estimate within 5,000-7,000 range
- ✅ Sidebar configuration prepared
- ✅ content-design.md artifact created

### Module Completion Success (After /sp.tasks execution)
- ✅ 5 markdown files created (index.md + 4 chapters)
- ✅ Total word count: 5,000-7,000 words
- ✅ Citation count: 15+ sources, 50%+ peer-reviewed
- ✅ Reading level: Flesch-Kincaid grade 10-12
- ✅ All learning objectives use Bloom's taxonomy verbs
- ✅ All chapters have key takeaways (3-5 bullets)
- ✅ Docusaurus build passes without errors
- ✅ Plagiarism check: 100% original content
- ✅ sidebars.js updated with Module 4 navigation
- ✅ All cross-module references validated (Modules 1-3 integration)

## Next Steps

1. **Execute Phase 0**: Run research agents to identify and validate 15+ sources
   - Command: Agent tasks will be dispatched based on Research Task Groups defined above
   - Output: `specs/005-module-4-vla/research.md`
   - Validation: Source count, peer-review percentage, URL accessibility

2. **Execute Phase 1**: Create content design based on research findings
   - Input: `research.md` with validated sources
   - Output: `specs/005-module-4-vla/content-design.md`
   - Validation: Chapter outlines, citation map, word count estimates

3. **Run /sp.tasks**: Generate task breakdown for implementation
   - Input: `plan.md` (this file), `research.md`, `content-design.md`
   - Output: `specs/005-module-4-vla/tasks.md`
   - Expected: ~50-55 tasks across 9 phases

4. **Execute /sp.implement**: Write module content following tasks.md
   - Phases 3-7: Content writing (one chapter at a time)
   - Phase 8: Validation and quality assurance
   - Phase 9: Deployment (commit, push, PR)

## References

*This section will be populated during research phase (Phase 0)*

All sources identified during research will be compiled with full APA 7th edition citations in `research.md`, then integrated as inline citations throughout module chapters.
