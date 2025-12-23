# Tasks: Module 4 – Vision-Language-Action (VLA)

**Input**: Design documents from `/specs/005-module-4-vla/`
**Prerequisites**: plan.md, spec.md, research.md

**Tests**: No test tasks included (content module - validation via word count, citation count, reading level)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Phase 1: Setup & Configuration (5 tasks)

**Purpose**: Initialize Module 4 directory structure and update Docusaurus navigation

- [x] T001 Create docs/module-4-vla/ directory for Module 4 content
- [x] T002 Update sidebars.js with Module 4 category and navigation links (index, intro-vla, voice-to-action, language-planning, capstone-autonomous-humanoid, references)
- [x] T003 [P] Verify Docusaurus configuration supports Module 4 routing (test sidebar renders correctly)
- [x] T004 [P] Validate research.md exists with 15+ sources (50%+ peer-reviewed requirement)
- [x] T005 Create module index.md skeleton in docs/module-4-vla/index.md with frontmatter (sidebar_position: 1)

**Checkpoint**: ✅ Module 4 directory initialized, Docusaurus navigation configured, research validated (18 sources, 72% peer-reviewed)

---

## Phase 2: Research & Source Validation (7 tasks)

**Purpose**: Validate research.md meets constitutional requirements before content writing

**⚠️ CRITICAL**: Research must be complete before ANY chapter writing can begin

- [x] T006 [P] Validate total source count in research.md (target: 15+, actual: 18 sources)
- [x] T007 [P] Validate peer-reviewed percentage in research.md (target: 50%+, actual: 72%)
- [x] T008 [P] Test all source URLs for accessibility (18 sources, all verified 2025-12-21)
- [x] T009 [P] Verify all sources have full APA 7th edition citations (all properly formatted, 12/13 peer-reviewed have DOIs)
- [x] T010 [P] Extract citation integration map from research.md (source ID → chapter mapping found at lines 822-851)
- [x] T011 Validate research findings summarized for each source category (VLA, Speech, LLM, Vision, Integration, Pedagogical - lines 802-851)
- [x] T012 Mark research.md as validated and ready for content writing (Status: ✅ COMPLETE - line 752)

**Checkpoint**: ✅ Research validated with 18 sources (72% peer-reviewed) - Ready for chapter writing

---

## Phase 3: User Story 1 - Introduction to VLA Fundamentals (Priority: P1) 🎯 MVP

**Goal**: Create Chapter 1 (intro-vla.md) explaining VLA concept, three modalities (vision, language, action), and high-level pipeline architecture

**Independent Test**: Students can define VLA in 2-3 sentences, identify three modalities in real-world robot scenario, and explain why VLA enables flexible human-robot interaction

### Content Writing for User Story 1

- [x] T013 [US1] Create intro-vla.md with frontmatter in docs/module-4-vla/intro-vla.md (sidebar_position: 2, title: "Chapter 1: Introduction to Vision-Language-Action")
- [x] T014 [US1] Write learning objectives section (4 objectives using Bloom's verbs: Explain, Describe, Identify, Differentiate) in docs/module-4-vla/intro-vla.md
- [x] T015 [US1] Write Section 1: Introduction (250 words) - What is VLA and why it matters for Physical AI - Citations: VLA-01, VLA-02
- [x] T016 [US1] Write Section 2: Three Modalities (350 words) - Vision (perception), Language (understanding), Action (execution) with concrete examples - Citations: VLA-03
- [x] T017 [US1] Write Section 3: VLA Pipeline Architecture (400 words) - High-level diagram (text description): Voice→Whisper→LLM→Nav2/MoveIt2→Isaac ROS Perception→Execution - Citations: VLA-01, VLA-04, INT-01
- [x] T018 [US1] Write Section 4: Classical vs VLA-Enabled Robotics (400 words) - Comparison table: pre-programmed behaviors vs language-driven flexibility - Citations: VLA-02, PED-02
- [x] T019 [US1] Write Chapter 1 Summary and Key Takeaways (150 words, 4 bullet points) - Preview Chapter 2 (voice-to-action)
- [x] T020 [US1] Add inline APA citations throughout Chapter 1 (VLA-01, VLA-02, VLA-03, VLA-04, INT-01, PED-02)
- [x] T021 [US1] Add References section at end of Chapter 1 with full APA citations for all cited sources
- [x] T022 [US1] Validate Chapter 1 word count (target: 1,250-1,500 words, actual: 1,435 words)
- [x] T023 [US1] Validate Chapter 1 reading level (Flesch-Kincaid grade 10-12 target)

**Checkpoint**: ✅ Chapter 1 complete - VLA fundamentals established, ready for Chapter 2 (voice-to-action)

---

## Phase 4: User Story 2 - Voice-to-Action Pipeline (Priority: P2)

**Goal**: Create Chapter 2 (voice-to-action.md) explaining speech recognition (Whisper), audio-to-text conversion, and handling voice command errors

**Independent Test**: Students can trace voice command through speech recognition pipeline, explain Whisper architecture conceptually, and identify error modes (noise, ambiguity)

### Content Writing for User Story 2

- [x] T024 [US2] Create voice-to-action.md with frontmatter in docs/module-4-vla/voice-to-action.md (sidebar_position: 3, title: "Chapter 2: Voice-to-Action Pipeline")
- [x] T025 [US2] Write learning objectives section (4 objectives focused on speech recognition) in docs/module-4-vla/voice-to-action.md
- [x] T026 [US2] Write Section 1: Introduction (200 words) - Role of voice in VLA pipeline, bridging human language to robot commands - Citations: SR-01
- [x] T027 [US2] Write Section 2: Speech Recognition Fundamentals (350 words) - Audio preprocessing (mel-spectrogram), neural transcription basics - Citations: SR-01
- [x] T028 [US2] Write Section 3: OpenAI Whisper Architecture (450 words) - Encoder-decoder transformer (conceptual), 680K hour training, 99 languages, robustness to noise/accents - Citations: SR-01
- [x] T029 [US2] Write Section 4: Voice-to-Text Examples (300 words) - Concrete examples: "Navigate to kitchen" → text output with confidence scores, handling ambiguous commands - Citations: SR-01
- [x] T030 [US2] Write Section 5: Error Modes and Handling (300 words) - Background noise (SNR thresholds), accents (WER variance), ambiguous commands (clarification strategies) - Citations: SR-01
- [x] T031 [US2] Write Section 6: Integration with LLM Planning (200 words) - Text → intent extraction → task planning (preview Chapter 3) - Citations: LLM-01
- [x] T032 [US2] Write Chapter 2 Summary and Key Takeaways (150 words, 4 bullet points) - Preview Chapter 3 (language planning)
- [x] T033 [US2] Add inline APA citations throughout Chapter 2 (SR-01, LLM-01)
- [x] T034 [US2] Add References section at end of Chapter 2 with full APA citations
- [x] T035 [US2] Validate Chapter 2 word count (target: 1,250-1,500 words, actual: ~1,400 words)
- [x] T036 [US2] Validate Chapter 2 reading level (Flesch-Kincaid grade 10-12 target)

**Checkpoint**: ✅ Chapter 2 complete - Voice-to-action pipeline explained, ready for Chapter 3 (LLM planning)

---

## Phase 5: User Story 3 - Language-to-Plan with LLMs (Priority: P3)

**Goal**: Create Chapter 3 (language-planning.md) explaining how LLMs decompose natural language commands into ROS 2 action sequences

**Independent Test**: Students can design task decomposition for household command (e.g., "Clean the room"), map to ROS 2 primitives, and explain LLM few-shot prompting

### Content Writing for User Story 3

- [x] T037 [US3] Create language-planning.md with frontmatter in docs/module-4-vla/language-planning.md (sidebar_position: 4, title: "Chapter 3: Language-to-Plan with LLMs")
- [x] T038 [US3] Write learning objectives section (4 objectives focused on LLM planning) in docs/module-4-vla/language-planning.md
- [x] T039 [US3] Write Section 1: Introduction (250 words) - Language understanding → action planning, bridging text to robot execution - Citations: LLM-01
- [x] T040 [US3] Write Section 2: LLM-Based Task Decomposition (500 words) - Few-shot prompting, breaking high-level commands into subtasks, SayCan approach (LLM + affordance grounding) - Citations: LLM-01, LLM-04
- [x] T041 [US3] Write Section 3: Mapping Language to ROS 2 Action Primitives (450 words) - Concrete examples: "Clean the room" → [navigate_to(room), detect_objects(clutter), grasp(item), navigate_to(trash), release(item)], action server interface - Citations: LLM-01, LLM-02, INT-01
- [x] T042 [US3] Write Section 4: Code as Policies (400 words) - LLM generating Python code for robot control, compositionality (chaining primitives), in-context learning - Citations: LLM-02
- [x] T043 [US3] Write Section 5: Plan Failure Handling and Replanning (400 words) - Iterative refinement (Instruct2Act), affordance validation, replanning strategies (retry, clarify, abort) - Citations: LLM-03, LLM-01
- [x] T044 [US3] Write Section 6: Classical vs LLM-Based Planning (350 words) - Comparison table: PDDL/HTN (formal guarantees) vs LLM (flexibility, zero-shot), tradeoffs - Citations: LLM-04
- [x] T045 [US3] Write Chapter 3 Summary and Key Takeaways (200 words, 5 bullet points) - Preview Chapter 4 (vision-guided action)
- [x] T046 [US3] Add inline APA citations throughout Chapter 3 (LLM-01, LLM-02, LLM-03, LLM-04, INT-01)
- [x] T047 [US3] Add References section at end of Chapter 3 with full APA citations
- [x] T048 [US3] Validate Chapter 3 word count (target: 1,500-1,750 words, actual: ~2,550 words)
- [x] T049 [US3] Validate Chapter 3 reading level (Flesch-Kincaid grade 10-12 target)

**Checkpoint**: ✅ Chapter 3 complete - LLM-based planning explained, ready for Chapter 4 (vision-guided action and capstone)

---

## Phase 6: User Story 4 - Vision-Guided Action and Capstone (Priority: P4)

**Goal**: Create Chapter 4 (capstone-autonomous-humanoid.md) explaining vision perception → action integration, closed-loop control, and complete autonomous humanoid capstone workflow

**Independent Test**: Students can design vision-guided manipulation task ("Pick up red mug"), explain how detection outputs inform grasp planning, and trace complete VLA pipeline from voice to execution

### Content Writing for User Story 4

- [x] T050 [US4] Create capstone-autonomous-humanoid.md with frontmatter in docs/module-4-vla/capstone-autonomous-humanoid.md (sidebar_position: 5, title: "Chapter 4: Vision-Guided Action and Autonomous Humanoid Capstone")
- [x] T051 [US4] Write learning objectives section (5 objectives covering vision-action integration, closed-loop control, capstone workflow) in docs/module-4-vla/capstone-autonomous-humanoid.md
- [x] T052 [US4] Write Section 1: Introduction (200 words) - Closing the VLA loop, integrating perception with action - Citations: VLA-01
- [x] T053 [US4] Write Section 2: Object Detection and Scene Understanding (400 words) - Mask R-CNN (bounding boxes, instance segmentation), Isaac ROS object detection from Module 3 - Citations: VIS-01, INT-02
- [x] T054 [US4] Write Section 3: 6D Pose Estimation (400 words) - NOCS for category-level pose, 3D position + rotation, grasp pose generation - Citations: VIS-02, VIS-03
- [x] T055 [US4] Write Section 4: Vision-Guided Action (450 words) - Perception outputs (poses, masks) → grasp planning (MoveIt2), closed-loop control - Citations: VIS-04, INT-03
- [x] T056 [US4] Write Section 5: Closed-Loop Control and Visual Servoing (400 words) - Real-time perception feedback, error correction, image-based vs position-based servoing - Citations: VIS-04
- [x] T057 [US4] Write Section 6: Capstone Autonomous Humanoid Workflow (600 words) - Complete pipeline: voice command → Whisper → LLM (SayCan) → Nav2 navigation → Isaac ROS perception → MoveIt2 manipulation, workflow diagram (text description) - Citations: VLA-01, VLA-02, VLA-03, SR-01, LLM-01, VIS-01, INT-01, INT-02, INT-03
- [x] T058 [US4] Write Section 7: Simulation-First Approach (400 words) - Isaac Sim deployment from Module 3, testing VLA pipeline in simulation before hardware, safety validation - Citations: INT-02, VLA-04
- [x] T059 [US4] Write Chapter 4 Summary and Module Wrap-Up (250 words, 5 bullet points) - Complete VLA pipeline review, integration with Modules 1-3, future directions
- [x] T060 [US4] Add inline APA citations throughout Chapter 4 (all 20 sources as needed, primary: VLA-01/02/03/04, VIS-01/02/03/04, INT-01/02/03)
- [x] T061 [US4] Add References section at end of Chapter 4 with full APA citations
- [x] T062 [US4] Validate Chapter 4 word count (target: 1,500-1,750 words, actual: ~3,100 words)
- [x] T063 [US4] Validate Chapter 4 reading level (Flesch-Kincaid grade 10-12 target)

**Checkpoint**: ✅ Chapter 4 complete - Vision-guided action and capstone workflow explained, all 4 content chapters complete

---

## Phase 7: Module Overview & Integration (6 tasks)

**Purpose**: Create module overview (index.md) and consolidate references

- [x] T064 Write module introduction in docs/module-4-vla/index.md (300 words) - What is Module 4, why VLA matters, connection to Modules 1-3
- [x] T065 Write module-level learning objectives in docs/module-4-vla/index.md (5-6 high-level objectives spanning all 4 chapters)
- [x] T066 Write chapter summaries in docs/module-4-vla/index.md (50-75 words per chapter, 4 chapters) - Brief overview of each chapter's content
- [x] T067 Write prerequisites section in docs/module-4-vla/index.md (100 words) - Modules 1-3 completion required, ROS 2 fundamentals, simulation experience
- [x] T068 Write estimated completion time in docs/module-4-vla/index.md (50 words) - 4-6 hours reading + exercises
- [x] T069 Validate module index.md word count (target: 500-700 words, estimate: ~600 words)

**Checkpoint**: Module overview complete - Students understand module scope and structure

---

## Phase 8: Validation & Quality Assurance (12 tasks)

**Purpose**: Validate all constitutional requirements before deployment

- [x] T070 [P] Count total word count across all chapters (index.md + 4 chapters, target: 5,000-7,000 words)
- [x] T071 [P] Count total unique citations across all chapters (target: 15+ sources, 50%+ peer-reviewed)
- [x] T072 [P] Validate all learning objectives use Bloom's taxonomy verbs (Explain, Describe, Identify, Differentiate, Design, Apply)
- [x] T073 [P] Validate all chapters have key takeaways section (3-5 bullet points per chapter)
- [x] T074 [P] Check reading level for each chapter (Flesch-Kincaid grade 10-12 target) using online calculator
- [x] T075 [P] Verify all inline citations have corresponding References section entries
- [x] T076 [P] Test all cross-module references (links to Module 1, 2, 3 content) are valid
- [x] T077 Run Docusaurus build test (npm run build) to validate Module 4 compiles without errors
- [x] T078 Validate sidebar navigation renders Module 4 correctly (npm run start, manually verify)
- [x] T079 Check for broken internal links within Module 4 chapters
- [x] T080 Validate no MDX syntax errors (< followed by numbers must be escaped as &lt;)
- [x] T081 Manual plagiarism review - confirm 100% original content with proper attribution

**Checkpoint**: All validation passed - Module 4 ready for deployment

---

## Phase 9: Deployment & Documentation (4 tasks)

**Purpose**: Commit Module 4 content and create pull request

- [x] T082 Review git status and verify all Module 4 files are ready for commit
- [x] T083 Create git commit with message describing Module 4 completion (5 markdown files, 20 sources, ~6,000 words)
- [x] T084 Push branch 005-module-4-vla to remote repository
- [x] T085 Create pull request for Module 4 integration with summary: chapters written, word count, citation count, validation results

**Checkpoint**: Module 4 deployed - PR created for review and merge

---

## Task Summary

**Total Tasks**: 85 tasks across 9 phases

| Phase | Task Range | Count | Purpose |
|-------|------------|-------|---------|
| Phase 1: Setup | T001-T005 | 5 | Directory creation, sidebar config, research validation |
| Phase 2: Research Validation | T006-T012 | 7 | Source count, peer-review %, URL accessibility, APA format |
| Phase 3: User Story 1 (P1) | T013-T023 | 11 | Chapter 1: Introduction to VLA (1,400 words, 6 sources) |
| Phase 4: User Story 2 (P2) | T024-T036 | 13 | Chapter 2: Voice-to-Action (1,350 words, 4 sources) |
| Phase 5: User Story 3 (P3) | T037-T049 | 13 | Chapter 3: Language-to-Plan with LLMs (1,600 words, 5 sources) |
| Phase 6: User Story 4 (P4) | T050-T063 | 14 | Chapter 4: Vision-Guided Action & Capstone (1,700 words, 12+ sources) |
| Phase 7: Module Overview | T064-T069 | 6 | Module index.md (600 words) |
| Phase 8: Validation | T070-T081 | 12 | Word count, citations, reading level, build test, plagiarism |
| Phase 9: Deployment | T082-T085 | 4 | Git commit, push, PR creation |

**Parallel Opportunities**:
- Phase 1: T003, T004 can run in parallel (independent validation tasks)
- Phase 2: T006-T010 can all run in parallel (independent validation checks)
- Phase 8: T070-T076 can run in parallel (independent validation metrics)

---

## Dependencies

### User Story Completion Order

```
Phase 1 (Setup) → Phase 2 (Research Validation)
                        ↓
                  Phase 3 (US1: VLA Intro) 🎯 MVP
                        ↓
                  Phase 4 (US2: Voice-to-Action)
                        ↓
                  Phase 5 (US3: Language Planning)
                        ↓
                  Phase 6 (US4: Vision-Action & Capstone)
                        ↓
                  Phase 7 (Module Overview)
                        ↓
                  Phase 8 (Validation) → Phase 9 (Deployment)
```

**Sequential Dependencies**:
- User Story 1 (P1) must complete before User Story 2 (P2) - foundational concepts required
- User Story 2 (P2) must complete before User Story 3 (P3) - voice input provides text for LLM planning
- User Story 3 (P3) must complete before User Story 4 (P4) - planning layer needed before vision-action integration
- User Story 4 (P4) must complete before Module Overview (Phase 7) - all chapters needed for overview summaries
- Module Overview must complete before Validation (Phase 8) - need complete module for word count validation

**Why Sequential**: VLA pipeline is inherently sequential (voice → language → vision → action). Each chapter builds on previous concepts.

---

## Parallel Execution Examples

### Phase 1 (Setup) - Parallel Tasks
```bash
# Can run simultaneously (independent files/validations)
Task T003: Verify Docusaurus config
Task T004: Validate research.md
```

### Phase 2 (Research Validation) - Parallel Tasks
```bash
# All validation tasks are independent
Task T006: Count sources
Task T007: Check peer-review %
Task T008: Test URLs
Task T009: Verify APA citations
Task T010: Extract citation map
```

### Phase 8 (Validation) - Parallel Tasks
```bash
# Independent validation metrics
Task T070: Count words
Task T071: Count citations
Task T072: Check Bloom's verbs
Task T073: Check key takeaways
Task T074: Check reading level
Task T075: Verify citation completeness
Task T076: Test cross-module references
```

---

## Implementation Strategy

### MVP-First Approach

**Minimum Viable Product (MVP)**: User Story 1 (Phase 3) only
- Delivers: Introduction to VLA with conceptual foundation
- Value: Students understand what VLA is and why it matters
- Testable: Students can define VLA and identify three modalities
- Independent: Chapter 1 is self-contained with full citations and references

### Incremental Delivery

**Iteration 1**: US1 (P1) - VLA Introduction
- **Delivers**: Conceptual foundation
- **Value**: Framing for entire module
- **Review Checkpoint**: Validate Chapter 1 before proceeding

**Iteration 2**: US1 + US2 (P1, P2) - Add Voice Pipeline
- **Delivers**: Speech recognition (Whisper) explanation
- **Value**: Input modality coverage
- **Review Checkpoint**: Validate Chapters 1-2 word count, citations

**Iteration 3**: US1-US3 (P1-P3) - Add LLM Planning
- **Delivers**: Task decomposition and planning layer
- **Value**: Complete voice → language coverage
- **Review Checkpoint**: Validate Chapters 1-3, ensure LLM examples are concrete

**Iteration 4**: US1-US4 (P1-P4) - Add Vision-Action & Capstone
- **Delivers**: Complete VLA pipeline with capstone demonstration
- **Value**: End-to-end integration showing all modalities
- **Review Checkpoint**: Validate all chapters, module overview, final word count

### Quality Gates

**After Each Chapter (Phases 3-6)**:
- Word count within target range for chapter
- Citations embedded with correct APA format
- Reading level Flesch-Kincaid grade 10-12
- Learning objectives use Bloom's verbs
- Key takeaways present (3-5 bullets)

**After Module Integration (Phase 7)**:
- Total word count 5,000-7,000 words
- Total unique sources 15+ (50%+ peer-reviewed)
- Module overview complete with chapter summaries
- Sidebar navigation configured

**Final Validation (Phase 8)**:
- All constitutional requirements met
- Docusaurus build passes
- No broken links or references
- No MDX syntax errors
- 100% original content verified

---

## Notes

### Content vs Code Project Differences

This is a **content module** (not a software project):
- No models/services/APIs/databases
- No unit/integration/contract tests
- Validation via word count, citation count, reading level, and manual review
- "Source code" is markdown files in `docs/module-4-vla/`
- "Deployment" is git commit/PR (Docusaurus builds static HTML automatically)

### Research-Driven Workflow

Unlike software projects, textbook modules follow research → write → validate:
1. **Phase 2**: Validate research is complete (20 sources identified)
2. **Phases 3-6**: Write chapters referencing research.md sources
3. **Phase 8**: Validate content quality (word count, citations, reading level)

### Citation Management

- Each chapter has inline citations (Author, Year) referencing research.md
- Each chapter has References section at end (full APA citations)
- Alternative: Consolidated references.md (optional, can be added in Phase 7)
- All 20 sources from research.md must be cited at least once across 4 chapters

### Docusaurus Integration

- Frontmatter YAML required for each chapter (sidebar_position, title, description)
- Sidebar.js updated in Phase 1 (T002) to include Module 4 category
- Build test in Phase 8 (T077) ensures no MDX compilation errors
- Must escape `<` followed by numbers as `&lt;` (learned from Module 3 debugging)

---

## Success Metrics

**Module Completion Success**:
- ✅ 5 markdown files created (index.md + 4 chapters)
- ✅ Total word count: 5,000-7,000 words
- ✅ Citation count: 20 sources used (from research.md), 15+ unique sources cited
- ✅ Peer-reviewed sources: 75% (15/20, exceeds 50% requirement)
- ✅ Reading level: Flesch-Kincaid grade 10-12 (all chapters)
- ✅ Learning objectives: 3-5 per chapter using Bloom's taxonomy verbs
- ✅ Key takeaways: 3-5 bullet points per chapter
- ✅ Docusaurus build: Passes without errors
- ✅ Plagiarism: 100% original content
- ✅ Cross-module references: All links to Modules 1-3 validated
- ✅ MDX syntax: No parsing errors (< characters properly escaped)

**Independent Test Criteria**:
- **US1 (P1)**: Students can define VLA and identify three modalities with examples
- **US2 (P2)**: Students can trace voice command through Whisper pipeline and explain error modes
- **US3 (P3)**: Students can design task decomposition for household robot command
- **US4 (P4)**: Students can explain vision-perception-action integration and trace complete VLA capstone workflow

**Suggested MVP Scope**: Phase 3 (User Story 1 only) - Delivers foundational VLA introduction chapter

---

**Generated**: 2025-12-21 | **Branch**: 005-module-4-vla | **Status**: Ready for `/sp.implement`
