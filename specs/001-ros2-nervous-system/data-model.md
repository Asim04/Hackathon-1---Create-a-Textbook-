# Content Data Model: Module 1 - The Robotic Nervous System (ROS 2)

**Date**: 2025-12-17
**Feature**: 001-ros2-nervous-system
**Purpose**: Define structure of educational content entities and their relationships

---

## Entity Definitions

### 1. Module

**Description**: Top-level organizational unit representing a major topic area in the textbook

**Attributes**:
- `module_number` (integer): Sequential identifier (1, 2, 3, 4)
- `title` (string): Full module title (e.g., "The Robotic Nervous System (ROS 2)")
- `slug` (string): URL-friendly identifier (e.g., "module-1-ros2")
- `overview` (text): 2-3 paragraph description of module scope and goals
- `learning_objectives` (list[string]): 3-5 measurable learning outcomes
- `prerequisites` (list[string]): Required prior knowledge or skills
- `estimated_time_hours` (integer): Expected completion time (10-15 hours)
- `word_count_target` (range): Target word count (5,000-7,000)
- `version` (string): Semantic version (e.g., "1.0.0")
- `last_updated` (date): ISO 8601 format (YYYY-MM-DD)

**Contains**:
- 4 `Chapter` entities
- 1 `References` section
- 1 module `index.md` file

**Relationships**:
- Part of `Textbook` (parent: Physical AI & Humanoid Robotics)
- References `Module` (related modules for cross-references)

**File Location**: `docs/module-{number}-{slug}/index.md`

---

### 2. Chapter

**Description**: Major subdivision of a module covering a specific learning objective

**Attributes**:
- `chapter_number` (integer): Sequential within module (1, 2, 3, 4)
- `title` (string): Chapter title (e.g., "ROS 2 Fundamentals")
- `slug` (string): URL-friendly identifier (e.g., "01-ros2-fundamentals")
- `module_slug` (string): Parent module identifier
- `learning_objectives` (list[string]): 2-3 chapter-specific outcomes
- `prerequisites` (list[string]): Prior chapters or external knowledge
- `word_count_target` (integer): Target word count for this chapter
- `estimated_reading_time_minutes` (integer): Calculated from word count
- `sidebar_position` (integer): Order in navigation sidebar

**Contains**:
- `Introduction` (text, 2-3 paragraphs)
- `Core Content` (3-5 `Section` entities)
- 2-4 `Code Example` entities
- 2-3 `Practice Exercise` entities
- `Summary` (3-5 key takeaways)
- `Further Reading` (2-3 references)

**Relationships**:
- Belongs to `Module` (parent)
- References `Reference` entities (citations)
- Contains `Code Example` entities
- Depends on `Chapter` (prerequisite chapters)

**File Location**: `docs/module-{module_slug}/{chapter_slug}.md`

**Validation Rules**:
- Word count within ±15% of target
- Flesch-Kincaid grade level 10-12
- Minimum 3 inline citations (APA format)
- At least 2 code examples

---

### 3. Section

**Description**: Subdivision within a chapter covering a specific concept

**Attributes**:
- `section_number` (string): Hierarchical identifier (e.g., "1.1", "2.3")
- `title` (string): Section heading
- `content` (markdown): Explanation text with examples, analogies, diagrams
- `includes_diagram` (boolean): Whether section has visual diagram
- `diagram_description` (text): Mermaid.js or textual description if included

**Contains**:
- Explanation text (paragraphs)
- Optional inline `Code Example` or code snippet
- Optional diagram description (Mermaid.js compatible)
- Inline citations (APA format)

**Relationships**:
- Belongs to `Chapter` (parent)
- May reference `Code Example` (demonstrates concept in code)

**Markdown Structure**:
```markdown
### Section Title

[2-4 paragraphs explaining concept]

**Key Points**:
- Point 1
- Point 2

[Optional diagram or code example]

(Author, Year) [inline citation]
```

---

### 4. Code Example

**Description**: Working code demonstrating a specific concept or pattern

**Attributes**:
- `example_id` (string): Unique identifier (e.g., "ch1-publisher-node")
- `filename` (string): Source file name (e.g., "publisher_node.py")
- `file_path` (path): Relative path from repo root
- `language` (string): Programming language ("python", "xml", "bash")
- `purpose` (text): 1-sentence description of what example demonstrates
- `prerequisites` (list[string]): Software/concepts needed to run
- `estimated_time_minutes` (integer): Time to complete exercise
- `difficulty` (string): "beginner", "intermediate", or "advanced"

**Contains**:
- `source_code` (text): Full working code with inline comments
- `setup_instructions` (list[string]): Step-by-step setup
- `expected_output` (text): What user should see when running
- `explanation` (text): Paragraph explaining key concepts demonstrated
- `common_errors` (list[object]): [{error, cause, solution}, ...]
- `extensions` (list[string]): Optional modification challenges

**Relationships**:
- Referenced by `Chapter` (embedded in chapter text)
- Belongs to `Chapter` (organizational parent)
- May reference `Reference` (if based on external source)

**File Location**: `code-examples/module-{module_slug}/chapter-{number}/{filename}`

**Validation Rules**:
- Must compile/run without errors on target platform (Ubuntu 22.04 + ROS 2 Humble)
- Includes header comment with purpose, prerequisites, usage
- Inline comments explain concepts (not just code mechanics)
- Accompanied by README.md in chapter directory

**README.md Structure**:
```markdown
# Chapter N Code Examples

## Setup
- Prerequisites
- Installation steps

## Running Examples
- example1.py: [purpose] - `python3 example1.py`
- example2.py: [purpose] - `python3 example2.py`

## Troubleshooting
- Common issue 1: solution
- Common issue 2: solution
```

---

### 5. Lab Guide

**Description**: Comprehensive hands-on exercise integrating multiple concepts (Chapter 4)

**Attributes**:
- `lab_id` (string): Unique identifier (e.g., "module1-final-lab")
- `title` (string): Lab title
- `objectives` (list[string]): 3-5 specific skills to demonstrate
- `estimated_duration_hours` (float): Time to complete (2-3 hours)
- `difficulty` (string): "beginner", "intermediate", "advanced"
- `chapter_number` (integer): Usually Chapter 4 (integrative)

**Contains**:
- `Introduction` (text): Lab overview and motivation
- `Prerequisites` (list[object]): [{type: "chapter"|"software", name, check_command}, ...]
- `Materials` (object): {software: list, hardware: list, starter_code_url: string}
- `Procedures` (list[object]): [{part_title, steps: list[{step_text, expected_outcome, checkpoint}]}]
- `Expected Outcomes` (list[string]): Measurable success criteria (checkboxes)
- `Troubleshooting` (list[object]): [{issue, likely_cause, solution}, ...]
- `Extensions` (list[string]): Optional challenges for advanced students
- `Submission` (object): {artifacts: list, reflection_questions: list} (if graded)

**Relationships**:
- Belongs to `Chapter` (usually Chapter 4)
- Integrates concepts from multiple `Chapter` entities
- References multiple `Code Example` entities
- May include new code specific to lab

**File Location**: Embedded in `docs/module-{module_slug}/04-lab-{slug}.md`

**Validation Rules**:
- All procedures tested end-to-end on target platform
- Checkpoints enable incremental validation
- Troubleshooting covers 80%+ of common issues (based on testing)
- Extensions are optional (not required for completion)

---

### 6. Practice Exercise

**Description**: Small challenge or question to reinforce learning within a chapter

**Attributes**:
- `exercise_id` (string): Unique identifier (e.g., "ch1-ex1")
- `number` (string): Display number (e.g., "Exercise 1.1")
- `description` (text): Problem statement or task description
- `difficulty` (string): "beginner", "intermediate", "advanced"
- `estimated_time_minutes` (integer): Expected completion time
- `hint` (text, optional): Optional guidance without giving away answer
- `solution_reference` (url, optional): Link to solutions repository (if provided)

**Contains**:
- Problem statement (text)
- Optional hint (text)
- Optional starter code (code snippet)

**Relationships**:
- Belongs to `Chapter` (parent)
- May reference `Code Example` (modify existing example)

**Markdown Structure**:
```markdown
## Practice Exercises

1. **Exercise 1.1**: [Description] (Difficulty: Beginner, ~10 minutes)
   - Hint: [Optional guidance]
   - Solution reference: [Link]

2. **Exercise 1.2**: [Description] (Difficulty: Intermediate, ~20 minutes)
   - Hint: [Optional guidance]
```

---

### 7. Reference

**Description**: Bibliographic citation for external source (APA format)

**Attributes**:
- `reference_id` (string): Unique identifier (e.g., "macenski2022")
- `citation_key` (string): Short form for inline citations (e.g., "Macenski et al., 2022")
- `authors` (list[string]): Author names (last, first format)
- `year` (integer): Publication year
- `title` (string): Full title
- `publication` (string): Journal, conference, or source name
- `volume` (string, optional): Volume number (journals)
- `issue` (string, optional): Issue number (journals)
- `pages` (string, optional): Page range
- `url` (string, optional): URL or DOI
- `type` (string): "peer-reviewed", "documentation", "specification", "book"
- `accessed_date` (date, optional): ISO 8601 format for online sources

**Relationships**:
- Referenced by `Chapter` entities (inline citations)
- Referenced by `Code Example` (if example based on external source)
- Compiled in module `references.md` file

**File Location**: `docs/module-{module_slug}/references.md`

**APA Format Example**:
```
Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. Science Robotics, 7(66). https://doi.org/10.1126/scirobotics.abm6074
```

**Validation Rules**:
- Minimum 15 references per module
- At least 50% peer-reviewed (type = "peer-reviewed")
- All URLs/DOIs accessible and stable
- No duplicate citations
- Every inline citation has matching reference entry

---

## Content Flow Diagram

```
Textbook
└── Module (4 total)
    ├── Module Index (overview, objectives, prerequisites)
    ├── Chapter 1 (P1 priority)
    │   ├── Introduction
    │   ├── Section 1.1
    │   ├── Section 1.2
    │   │   └── Code Example 1.1
    │   ├── Section 1.3
    │   │   └── Code Example 1.2
    │   ├── Practice Exercises (2-3)
    │   └── Summary
    ├── Chapter 2 (P2 priority)
    │   └── [similar structure]
    ├── Chapter 3 (P3 priority)
    │   └── [similar structure]
    ├── Chapter 4 (P4 priority - integrative lab)
    │   ├── Lab Introduction
    │   ├── Prerequisites Check
    │   ├── Procedures (multi-part)
    │   ├── Troubleshooting
    │   └── Extensions
    └── References (APA formatted bibliography)
```

---

## Validation Rules Summary

### Module-Level
- Word count: 5,000-7,000 total (sum of all chapters)
- Minimum 15 references, 50%+ peer-reviewed
- All chapters complete and validated
- Module index with objectives and prerequisites
- References section with all cited sources

### Chapter-Level
- Word count within ±15% of target
- Flesch-Kincaid grade 10-12
- Minimum 3 inline citations
- At least 2 code examples
- 2-3 practice exercises
- Summary with 3-5 key takeaways

### Code Example-Level
- Runs without errors on Ubuntu 22.04 + ROS 2 Humble
- Header comment with purpose, prerequisites, usage
- Inline comments explain concepts
- README.md with setup and run instructions
- Tested and reproducible

### Reference-Level
- APA format compliance
- Accessible URLs/DOIs
- No duplicates
- Matches all inline citations

---

## Entity Relationship Diagram (Textual)

```
Textbook (1) ──< has ──> (4) Module
Module (1) ──< contains ──> (4) Chapter
Module (1) ──< has ──> (1) References Section
Chapter (1) ──< contains ──> (3-5) Section
Chapter (1) ──< contains ──> (2-4) Code Example
Chapter (1) ──< contains ──> (2-3) Practice Exercise
Chapter (1) ──< may have ──> (0-1) Lab Guide
Chapter (n) ──< cites ──> (m) Reference
Code Example (n) ──< may cite ──> (m) Reference
Section (n) ──< may reference ──> (m) Code Example
```

---

**Data Model Status**: ✅ Complete
**Entities Defined**: 7 (Module, Chapter, Section, Code Example, Lab Guide, Practice Exercise, Reference)
**Validation Rules**: Comprehensive quality criteria established
**Ready for Template Creation**: Yes
