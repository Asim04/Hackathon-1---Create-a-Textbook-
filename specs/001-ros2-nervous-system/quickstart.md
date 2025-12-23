# Quickstart: Creating Module 1 Content

**Purpose**: Guide for content creators (human or AI) to produce Module 1: The Robotic Nervous System (ROS 2) following established templates and constitution standards

**Last Updated**: 2025-12-17

---

## Overview

This guide provides a step-by-step workflow for creating educational content that meets the Physical AI & Humanoid Robotics textbook constitution requirements:
- **Accuracy**: Factually correct, cited in APA style
- **Clarity**: Flesch-Kincaid grade 10-12 readability
- **Reproducibility**: 100% working code examples
- **Rigor**: Minimum 15 sources, 50%+ peer-reviewed
- **Originality**: Zero plagiarism
- **Modularity**: Self-contained, Spec-Kit Plus compatible

---

## Prerequisites

Before creating content, ensure you have:
- [ ] Access to `research.md` (sources and technical decisions)
- [ ] Access to `data-model.md` (content structure requirements)
- [ ] Access to `contracts/` templates (chapter, code example, lab guide)
- [ ] Familiarity with Markdown and Docusaurus
- [ ] Access to Ubuntu 22.04 + ROS 2 Humble for testing code examples
- [ ] Text editor (VS Code, Vim, etc.)
- [ ] Readability analysis tool (Hemingway Editor, textstat library)
- [ ] Plagiarism detection tool (Turnitin or equivalent)

---

## Content Creation Workflow

### Step 1: Review Research & Data Model (15 minutes)

**Actions**:
1. Read `research.md` to understand:
   - 16 identified sources (9 peer-reviewed)
   - Technical decisions (ROS 2 Humble, Gazebo Classic, Docusaurus)
   - Chapter word count targets: Ch1=1,500w, Ch2=1,200w, Ch3=1,300w, Ch4=2,000w

2. Review `data-model.md` for:
   - Content entity definitions (Module, Chapter, Code Example, Lab Guide, Reference)
   - Validation rules (word count, readability, citations)
   - Required chapter structure

3. Check constitution standards (`.specify/memory/constitution.md`):
   - Total module: 5,000-7,000 words
   - Minimum 15 citations, 50%+ peer-reviewed
   - APA citation format
   - Flesch-Kincaid grade 10-12

**Output**: Clear understanding of requirements and constraints

---

### Step 2: Write Chapter Content (4-8 hours per chapter)

**Actions**:
1. Copy `contracts/chapter-template.md` to target location:
   ```bash
   cp specs/001-ros2-nervous-system/contracts/chapter-template.md \
      docs/module-1-ros2/[chapter-slug].md
   ```

2. Fill in template sections:
   - **Front matter** (if using Docusaurus):
     ```markdown
     ---
     sidebar_position: [N]
     title: "Chapter [N]: [Title]"
     description: "[Brief chapter description]"
     ---
     ```

   - **Learning Objectives**: 2-3 specific, measurable outcomes using action verbs (explain, create, analyze, demonstrate)

   - **Prerequisites**: List prior chapters and/or external knowledge needed

   - **Introduction**: 2-3 paragraphs (motivation, connection to prior/future chapters, overview)

   - **Core Content**: 3-5 sections
     - Each section: 2-4 paragraphs explaining concept
     - Define technical terms on first use
     - Use examples, analogies, or diagrams (Mermaid.js descriptions acceptable)
     - Include **Key Points** bullets for important takeaways
     - Embed code examples inline (see Step 3)
     - Add inline APA citations for factual claims: (Author, Year)

   - **Practice Exercises**: 2-3 exercises with difficulty levels and time estimates

   - **Summary**: 3-5 key takeaways as bullet points

   - **Further Reading**: 2-3 references with relevance descriptions

3. Target word counts:
   - Chapter 1: 1,500 words ±200
   - Chapter 2: 1,200 words ±150
   - Chapter 3: 1,300 words ±150
   - Chapter 4: 2,000 words ±250

4. Embed inline citations:
   - Every factual claim needs citation: (Author, Year)
   - Minimum 3-4 unique sources per chapter
   - Track citations to add to references.md later

5. Use clear, educational language:
   - Explain technical jargon on first use
   - Use concrete examples and analogies
   - Break complex concepts into steps
   - Maintain conversational but professional tone

**Output**: Draft chapter file in `docs/module-1-ros2/[chapter-slug].md`

---

### Step 3: Create Code Examples (1-2 hours per example)

**Actions**:
1. Copy `contracts/code-example-template.md` for reference (documentation)

2. Create working code file:
   ```bash
   # Create file in appropriate chapter directory
   touch code-examples/module-1-ros2/chapter-[N]/[filename].py
   ```

3. Write code following template structure:
   - **Header docstring**: Purpose, prerequisites, usage
   - **Inline comments**: Explain CONCEPTS (why), not just mechanics (how)
   - **Clear structure**: Logical flow, readable variable names
   - **Error handling**: Basic try/except for common failures
   - **Educational focus**: Demonstrate one clear concept per example

4. Code requirements:
   - Must run without errors on Ubuntu 22.04 + ROS 2 Humble
   - 50-150 lines (keep concise for educational clarity)
   - Follow PEP 8 for Python code
   - Include `if __name__ == '__main__':` for executable scripts

5. Create README.md in chapter directory:
   ```markdown
   # Chapter [N] Code Examples

   ## Setup
   - Prerequisites: [List]
   - Installation: [Commands]

   ## Running Examples
   - `example1.py`: [Purpose] - Run with: `python3 example1.py`
   - `example2.py`: [Purpose] - Run with: `python3 example2.py`

   ## Troubleshooting
   - [Common issue 1]: [Solution]
   - [Common issue 2]: [Solution]
   ```

6. Test code (see Step 4)

7. Embed code in chapter markdown:
   ```markdown
   **Code Example [N].[M]**: [Descriptive Title]

   ```python
   [Full code with comments]
   ```

   [Explanation paragraph: What does this demonstrate? How to run? What to observe?]
   ```

**Output**: Working code files in `code-examples/module-1-ros2/chapter-[N]/` and embedded in chapter

---

### Step 4: Test Code Examples (30 minutes per example)

**Actions**:
1. Set up clean test environment:
   ```bash
   # Verify ROS 2 installation
   source /opt/ros/humble/setup.bash
   ros2 --version

   # Navigate to code directory
   cd code-examples/module-1-ros2/chapter-[N]/
   ```

2. Test each example:
   ```bash
   # Run example
   python3 [filename].py

   # Verify expected output matches documentation
   # Check for errors or warnings
   ```

3. Test with intentional errors (if applicable):
   - Wrong parameters
   - Missing prerequisites
   - Network issues
   - Document common errors in README troubleshooting section

4. Verify reproducibility:
   - Test on fresh Ubuntu 22.04 VM if possible
   - Follow setup instructions exactly as documented
   - Confirm 100% success rate

**Output**: Verified, reproducible code examples

---

### Step 5: Create Chapter 4 Lab Guide (6-10 hours)

**Actions**:
1. Copy `contracts/lab-guide-template.md` to `docs/module-1-ros2/04-lab-[slug].md`

2. Design integrative lab:
   - **Objectives**: 3-5 skills from Chapters 1-3
   - **Prerequisites**: Checklist of prior chapters and software
   - **Materials**: List all software, starter code URL
   - **Procedures**: 3-4 parts with step-by-step instructions
     - Each step: command/action, expected outcome, checkpoint
   - **Troubleshooting**: Table with 4-6 common issues + solutions
   - **Extensions**: 2-3 optional advanced challenges

3. Create lab solution code:
   ```bash
   mkdir -p code-examples/module-1-ros2/chapter-4/full_lab_solution/
   # Create all necessary files (URDF, launch files, Python scripts)
   ```

4. Test lab end-to-end:
   - Follow procedures exactly as written
   - Verify every checkpoint passes
   - Time yourself (should match estimated duration ±30%)
   - Document any encountered issues in troubleshooting section

5. Verify lab meets success criteria:
   - All 4 outcomes from spec.md achievable
   - Lab demonstrates mastery of Module 1 objectives
   - Integrates concepts from all prior chapters

**Output**: Complete lab guide in `docs/module-1-ros2/04-lab-[slug].md` with working solution code

---

### Step 6: Compile References (1 hour)

**Actions**:
1. Extract all cited sources from Chapters 1-4:
   ```bash
   # Search for inline citations (Author, Year) pattern
   grep -oE '\([A-Z][a-z]+( et al\.)?, [0-9]{4}\)' docs/module-1-ros2/*.md | sort | uniq
   ```

2. Create `docs/module-1-ros2/references.md`:
   ```markdown
   ---
   sidebar_position: 6
   title: "References"
   description: "Bibliography for Module 1"
   ---

   # References

   All sources cited in Module 1, formatted in APA style.

   ## Peer-Reviewed Literature

   [Author], [Initials]. ([Year]). [Title]. *[Journal]*, *[Volume]*([Issue]), [Pages]. [DOI/URL]

   ## Official Documentation

   [Organization]. ([Year]). [Title]. [URL]

   ## Books and Chapters

   [Author], [Initials]. ([Year]). [Chapter title]. In *[Book title]* (pp. [pages]). [Publisher].
   ```

3. Format each reference in APA style:
   - Use research.md as starting point (16 sources already formatted)
   - Add any additional sources cited in chapters
   - Alphabetize by first author's last name

4. Validate:
   - Minimum 15 references total
   - At least 50% peer-reviewed (8+)
   - Every inline citation has matching reference entry
   - No duplicate entries
   - All URLs/DOIs accessible (click each link)

**Output**: Complete `docs/module-1-ros2/references.md` file

---

### Step 7: Quality Validation (2-3 hours)

Run these validation checks before considering module complete:

#### Word Count Check
```bash
# Count words in all chapter files
cd docs/module-1-ros2/
wc -w 01-*.md 02-*.md 03-*.md 04-*.md

# Expected output:
# Ch1: ~1,500 words
# Ch2: ~1,200 words
# Ch3: ~1,300 words
# Ch4: ~2,000 words
# Total: 5,000-7,000 words
```

**Validation**: ✅ Total within 5,000-7,000 range, chapters within ±15% of targets

#### Readability Check

**Option A: Online Tool** (Hemingway Editor)
1. Go to https://hemingwayapp.com/
2. Paste chapter content
3. Check readability grade (target: 10-12)
4. If grade >12, simplify complex sentences

**Option B: Python textstat Library**
```bash
# Install textstat
pip install textstat

# Check each chapter
python3 << EOF
import textstat
with open('01-ros2-fundamentals.md', 'r') as f:
    content = f.read()
    grade = textstat.flesch_kincaid_grade(content)
    print(f"Flesch-Kincaid Grade: {grade}")
EOF

# Target: Grade 10-12
```

**Validation**: ✅ All chapters grade 10-12

#### Citation Count
```bash
# Count inline citations (Author, Year) pattern
grep -o '([A-Z][a-z]*, [0-9]\{4\})' docs/module-1-ros2/*.md | wc -l

# Count unique references in references.md
grep -E '^\[?[A-Z]' docs/module-1-ros2/references.md | wc -l

# Counts should match (or citations slightly higher if some sources cited multiple times)
```

**Validation**: ✅ Minimum 15 references, 50%+ peer-reviewed

#### Code Testing
```bash
# Test syntax (Python files)
cd code-examples/module-1-ros2/
find . -name "*.py" -exec python3 -m py_compile {} \;

# If no errors, syntax is valid
# Then manually run each example and verify output matches documentation
```

**Validation**: ✅ All code examples run without errors, outputs match documentation

#### Plagiarism Check
- Run all chapter content through Turnitin or equivalent plagiarism detection tool
- Target: 100% original content (properly cited paraphrases are acceptable)
- Any similarity >10% should be reviewed and rewritten if not properly quoted/cited

**Validation**: ✅ Plagiarism check passed

---

### Step 8: Docusaurus Integration (1 hour)

**Actions**:
1. Ensure `sidebars.js` includes Module 1:
   ```javascript
   // Already created in Phase 1, verify structure:
   {
     type: 'category',
     label: 'Module 1: ROS 2 Nervous System',
     items: [
       'module-1-ros2/index',
       'module-1-ros2/01-ros2-fundamentals',
       'module-1-ros2/02-python-agents-ros',
       'module-1-ros2/03-urdf-humanoids',
       'module-1-ros2/04-lab-building-ros2-robot',
       'module-1-ros2/references',
     ],
   }
   ```

2. Add Docusaurus front matter to all markdown files:
   ```markdown
   ---
   sidebar_position: [N]
   title: "[Full Title]"
   description: "[Brief description for SEO/preview]"
   ---
   ```

3. Create module index (`docs/module-1-ros2/index.md`):
   ```markdown
   ---
   sidebar_position: 1
   title: "Module 1: The Robotic Nervous System (ROS 2)"
   description: "Module overview, objectives, and prerequisites"
   ---

   # Module 1: The Robotic Nervous System (ROS 2)

   ## Overview
   [2-3 paragraphs from spec.md]

   ## Learning Objectives
   - [Objective 1]
   - [Objective 2]
   - [Objective 3]

   ## Prerequisites
   - [Prerequisite 1]
   - [Prerequisite 2]

   ## Module Structure
   1. Chapter 1: ROS 2 Fundamentals
   2. Chapter 2: Python Agents & ROS Integration
   3. Chapter 3: URDF for Humanoids
   4. Chapter 4: Hands-On Lab

   ## Estimated Time
   10-15 hours

   ## Related Modules
   - Module 2: The Digital Twin
   - Module 3: The AI-Robot Brain
   ```

4. Test Docusaurus build:
   ```bash
   # Install dependencies (first time only)
   npm install

   # Start development server
   npm run start

   # Open browser to http://localhost:3000
   # Navigate through Module 1, verify:
   # - All pages load
   # - Sidebar navigation works
   # - Code syntax highlighting works
   # - Links between pages work
   ```

5. Test production build:
   ```bash
   npm run build

   # Should complete without errors
   # Output: build/ directory with static HTML files
   ```

**Validation**: ✅ Docusaurus builds without errors, all navigation functional

---

## Common Pitfalls

### ❌ Mistake: Missing inline citations
**Problem**: Writing factual claims without (Author, Year) citations
**Solution**: Add citation after every factual claim, definition, or statistic

### ❌ Mistake: Code examples not tested
**Problem**: Publishing code that doesn't run or has errors
**Solution**: Test every example on Ubuntu 22.04 + ROS 2 Humble before publishing

### ❌ Mistake: Technical jargon without explanation
**Problem**: Using terms like "DDS", "QoS", "URDF" without defining on first use
**Solution**: First use: "Quality of Service (QoS)" then "QoS" after

### ❌ Mistake: Reading level too advanced
**Problem**: Graduate-level writing (grade 16+) instead of undergrad (10-12)
**Solution**: Use Hemingway Editor, simplify complex sentences, avoid passive voice

### ❌ Mistake: Word count violations
**Problem**: Chapter significantly over/under target word count
**Solution**: Check word count after each section, adjust content as needed

### ❌ Mistake: Unreproducible code
**Problem**: Code works on author's machine but not fresh installation
**Solution**: Test on fresh Ubuntu 22.04 VM or follow installation instructions exactly

---

## Resources

**Specification & Planning**:
- Constitution: `.specify/memory/constitution.md`
- Feature Spec: `specs/001-ros2-nervous-system/spec.md`
- Implementation Plan: `specs/001-ros2-nervous-system/plan.md`
- Research: `specs/001-ros2-nervous-system/research.md`
- Data Model: `specs/001-ros2-nervous-system/data-model.md`

**Templates**:
- Chapter: `specs/001-ros2-nervous-system/contracts/chapter-template.md`
- Code Example: `specs/001-ros2-nervous-system/contracts/code-example-template.md`
- Lab Guide: `specs/001-ros2-nervous-system/contracts/lab-guide-template.md`

**External Resources**:
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- URDF Specification: http://wiki.ros.org/urdf/XML
- APA Citation Guide: https://apastyle.apa.org/
- Hemingway Editor: https://hemingwayapp.com/
- Docusaurus Docs: https://docusaurus.io/docs

---

## Success Criteria Checklist

Before considering Module 1 complete, verify:

- [ ] All 4 chapters written and complete
- [ ] Module index created
- [ ] References compiled (15+ sources, 50%+ peer-reviewed)
- [ ] Word count: 5,000-7,000 total
- [ ] Readability: Flesch-Kincaid grade 10-12 (all chapters)
- [ ] Citations: All factual claims cited in APA format
- [ ] Code examples: 15-20 total, all tested and reproducible
- [ ] Lab guide: Chapter 4 integrative lab complete with solution
- [ ] Plagiarism check: Passed (100% original)
- [ ] Docusaurus: Builds without errors, navigation functional
- [ ] Quality review: Subject matter expert validation (if applicable)

---

**Quickstart Guide Status**: ✅ Complete
**Ready for Content Creation**: Yes
**Estimated Time to Complete Module 1**: 40-60 hours (1-2 weeks full-time)
