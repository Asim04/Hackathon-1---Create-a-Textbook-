---
id: 001
title: Ratify textbook constitution
stage: constitution
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "textbook", "physical-ai", "robotics", "initial-ratification"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - N/A
---

## Prompt

```text
/sp.constitution

Project: Textbook on Physical AI & Humanoid Robotics

Core Principles:
- Accuracy: All content must be factually correct and verified against primary sources.
- Clarity: Writing should be clear and suitable for a technical audience (computer science and engineering students).
- Reproducibility: All examples, experiments, and references must be traceable and verifiable.
- Rigor: Prefer peer-reviewed sources and authoritative references. Avoid speculative or unverified claims.

Key Standards:
- Factual claims must be cited. Citation format: APA style.
- Minimum 50% of sources should be peer-reviewed articles or official technical publications.
- Zero tolerance for plagiarism; content must be original.
- Writing level: Flesch-Kincaid grade 10-12 (understandable but technical).

Constraints:
- Total word count per module: 5,000–7,000 words.
- Minimum 15 sources per module.
- Format: PDF with embedded citations.
- Must integrate instructions for building practical AI & humanoid robotics experiments.

Success Criteria:
- All content verified and accurate.
- Zero plagiarism.
- Fully reproducible experiments/examples.
- Passes technical review.
- Ready for deployment via Docusaurus and GitHub Pages with embedded RAG chatbot integration.

Special Instructions:
- Textbook must be modular for Spec-Kit Plus generation.
- Include prompts and structures compatible with Claude Code for automated content creation.
- Ensure content aligns with a course curriculum for Physical AI & Humanoid Robotics.
```

## Response snapshot

Created the initial constitution (v1.0.0) for the Physical AI & Humanoid Robotics textbook project with six core principles:

1. **Accuracy**: All content must be factually correct, verified, and cited (APA style)
2. **Clarity**: Technical writing at Flesch-Kincaid grade 10-12 level
3. **Reproducibility**: All experiments and examples must be traceable and verifiable
4. **Rigor**: Minimum 15 sources per module, 50%+ peer-reviewed
5. **Originality**: Zero tolerance for plagiarism
6. **Modularity**: Content structured for Spec-Kit Plus and Claude Code workflows

Added Content Standards section specifying:
- 5,000–7,000 words per module
- APA citation format with embedded references
- PDF output compatible with Docusaurus and GitHub Pages deployment
- RAG chatbot integration support
- Practical experiments with detailed instructions

Added Development Workflow section with:
- 6-step content creation process (Research → Draft → Review → Test → Technical Review → Finalize)
- Quality gates at pre-draft, post-draft, and pre-publication stages
- Automation requirements for Claude Code and Spec-Kit Plus integration

Added Governance section establishing:
- Constitution authority over all project practices
- Semantic versioning for amendments (MAJOR.MINOR.PATCH)
- Compliance review requirements
- References to related artifacts (specs, plans, tasks, PHRs, ADRs)

## Outcome

- ✅ Impact: Established foundational governance for textbook content creation with clear quality standards and automation requirements
- 🧪 Tests: N/A (governance document)
- 📁 Files: Created/updated `.specify/memory/constitution.md` (v1.0.0)
- 🔁 Next prompts: Create feature specifications for individual textbook modules using `/sp.specify`
- 🧠 Reflection: Constitution provides clear, testable standards aligned with academic textbook requirements. All principles include explicit rationale. Ready for module development.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (initial ratification)
- Prompt variant (if applicable): Standard `/sp.constitution` workflow
- Next experiment (smallest change to try): Test constitution compliance during first module specification with `/sp.specify`
