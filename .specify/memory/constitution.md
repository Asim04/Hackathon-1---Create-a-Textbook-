<!--
Sync Impact Report
==================
Version Change: 0.0.0 → 1.0.0
Modified Principles: N/A (initial ratification)
Added Sections: Core Principles (6), Content Standards, Development Workflow, Governance
Removed Sections: None
Templates Requiring Updates:
  - ✅ .specify/templates/plan-template.md (aligned with constitution checks)
  - ✅ .specify/templates/spec-template.md (aligned with requirements format)
  - ✅ .specify/templates/tasks-template.md (aligned with modular approach)
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook Constitution

## Core Principles

### I. Accuracy
All content MUST be factually correct and verified against primary sources. No speculative or unverified claims are permitted. Every factual claim MUST be cited using APA style. A minimum of 50% of sources MUST be peer-reviewed articles or official technical publications.

**Rationale**: Academic and technical credibility depends on factual accuracy. Students and educators rely on textbooks as authoritative references; errors undermine learning and trust.

### II. Clarity
Writing MUST be clear and suitable for a technical audience (computer science and engineering students). Target Flesch-Kincaid reading level: grade 10-12. Technical terminology MUST be explained on first use. Complex concepts MUST be broken down with examples, diagrams, or analogies where appropriate.

**Rationale**: Textbooks serve as learning tools. Clarity ensures accessibility to the target audience while maintaining technical rigor.

### III. Reproducibility
All examples, experiments, and references MUST be traceable and verifiable. Experiments MUST include step-by-step instructions that enable readers to reproduce results. Code samples MUST be tested and functional. External resources MUST be accessible and stable.

**Rationale**: Hands-on learning through reproducible experiments solidifies theoretical knowledge. Students must be able to verify claims and build practical skills.

### IV. Rigor
Content MUST prioritize peer-reviewed sources and authoritative references. Claims MUST be supported by evidence from reputable sources. Speculation MUST be clearly labeled as such and distinguished from established facts. Each module MUST include a minimum of 15 sources.

**Rationale**: Academic rigor ensures the textbook maintains scholarly standards and provides reliable knowledge foundation for advanced study and research.

### V. Originality
Content MUST be original. Zero tolerance for plagiarism. All borrowed ideas, quotes, and paraphrased content MUST be properly attributed with citations. Content reuse from other sources MUST be properly licensed or in public domain.

**Rationale**: Intellectual integrity is non-negotiable in academic work. Plagiarism violates ethical standards and legal requirements.

### VI. Modularity
Content MUST be structured in independently deployable modules. Each module MUST be self-contained with clear learning objectives, prerequisites, and outcomes. Modules MUST integrate with Spec-Kit Plus generation workflow and be compatible with Claude Code for automated content creation.

**Rationale**: Modular design enables flexible curriculum design, iterative development, and automated tooling integration for efficient content creation and maintenance.

## Content Standards

### Word Count and Structure
- Each module MUST contain 5,000–7,000 words
- Content MUST be organized with clear headings, subheadings, and logical flow
- Each module MUST include: introduction, learning objectives, core content, examples/experiments, summary, references, exercises (optional)

### Citation and Reference Requirements
- Citation format: APA style (mandatory)
- Minimum 15 sources per module (mandatory)
- At least 50% of sources MUST be peer-reviewed articles or official technical publications
- Citations MUST be embedded inline and compiled in a references section
- All URLs in references MUST be accessible and stable (use DOI when available)

### Format and Delivery
- Output format: PDF with embedded citations
- Content MUST be compatible with Docusaurus static site generation
- Content MUST support GitHub Pages deployment
- Integration with RAG (Retrieval-Augmented Generation) chatbot MUST be supported

### Practical Components
- Each module MUST include instructions for building practical experiments related to Physical AI and Humanoid Robotics
- Experiments MUST be documented with clear objectives, materials list, procedures, expected outcomes, and troubleshooting guidance
- Code examples MUST be tested and include comments explaining key concepts

## Development Workflow

### Content Creation Process
1. **Research**: Gather primary sources, verify factual accuracy, document citations
2. **Draft**: Write content following clarity and rigor principles, embed citations
3. **Review**: Verify all claims against sources, check citation format, ensure originality
4. **Test**: Validate experiments and code samples for reproducibility
5. **Technical Review**: Subject matter expert validation (required before publication)
6. **Finalize**: Generate PDF, validate formatting, prepare for deployment

### Quality Gates
- **Pre-Draft Gate**: Minimum 15 sources identified and verified
- **Post-Draft Gate**: All factual claims cited, plagiarism check passed (100% original)
- **Pre-Publication Gate**: Technical review completed, all experiments reproducible, format validated

### Automation and Tooling
- Content generation MUST be compatible with Claude Code workflows
- Templates MUST support Spec-Kit Plus modular structure
- Content MUST include metadata for automated deployment via GitHub Pages
- RAG chatbot integration MUST be considered in content structuring (clear section headings, self-contained explanations)

## Governance

### Constitution Authority
This constitution supersedes all other project practices and guidelines. All content creation, review, and publication decisions MUST comply with these principles and standards.

### Amendment Procedure
1. Proposed amendments MUST be documented with rationale and impact analysis
2. Amendments MUST receive approval from project lead and technical reviewers
3. Amendments require migration plan for existing content to align with new standards
4. Version MUST be incremented following semantic versioning:
   - **MAJOR**: Backward-incompatible changes (e.g., citation format change, principle removal)
   - **MINOR**: New principles or standards added (e.g., new quality gate)
   - **PATCH**: Clarifications, wording improvements, non-semantic refinements

### Compliance and Review
- All content PRs MUST verify compliance with constitution principles
- Technical reviewers MUST validate accuracy, rigor, and reproducibility
- Complexity or deviations from standards MUST be explicitly justified and documented
- Automated checks SHOULD be implemented where possible (word count, citation count, plagiarism detection)

### Related Artifacts
- Runtime development guidance: `CLAUDE.md`
- Feature specifications: `specs/<feature>/spec.md`
- Implementation plans: `specs/<feature>/plan.md`
- Task definitions: `specs/<feature>/tasks.md`
- Prompt History Records: `history/prompts/`
- Architecture Decision Records: `history/adr/`

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17
