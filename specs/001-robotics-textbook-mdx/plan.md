# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `001-robotics-textbook-mdx` | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-robotics-textbook-mdx/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a comprehensive interactive textbook in MDX format using Docusaurus 3.5.2 covering Physical AI and Humanoid Robotics across 4 modules (ROS 2, Simulation, NVIDIA Isaac, Vision-Language-Action) plus a Capstone Project. Each module includes detailed explanations, runnable code examples, exercises, diagrams (Mermaid.js where possible), and hardware/software deployment notes. The textbook leverages modern Docusaurus features (offline search, math rendering, code tabs, collapsibles) and is deployable on GitHub Pages or Vercel for universal accessibility.

## Technical Context

**Language/Version**: MDX for content; JavaScript/TypeScript for Docusaurus config; Python 3.10+ for code examples; C++ 17 for ROS 2 nodes
**Primary Dependencies**: Docusaurus 3.5.2, Node.js 22.x, React 18, @easyops-cn/docusaurus-search-local 0.40.x, @docusaurus/theme-mermaid 3.5.x, remark-math 6.0.x, rehype-katex 7.0.x (KaTeX 0.16.x)
**Storage**: Static file generation (Docusaurus builds to static HTML/CSS/JS); no database
**Testing**: Docusaurus build validation, markdown-link-check for internal links, Lighthouse CI + axe-core for accessibility (WCAG 2.1 AA), manual code example testing
**Target Platform**: Web browsers (Chrome, Firefox, Safari, Edge current versions); deployable on GitHub Pages or Vercel
**Project Type**: Static documentation site (Docusaurus project structure)
**Performance Goals**: First meaningful paint <3s on standard broadband; search response <500ms; offline-capable search
**Constraints**: Offline-first search (no external API dependencies); open-source only; WCAG 2.1 AA accessibility; build without errors
**Scale/Scope**: 4 modules + capstone (20-25 chapters total); ~50-100 MDX files; ~200+ code examples; ~50+ diagrams/images; deployable site ~50-100MB

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Content Completeness & Accuracy ✅ PASS
- Plan ensures all modules include detailed explanations, runnable code (Python/C++), exercises, hardware/software notes
- Code examples will be tested during implementation with documented dependencies
- Technical Context specifies exact versions for reproducibility

### II. Learner-Centric Design ✅ PASS
- User stories prioritize progressive learning (Module 1 → Modules 2-4 → Capstone)
- Each module includes learning outcomes, prerequisites, context/motivation
- Exercises build on previous concepts per FR-014

### III. Modern Interactive Documentation ✅ PASS
- Docusaurus 3.5.2 provides all required features: collapsibles, code tabs, copy-to-clipboard, sidebar, search, light/dark mode
- @easyops-cn/docusaurus-search-local plugin configured for full-text search
- FR-022 through FR-027 explicitly require these interactive features

### IV. Visual Learning Support ✅ PASS
- Mermaid.js integration (@docusaurus/theme-mermaid) for automatable diagrams (flowcharts, sequence, state, class diagrams)
- FR-019, FR-020 require visual content for architectures, data flows, kinematics with placeholders where manual creation needed
- All images will have alt text per FR-021 for accessibility

### V. Open Source & Accessibility ✅ PASS
- All dependencies are open-source: Docusaurus (MIT), ROS 2 (Apache 2.0), Gazebo (Apache 2.0), KaTeX (MIT)
- Deployable on GitHub Pages/Vercel (free platforms) per FR-030
- WCAG 2.1 AA compliance required per SC-011, tested with Lighthouse CI + axe-core

### VI. Reproducibility & Environment Clarity ✅ PASS
- All versions pinned: Docusaurus 3.5.2, Node.js 22.x, KaTeX 0.16.x
- FR-011 requires exact software versions, dependencies, expected outputs for all code examples
- Hardware-specific examples have simulation alternatives per FR-018

### VII. Incremental Modular Architecture ✅ PASS
- Each module has index.md with overview, learning outcomes, prerequisites per FR-002
- Cross-references required via FR-027 (markdown links with clear destination text)
- Module independence supported by explicit prerequisite documentation per spec assumptions

**Overall Constitution Status**: ✅ **ALL GATES PASS** - Proceed to Phase 0 Research

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
robotics-textbook/
├── docs/                             # MDX content files (Docusaurus)
│   ├── intro.md                     # Homepage with course overview
│   ├── module-1-ros2/               # Module 1: ROS 2
│   │   ├── index.md                 # Module overview
│   │   ├── 1-1-ros2-intro.md
│   │   ├── 1-2-nodes-topics.md
│   │   ├── 1-3-services-actions.md
│   │   ├── 1-4-parameters.md
│   │   ├── 1-5-urdf.md
│   │   └── exercises/
│   │       └── module-1-exercises.md
│   ├── module-2-simulation/         # Module 2: Gazebo & Unity
│   │   ├── index.md
│   │   ├── 2-1-gazebo-intro.md
│   │   ├── 2-2-unity-intro.md
│   │   ├── 2-3-sensors.md
│   │   ├── 2-4-advanced.md
│   │   └── exercises/
│   ├── module-3-isaac/              # Module 3: NVIDIA Isaac
│   │   ├── index.md
│   │   ├── 3-1-isaac-sim-intro.md
│   │   ├── 3-2-reinforcement-learning.md
│   │   ├── 3-3-training-scenarios.md
│   │   ├── 3-4-sim-to-real.md
│   │   └── exercises/
│   ├── module-4-vla/                # Module 4: Vision-Language-Action
│   │   ├── index.md
│   │   ├── 4-1-vision.md
│   │   ├── 4-2-language.md
│   │   ├── 4-3-vla-models.md
│   │   ├── 4-4-deployment.md
│   │   └── exercises/
│   ├── capstone/                    # Capstone Project
│   │   ├── index.md
│   │   ├── architecture-guide.md
│   │   └── faq.md
│   └── resources/                   # Appendices
│       ├── glossary.md
│       ├── hardware-specs.md
│       ├── software-setup.md
│       └── further-reading.md
├── src/                             # React components and styles
│   ├── components/                  # Custom React components (if needed)
│   └── css/
│       └── custom.css              # Theme customization
├── static/                          # Static assets
│   └── img/
│       ├── diagrams/               # Mermaid diagram exports
│       ├── hardware/               # Hardware photos
│       ├── logos/                  # Project logos
│       └── screenshots/            # UI screenshots
├── docusaurus.config.js            # Site configuration
├── sidebars.js                      # Navigation structure
├── package.json                     # Node.js dependencies
└── README.md                        # Project documentation
```

**Structure Decision**: Docusaurus standard documentation site structure. Content organized by learning progression (intro → modules 1-4 → capstone → resources). MDX files in `docs/` directory with hierarchical organization matching sidebar navigation. Static assets in `static/img/` subdirectories. Configuration files at repository root.

## Complexity Tracking

**Not applicable** - All Constitution principles pass without exceptions. No violations to justify.

---

## Phase 0: Research & Technology Decisions

**Completed**: 2025-12-05
**Output**: [`research.md`](./research.md)

### Research Questions Resolved

1. **Docusaurus Configuration**: Use @docusaurus/preset-classic with Docusaurus 3.5.2
2. **Search Implementation**: @easyops-cn/docusaurus-search-local for offline-capable search
3. **Mermaid Integration**: @docusaurus/theme-mermaid for automatable diagrams
4. **Math Rendering**: KaTeX 0.16.x via remark-math/rehype-katex
5. **Code Tabs**: Native Docusaurus Tabs component for multi-language comparison
6. **Collapsibles**: HTML details/summary + Docusaurus admonitions
7. **Accessibility Testing**: Lighthouse CI + axe-core (automated) + manual testing
8. **Deployment**: GitHub Pages (primary), Vercel (alternative)

All technical decisions documented with rationale, alternatives considered, and implementation notes.

---

## Phase 1: Design & Contracts

**Completed**: 2025-12-05
**Outputs**:
- [`data-model.md`](./data-model.md) - Content entity definitions
- [`contracts/content-structure.md`](./contracts/content-structure.md) - MDX frontmatter schemas, directory conventions
- [`quickstart.md`](./quickstart.md) - Contributor onboarding guide
- Agent context updated in `CLAUDE.md`

### Data Model

8 content entities defined:
1. **Module**: Top-level learning unit (4 modules + capstone)
2. **Chapter**: Individual lessons within modules
3. **CodeExample**: Runnable code snippets with dependencies
4. **Exercise**: Practice tasks with validation
5. **Diagram**: Visual content (Mermaid or placeholder)
6. **HardwareSpec**: Equipment specifications
7. **GlossaryEntry**: Term definitions
8. **LearningOutcome**: Measurable skills/knowledge

### Contracts

Content structure contracts define:
- MDX frontmatter schemas (module index, chapter, exercise, resource files)
- Directory organization patterns
- Code block conventions (language identifiers, titles, copy buttons)
- Mermaid diagram syntax standards
- Collapsible content patterns (details/summary, admonitions)
- Cross-reference link formats
- Accessibility requirements (alt text, semantic HTML)
- Build validation checkpoints

---

## Post-Phase 1 Constitution Re-Check

✅ **ALL GATES STILL PASS**

- **I. Content Completeness**: data-model.md defines CodeExample entity with required dependencies/testing
- **II. Learner-Centric**: Module/Chapter structure enforces learning outcomes and prerequisites
- **III. Modern Interactive**: Contracts specify Tabs, collapsibles, search, math rendering patterns
- **IV. Visual Learning**: Diagram entity covers Mermaid + placeholders strategy
- **V. Open Source & Accessibility**: All tech choices verified open-source; contracts mandate WCAG 2.1 AA
- **VI. Reproducibility**: data-model requires version pinning in CodeExample entities
- **VII. Modular Architecture**: Module entity enforces prerequisites and cross-references

**Recommendation**: ✅ **PROCEED TO `/sp.tasks`** - Planning complete, ready for task breakdown
