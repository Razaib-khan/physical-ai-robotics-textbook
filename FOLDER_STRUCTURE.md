# Project Folder Structure

Generated: 2025-12-05

## Complete Directory Tree

```
robotics-textbook/
├── .claude/                          # Claude Code configuration
│   └── commands/                     # Custom slash commands
│       ├── sp.adr.md
│       ├── sp.analyze.md
│       ├── sp.checklist.md
│       ├── sp.clarify.md
│       ├── sp.constitution.md
│       ├── sp.git.commit_pr.md
│       ├── sp.implement.md
│       ├── sp.phr.md
│       ├── sp.plan.md
│       ├── sp.specify.md
│       └── sp.tasks.md
│
├── .specify/                         # SpecKit Plus templates and scripts
│   ├── memory/
│   │   └── constitution.md          # Project constitution v1.0.0
│   ├── scripts/
│   │   └── bash/                    # Shell scripts for workflows
│   │       ├── check-prerequisites.sh
│   │       ├── common.sh
│   │       ├── create-adr.sh
│   │       ├── create-new-feature.sh
│   │       ├── create-phr.sh
│   │       ├── setup-plan.sh
│   │       └── update-agent-context.sh
│   └── templates/                   # Document templates
│       ├── adr-template.md
│       ├── agent-file-template.md
│       ├── checklist-template.md
│       ├── phr-template.prompt.md
│       ├── plan-template.md
│       ├── spec-template.md
│       └── tasks-template.md
│
├── docs/                             # MDX content files (Docusaurus)
│   ├── intro.md                     # ✅ COMPLETE - Homepage
│   │
│   ├── module-1-ros2/               # Module 1: ROS 2
│   │   ├── index.md                 # ✅ COMPLETE - Module overview
│   │   ├── 1-1-ros2-intro.md       # ✅ COMPLETE - Full chapter with code examples
│   │   ├── 1-2-nodes-topics.md     # ✅ STUB - Section headers ready
│   │   ├── 1-3-services-actions.md # ✅ STUB - Section headers ready
│   │   ├── 1-4-parameters.md       # ✅ STUB - Section headers ready
│   │   ├── 1-5-urdf.md             # ✅ STUB - Section headers ready
│   │   └── exercises/
│   │       └── module-1-exercises.md # ✅ STUB - 5 exercises outlined
│   │
│   ├── module-2-simulation/         # Module 2: Gazebo & Unity
│   │   ├── index.md                 # ✅ COMPLETE - Module overview
│   │   ├── 2-1-gazebo-intro.md     # ⏳ TODO - Stub needed
│   │   ├── 2-2-unity-intro.md      # ⏳ TODO - Stub needed
│   │   ├── 2-3-sensors.md          # ⏳ TODO - Stub needed
│   │   ├── 2-4-advanced.md         # ⏳ TODO - Stub needed
│   │   └── exercises/
│   │       └── module-2-exercises.md # ⏳ TODO - Stub needed
│   │
│   ├── module-3-isaac/              # Module 3: NVIDIA Isaac
│   │   ├── index.md                 # ✅ COMPLETE - Module overview
│   │   ├── 3-1-isaac-sim-intro.md  # ⏳ TODO - Stub needed
│   │   ├── 3-2-reinforcement-learning.md # ⏳ TODO - Stub needed
│   │   ├── 3-3-training-scenarios.md # ⏳ TODO - Stub needed
│   │   ├── 3-4-sim-to-real.md      # ⏳ TODO - Stub needed
│   │   └── exercises/
│   │       └── module-3-exercises.md # ⏳ TODO - Stub needed
│   │
│   ├── module-4-vla/                # Module 4: Vision-Language-Action
│   │   ├── index.md                 # ✅ COMPLETE - Module overview
│   │   ├── 4-1-vision.md           # ⏳ TODO - Stub needed
│   │   ├── 4-2-language.md         # ⏳ TODO - Stub needed
│   │   ├── 4-3-vla-models.md       # ⏳ TODO - Stub needed
│   │   ├── 4-4-deployment.md       # ⏳ TODO - Stub needed
│   │   └── exercises/
│   │       └── module-4-exercises.md # ⏳ TODO - Stub needed
│   │
│   ├── capstone/                    # Capstone Project
│   │   ├── index.md                 # ✅ COMPLETE - Project overview
│   │   ├── architecture-guide.md   # ⏳ TODO - Stub needed
│   │   ├── faq.md                  # ⏳ TODO - Stub needed
│   │   └── examples/               # ⏳ TODO - Example implementations
│   │       ├── vla-integration.md
│   │       ├── humanoid-rl.md
│   │       └── architectures.md
│   │
│   └── resources/                   # Appendices
│       ├── glossary.md             # ✅ COMPLETE - A-V technical terms
│       ├── hardware-specs.md       # ✅ COMPLETE - Hardware requirements
│       ├── software-setup.md       # ✅ COMPLETE - Installation guide
│       └── further-reading.md      # ✅ COMPLETE - Books, papers, resources
│
├── src/                             # React components and styles
│   ├── components/                  # Custom React components (future)
│   └── css/
│       └── custom.css              # ✅ COMPLETE - Theme styling
│
├── static/                          # Static assets
│   └── img/
│       ├── diagrams/               # Mermaid diagram exports
│       ├── hardware/               # Hardware photos
│       ├── logos/                  # Project logos
│       └── screenshots/            # UI screenshots
│
├── specs/                           # Feature specifications
│   └── 001-robotics-textbook-mdx/
│       ├── spec.md                 # ✅ Feature specification
│       ├── plan.md                 # ✅ Implementation plan
│       ├── tasks.md                # ✅ Task breakdown (235 tasks)
│       ├── research.md             # ✅ Research decisions
│       ├── quickstart.md           # ✅ Contributor guide
│       ├── checklists/
│       │   └── requirements.md     # ✅ Requirements checklist
│       └── contracts/
│           └── content-structure.md # ✅ Content standards
│
├── history/                         # Prompt History Records
│   └── prompts/
│       ├── 001-robotics-textbook-mdx/ # Feature-specific PHRs
│       └── constitution/           # Constitution PHRs
│
├── .gitignore                      # ✅ COMPLETE - Git ignore patterns
├── CLAUDE.md                       # ✅ COMPLETE - Agent context
├── package.json                    # ✅ COMPLETE - Node.js dependencies
├── docusaurus.config.js            # ✅ COMPLETE - Docusaurus config
├── sidebars.js                     # ✅ COMPLETE - Navigation structure
├── README.md                       # ✅ COMPLETE - Project documentation
└── FOLDER_STRUCTURE.md             # ✅ This file
```

## Created Files Summary

### Configuration Files (7)
- ✅ `.gitignore` - Git exclusions
- ✅ `package.json` - Node.js project config
- ✅ `docusaurus.config.js` - Site configuration
- ✅ `sidebars.js` - Navigation structure
- ✅ `src/css/custom.css` - Custom styling
- ✅ `README.md` - Project documentation
- ✅ `CLAUDE.md` - Agent instructions

### Content Files (16 total)
- ✅ **Homepage**: `docs/intro.md` (COMPLETE)
- ✅ **Module 1**: 7 files (1 complete chapter, 5 stubs, 1 exercises)
  - `index.md` (COMPLETE)
  - `1-1-ros2-intro.md` (COMPLETE - 288 lines with code examples)
  - `1-2-nodes-topics.md` (STUB)
  - `1-3-services-actions.md` (STUB)
  - `1-4-parameters.md` (STUB)
  - `1-5-urdf.md` (STUB)
  - `exercises/module-1-exercises.md` (STUB)
- ✅ **Module 2**: 1 file (index only)
  - `index.md` (COMPLETE)
- ✅ **Module 3**: 1 file (index only)
  - `index.md` (COMPLETE)
- ✅ **Module 4**: 1 file (index only)
  - `index.md` (COMPLETE)
- ✅ **Capstone**: 1 file (index only)
  - `index.md` (COMPLETE)
- ✅ **Resources**: 4 files (all complete)
  - `glossary.md` (COMPLETE)
  - `hardware-specs.md` (COMPLETE)
  - `software-setup.md` (COMPLETE)
  - `further-reading.md` (COMPLETE)

### Planning Artifacts (7)
- ✅ `.specify/memory/constitution.md` - Project principles
- ✅ `specs/001-robotics-textbook-mdx/spec.md` - Feature spec
- ✅ `specs/001-robotics-textbook-mdx/plan.md` - Implementation plan
- ✅ `specs/001-robotics-textbook-mdx/tasks.md` - Task breakdown
- ✅ `specs/001-robotics-textbook-mdx/research.md` - Research decisions
- ✅ `specs/001-robotics-textbook-mdx/quickstart.md` - Contributor guide
- ✅ `specs/001-robotics-textbook-mdx/contracts/content-structure.md` - Standards

## Phase Completion Status

### ✅ Phase 1: Project Setup & Initialization (T001-T014)
- [x] Initialize Node.js project
- [x] Configure Docusaurus
- [x] Setup navigation structure
- [x] Create custom styles
- [x] Setup directory structure

### ✅ Phase 2: Foundational Content Infrastructure (T015-T026)
- [x] Create homepage (intro.md)
- [x] Create Module 1 index
- [x] Create Module 2 index
- [x] Create Module 3 index
- [x] Create Module 4 index
- [x] Create Capstone index
- [x] Create all resource files (glossary, hardware, software, further reading)
- [x] Create Module 1 chapter stubs (1.1 complete, 1.2-1.5 stubs)
- [x] Create Module 1 exercises stub

### ⏳ Phase 3: User Story 1 - Module 1 MVP (T027-T076)
**Next Steps**: Create detailed content for:
- [ ] Complete chapters 1.2-1.5 with full content
- [ ] Add code examples to all chapters
- [ ] Complete exercises with solutions
- [ ] Add Mermaid diagrams

### ⏳ Phase 4-9: Remaining Modules and Deployment
To be completed in subsequent implementation phases.

## Navigation Structure

All files are properly linked in `sidebars.js`:
```javascript
tutorialSidebar: [
  'intro',                          // Homepage
  { Module 1: ROS 2 },             // 7 chapters + exercises
  { Module 2: Simulation },        // 4 chapters + exercises (stubs needed)
  { Module 3: NVIDIA Isaac },      // 4 chapters + exercises (stubs needed)
  { Module 4: VLA },               // 4 chapters + exercises (stubs needed)
  { Capstone Project },            // Project overview + guides (stubs needed)
  { Resources },                   // 4 appendices (all complete)
]
```

## Next Steps

1. **Install dependencies**: Run `npm install` to download Docusaurus packages
2. **Start dev server**: Run `npm run start` to preview site at http://localhost:3000
3. **Continue content**: Begin Phase 3 (User Story 1) to complete Module 1 detailed content
4. **Create chapter stubs**: Add stub files for Modules 2-4 chapters and capstone guides

## File Counts

- **Total directories**: 36
- **Configuration files**: 7
- **Content files (MDX)**: 16
- **Planning/spec files**: 20+
- **Total project files**: 50+ (excluding node_modules)

---

**Status**: Scaffold is complete and navigable. Ready for content generation in Phase 3.
