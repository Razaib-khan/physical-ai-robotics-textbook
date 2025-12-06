# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-robotics-textbook-mdx/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are NOT requested in feature specification. Focus on content validation via build process, link checking, and accessibility audits.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus documentation project structure:
- **Content files**: `docs/` directory (MDX format)
- **Static assets**: `static/img/` directory
- **Configuration**: Root level (`docusaurus.config.js`, `sidebars.js`, `package.json`)
- **Custom components**: `src/components/` directory
- **Custom styles**: `src/css/` directory

---

## Phase 1: Project Setup & Initialization

**Purpose**: Initialize Docusaurus project with all required tooling and configuration

**Checkpoint**: Docusaurus dev server runs successfully with basic homepage

- [x] T001 Initialize Node.js project with package.json in repository root
- [x] T002 Install Docusaurus 3.x with classic preset via npm install @docusaurus/core @docusaurus/preset-classic
- [x] T003 [P] Install search plugin via npm install @easyops-cn/docusaurus-search-local
- [x] T004 [P] Install math rendering plugins via npm install remark-math rehype-katex
- [x] T005 [P] Install Mermaid theme via npm install @docusaurus/theme-mermaid
- [x] T006 Configure docusaurus.config.js with site metadata, theme, plugins (search, KaTeX, Mermaid)
- [x] T007 Configure sidebars.js with initial navigation structure for all modules
- [x] T008 [P] Create src/css/custom.css for theme customization (light/dark mode styles)
- [x] T009 [P] Create basic homepage at docs/intro.md with course overview, objectives, prerequisites
- [x] T010 [P] Add KaTeX stylesheet CDN link to docusaurus.config.js stylesheets array
- [x] T011 [P] Create static/img/ directory structure (diagrams/, hardware/, screenshots/, logos/)
- [x] T012 [P] Create .gitignore file to exclude node_modules/, build/, .docusaurus/
- [ ] T013 Test local development server (npm run start) verifies site loads at localhost:3000
- [ ] T014 Test build process (npm run build) completes without errors

---

## Phase 2: Foundational Content Infrastructure

**Purpose**: Core documentation structure that MUST be complete before ANY module content

**⚠️ CRITICAL**: No module work can begin until this phase is complete

- [x] T015 Create docs/resources/ directory for appendices
- [x] T016 [P] Create docs/resources/glossary.md with frontmatter and initial structure (A-Z sections)
- [x] T017 [P] Create docs/resources/hardware-specs.md with frontmatter and table structure (budget/standard/premium tiers)
- [x] T018 [P] Create docs/resources/software-setup.md with frontmatter covering Ubuntu, ROS 2 Humble, Node.js installation
- [x] T019 [P] Create docs/resources/further-reading.md with frontmatter and curated reference sections
- [x] T020 Update sidebars.js to include resources category at end of navigation
- [x] T021 [P] Create module directory structure: docs/module-1-ros2/, docs/module-2-simulation/, docs/module-3-isaac/, docs/module-4-vla/, docs/capstone/
- [x] T022 [P] Create exercises subdirectories: docs/module-1-ros2/exercises/, docs/module-2-simulation/exercises/, docs/module-3-isaac/exercises/, docs/module-4-vla/exercises/
- [x] T023 Add README.md in repository root explaining project purpose, setup instructions, contribution guidelines
- [ ] T024 [P] Configure GitHub Actions workflow file .github/workflows/validate.yml for build validation
- [ ] T025 [P] Configure GitHub Actions workflow file .github/workflows/accessibility.yml for Lighthouse CI + axe-core
- [ ] T026 [P] Create lighthouserc.js configuration file with accessibility threshold (minScore: 0.9)

**Checkpoint**: Foundation ready - all directories exist, appendices have structure, CI/CD configured

---

## Phase 3: User Story 1 - Self-Study Learning Journey (Module 1: ROS 2) 🎯 MVP

**Goal**: Enable learners to self-study ROS 2 fundamentals through Module 1 with runnable code examples and exercises

**Independent Test**: A learner with no prior robotics experience can navigate Module 1, execute all code examples successfully (with documented dependencies), and complete exercises with clear validation methods

### Module 1 Structure & Navigation

- [x] T027 [US1] Create docs/module-1-ros2/index.md with frontmatter (module overview, 4-5 learning outcomes, prerequisites, 2-3 week duration)
- [x] T028 [US1] Update sidebars.js to add Module 1 category with link to index and placeholder chapters

### Module 1 Chapter 1: Introduction to ROS 2

- [x] T029 [P] [US1] Create docs/module-1-ros2/1-1-ros2-intro.md with frontmatter
- [x] T030 [US1] Write Chapter 1-1 content: ROS 2 overview, history, architecture (nodes/topics/services), use cases
- [x] T031 [US1] Add Mermaid diagram to Chapter 1-1: ROS 2 architecture showing nodes, topics, services relationships
- [x] T032 [US1] Add installation code block to Chapter 1-1: Ubuntu apt install commands for ROS 2 Humble with copy button
- [x] T033 [US1] Add environment setup code block to Chapter 1-1: source command and verification steps
- [x] T034 [US1] Add cross-reference links from Chapter 1-1 to glossary entries (Node, Topic, Service)

### Module 1 Chapter 2: Nodes and Topics

- [x] T035 [P] [US1] Create docs/module-1-ros2/1-2-nodes-topics.md with frontmatter
- [ ] T036 [US1] Write Chapter 1-2 content: nodes as processes, publisher/subscriber pattern, topic naming conventions
- [ ] T037 [US1] Add Mermaid sequence diagram to Chapter 1-2: message flow from publisher through topic to subscriber
- [ ] T038 [US1] Add Python code example to Chapter 1-2: minimal publisher (complete runnable code with dependencies)
- [ ] T039 [US1] Add Python code example to Chapter 1-2: minimal subscriber (complete runnable code)
- [ ] T040 [US1] Add code tabs to Chapter 1-2 comparing Python vs C++ node implementation
- [ ] T041 [US1] Add collapsible admonition to Chapter 1-2: advanced topic on Quality of Service (QoS) settings
- [ ] T042 [US1] Add hardware note callout to Chapter 1-2: simulation-only, no physical hardware required

### Module 1 Chapter 3: Services and Actions

- [ ] T043 [P] [US1] Create docs/module-1-ros2/1-3-services-actions.md with frontmatter
- [ ] T044 [US1] Write Chapter 1-3 content: synchronous services vs asynchronous actions, request/response patterns
- [ ] T045 [US1] Add Mermaid diagram to Chapter 1-3: service call flow showing client-server interaction
- [ ] T046 [US1] Add Python code example to Chapter 1-3: service server implementation with request handling
- [ ] T047 [US1] Add Python code example to Chapter 1-3: service client implementation with response processing
- [ ] T048 [US1] Add Python code example to Chapter 1-3: action server with feedback and goal handling
- [ ] T049 [US1] Document dependencies for Chapter 1-3 code examples: rclpy versions, std_srvs, action interfaces

### Module 1 Chapter 4: Parameters and Launch Files

- [ ] T050 [P] [US1] Create docs/module-1-ros2/1-4-parameters.md with frontmatter
- [ ] T051 [US1] Write Chapter 1-4 content: parameter declaration, parameter callbacks, launch file structure
- [ ] T052 [US1] Add Python code example to Chapter 1-4: node with declared parameters and get/set operations
- [ ] T053 [US1] Add XML code example to Chapter 1-4: launch file starting multiple nodes with parameters
- [ ] T054 [US1] Add YAML code example to Chapter 1-4: parameter configuration file structure
- [ ] T055 [US1] Add troubleshooting collapsible to Chapter 1-4: common parameter access errors and solutions

### Module 1 Chapter 5: URDF Robot Description

- [ ] T056 [P] [US1] Create docs/module-1-ros2/1-5-urdf.md with frontmatter
- [ ] T057 [US1] Write Chapter 1-5 content: URDF structure, links, joints, robot visualization
- [ ] T058 [US1] Add diagram placeholder to Chapter 1-5: robot kinematic chain with joints and links labeled
- [ ] T059 [US1] Add XML code example to Chapter 1-5: simple 2-link robot URDF with revolute joint
- [ ] T060 [US1] Add XML code example to Chapter 1-5: robot with visual and collision properties
- [ ] T061 [US1] Add bash code block to Chapter 1-5: robot_state_publisher and joint_state_publisher commands
- [ ] T062 [US1] Add hardware note to Chapter 1-5: URDF for simulation vs real robot differences

### Module 1 Exercises

- [ ] T063 [P] [US1] Create docs/module-1-ros2/exercises/module-1-exercises.md with frontmatter
- [ ] T064 [US1] Write Exercise 1 in module-1-exercises.md: Create temperature sensor publisher (beginner, 20-30 min, objective, instructions, validation)
- [ ] T065 [US1] Add Exercise 1 solution in collapsible details tag with explanation
- [ ] T066 [US1] Write Exercise 2: Create subscriber that logs received messages (beginner, 15-20 min)
- [ ] T067 [US1] Add Exercise 2 solution with explanation
- [ ] T068 [US1] Write Exercise 3: Implement simple service for addition (intermediate, 30-40 min)
- [ ] T069 [US1] Add Exercise 3 solution with explanation
- [ ] T070 [US1] Write Exercise 4: Create launch file for multi-node system (intermediate, 25-35 min)
- [ ] T071 [US1] Add Exercise 4 solution with explanation
- [ ] T072 [US1] Write Exercise 5: Design and implement simple robot URDF (advanced, 45-60 min)
- [ ] T073 [US1] Add Exercise 5 solution with explanation

### Module 1 Appendix Contributions

- [ ] T074 [P] [US1] Add ROS 2 terms to docs/resources/glossary.md: Node, Topic, Publisher, Subscriber, Service, Action, Parameter, URDF (10-15 entries with cross-references)
- [ ] T075 [P] [US1] Add ROS 2 Humble installation details to docs/resources/software-setup.md including apt repository setup
- [ ] T076 [P] [US1] Add ROS 2 references to docs/resources/further-reading.md: official docs, tutorials, community resources

**Checkpoint**: Module 1 (User Story 1) complete and independently testable - learner can work through all chapters and exercises

---

## Phase 4: User Story 2 - Progressive Module Mastery (Modules 2, 3, 4)

**Goal**: Enable learners to progress through simulation (Gazebo/Unity), NVIDIA Isaac, and VLA with clear cross-references to Module 1 concepts

**Independent Test**: A learner with Module 1 knowledge can complete any single module (2, 3, or 4) independently, with prerequisites clearly documented and cross-referenced

### Module 2: Gazebo & Unity (Digital Twin)

#### Module 2 Structure

- [ ] T077 [US2] Create docs/module-2-simulation/index.md with frontmatter (module overview, learning outcomes, prerequisites referencing Module 1, 2-3 week duration)
- [ ] T078 [US2] Update sidebars.js to add Module 2 category with chapters

#### Module 2 Chapter 1: Gazebo Introduction

- [ ] T079 [P] [US2] Create docs/module-2-simulation/2-1-gazebo-intro.md with frontmatter
- [ ] T080 [US2] Write Chapter 2-1 content: Gazebo overview, simulation benefits, world structure, model library
- [ ] T081 [US2] Add Mermaid diagram to Chapter 2-1: Gazebo architecture showing physics engine, rendering, sensors
- [ ] T082 [US2] Add XML code example to Chapter 2-1: simple Gazebo world file with ground plane and lighting
- [ ] T083 [US2] Add XML code example to Chapter 2-1: spawning robot from URDF in Gazebo world
- [ ] T084 [US2] Add bash code block to Chapter 2-1: launching Gazebo with ROS 2 integration
- [ ] T085 [US2] Add cross-reference links from Chapter 2-1 to Module 1 URDF chapter

#### Module 2 Chapter 2: Unity Introduction

- [ ] T086 [P] [US2] Create docs/module-2-simulation/2-2-unity-intro.md with frontmatter
- [ ] T087 [US2] Write Chapter 2-2 content: Unity for robotics, ROS-TCP connector, scene setup
- [ ] T088 [US2] Add code tabs to Chapter 2-2 comparing Gazebo world vs Unity scene setup approaches
- [ ] T089 [US2] Add C# code example to Chapter 2-2: Unity ROS-TCP connector setup script
- [ ] T090 [US2] Add C# code example to Chapter 2-2: publishing Unity transform data to ROS 2 topic
- [ ] T091 [US2] Add hardware note to Chapter 2-2: Unity system requirements (GPU, RAM) and free personal license info

#### Module 2 Chapter 3: Sensor Simulation

- [ ] T092 [P] [US2] Create docs/module-2-simulation/2-3-sensors.md with frontmatter
- [ ] T093 [US2] Write Chapter 2-3 content: simulated sensors (LiDAR, cameras, IMU), noise models, sensor data in ROS 2
- [ ] T094 [US2] Add diagram placeholder to Chapter 2-3: robot with sensor placement and field of view visualization
- [ ] T095 [US2] Add XML code example to Chapter 2-3: Gazebo camera sensor plugin configuration
- [ ] T096 [US2] Add XML code example to Chapter 2-3: Gazebo LiDAR sensor plugin with noise parameters
- [ ] T097 [US2] Add Python code example to Chapter 2-3: subscribing to sensor topics and processing data
- [ ] T098 [US2] Add collapsible for Chapter 2-3: advanced topic on sensor calibration in simulation

#### Module 2 Chapter 4: World Building

- [ ] T099 [P] [US2] Create docs/module-2-simulation/2-4-world-building.md with frontmatter
- [ ] T100 [US2] Write Chapter 2-4 content: creating environments, obstacles, terrain, lighting, physics properties
- [ ] T101 [US2] Add XML code example to Chapter 2-4: Gazebo world with multiple models and custom terrain
- [ ] T102 [US2] Add Unity scene hierarchy example to Chapter 2-4: GameObject structure for robot environment
- [ ] T103 [US2] Add hardware note to Chapter 2-4: simulation complexity vs real-time performance trade-offs

#### Module 2 Exercises

- [ ] T104 [P] [US2] Create docs/module-2-simulation/exercises/module-2-exercises.md with frontmatter
- [ ] T105 [US2] Write Exercise 1: Create Gazebo world with robot and obstacles (beginner, 30-40 min, with solution)
- [ ] T106 [US2] Write Exercise 2: Add camera sensor to robot and subscribe to image topic (intermediate, 35-45 min, with solution)
- [ ] T107 [US2] Write Exercise 3: Implement LiDAR-based obstacle detection (intermediate, 40-50 min, with solution)
- [ ] T108 [US2] Write Exercise 4: Create Unity scene with ROS 2 integration (advanced, 60-75 min, with solution)

#### Module 2 Appendix Contributions

- [ ] T109 [P] [US2] Add simulation terms to docs/resources/glossary.md: Gazebo, Unity, Sensor Plugin, Digital Twin, Physics Engine (8-10 entries)
- [ ] T110 [P] [US2] Add Gazebo and Unity installation to docs/resources/software-setup.md
- [ ] T111 [P] [US2] Add simulation references to docs/resources/further-reading.md

### Module 3: NVIDIA Isaac (AI-Robot Brain)

#### Module 3 Structure

- [ ] T112 [US2] Create docs/module-3-isaac/index.md with frontmatter (prerequisites referencing Modules 1-2, 3-4 week duration)
- [ ] T113 [US2] Update sidebars.js to add Module 3 category

#### Module 3 Chapter 1: Isaac Sim Introduction

- [ ] T114 [P] [US2] Create docs/module-3-isaac/3-1-isaac-sim-intro.md with frontmatter
- [ ] T115 [US2] Write Chapter 3-1 content: Isaac Sim overview, Omniverse, photorealistic simulation, GPU requirements
- [ ] T116 [US2] Add Mermaid diagram to Chapter 3-1: Isaac Sim architecture with Omniverse, physics, rendering, ROS 2 bridge
- [ ] T117 [US2] Add Python code example to Chapter 3-1: Isaac Sim basic scene setup and robot spawn
- [ ] T118 [US2] Add hardware note to Chapter 3-1: GPU requirements (NVIDIA RTX recommended), free educational license info
- [ ] T119 [US2] Add cross-references from Chapter 3-1 to Module 1 (ROS 2) and Module 2 (simulation concepts)

#### Module 3 Chapter 2: Reinforcement Learning Integration

- [ ] T120 [P] [US2] Create docs/module-3-isaac/3-2-reinforcement-learning.md with frontmatter
- [ ] T121 [US2] Write Chapter 3-2 content: RL basics, Isaac Gym integration, training environments, reward functions
- [ ] T122 [US2] Add Python code example to Chapter 3-2: setting up Isaac Gym environment for robot training
- [ ] T123 [US2] Add Python code example to Chapter 3-2: defining reward function for robot navigation task
- [ ] T124 [US2] Add collapsible for Chapter 3-2: advanced RL algorithms (PPO, SAC) with pseudocode
- [ ] T125 [US2] Add diagram placeholder to Chapter 3-2: RL training loop showing environment, agent, rewards

#### Module 3 Chapter 3: Training Scenarios

- [ ] T126 [P] [US2] Create docs/module-3-isaac/3-3-training-scenarios.md with frontmatter
- [ ] T127 [US2] Write Chapter 3-3 content: manipulation tasks, navigation, sim-to-real transfer, domain randomization
- [ ] T128 [US2] Add Python code example to Chapter 3-3: creating custom training scenario in Isaac Sim
- [ ] T129 [US2] Add Python code example to Chapter 3-3: implementing domain randomization for sim-to-real
- [ ] T130 [US2] Add hardware note to Chapter 3-3: training time estimates and GPU memory requirements

#### Module 3 Exercises

- [ ] T131 [P] [US2] Create docs/module-3-isaac/exercises/module-3-exercises.md with frontmatter
- [ ] T132 [US2] Write Exercise 1: Set up Isaac Sim environment and spawn robot (beginner, 45-60 min, with solution)
- [ ] T133 [US2] Write Exercise 2: Create simple navigation training scenario (intermediate, 60-75 min, with solution)
- [ ] T134 [US2] Write Exercise 3: Implement custom reward function for manipulation task (advanced, 75-90 min, with solution)

#### Module 3 Appendix Contributions

- [ ] T135 [P] [US2] Add Isaac terms to docs/resources/glossary.md: Isaac Sim, Omniverse, Isaac Gym, Reinforcement Learning, Domain Randomization (8-10 entries)
- [ ] T136 [P] [US2] Add NVIDIA Isaac Sim installation to docs/resources/software-setup.md including GPU driver setup
- [ ] T137 [P] [US2] Add Isaac and RL references to docs/resources/further-reading.md

### Module 4: Vision-Language-Action (VLA)

#### Module 4 Structure

- [ ] T138 [US2] Create docs/module-4-vla/index.md with frontmatter (prerequisites referencing Modules 1-3, 2-3 week duration)
- [ ] T139 [US2] Update sidebars.js to add Module 4 category

#### Module 4 Chapter 1: Vision Processing Pipelines

- [ ] T140 [P] [US2] Create docs/module-4-vla/4-1-vision-pipelines.md with frontmatter
- [ ] T141 [US2] Write Chapter 4-1 content: computer vision basics, object detection, semantic segmentation, depth estimation
- [ ] T142 [US2] Add Mermaid flowchart to Chapter 4-1: vision pipeline from camera input to detected objects
- [ ] T143 [US2] Add Python code example to Chapter 4-1: OpenCV image processing for robot vision (with cv2 dependencies)
- [ ] T144 [US2] Add Python code example to Chapter 4-1: using pre-trained YOLO model for object detection
- [ ] T145 [US2] Add hardware note to Chapter 4-1: camera requirements and processing power for real-time vision

#### Module 4 Chapter 2: Language Model Integration

- [ ] T146 [P] [US2] Create docs/module-4-vla/4-2-language-models.md with frontmatter
- [ ] T147 [US2] Write Chapter 4-2 content: LLM basics, prompt engineering, vision-language models, grounding language in robotics
- [ ] T148 [US2] Add diagram placeholder to Chapter 4-2: VLA architecture showing vision, language, and action modules
- [ ] T149 [US2] Add Python pseudocode to Chapter 4-2: integrating language model API for task interpretation (architectural guidance)
- [ ] T150 [US2] Add Python code example to Chapter 4-2: parsing natural language commands into robot actions
- [ ] T151 [US2] Add collapsible for Chapter 4-2: ethical considerations in language-grounded robotics

#### Module 4 Chapter 3: Action Coordination

- [ ] T152 [P] [US2] Create docs/module-4-vla/4-3-action-coordination.md with frontmatter
- [ ] T153 [US2] Write Chapter 4-3 content: VLA pipeline, multi-modal fusion, action planning, execution monitoring
- [ ] T154 [US2] Add Mermaid sequence diagram to Chapter 4-3: VLA execution flow from perception to action
- [ ] T155 [US2] Add Python code example to Chapter 4-3: coordinating vision and language inputs for manipulation
- [ ] T156 [US2] Add Python code example to Chapter 4-3: action execution with feedback and error recovery
- [ ] T157 [US2] Add cross-references from Chapter 4-3 to Modules 2 (sensors), 3 (RL), and 1 (actions/services)

#### Module 4 Exercises

- [ ] T158 [P] [US2] Create docs/module-4-vla/exercises/module-4-exercises.md with frontmatter
- [ ] T159 [US2] Write Exercise 1: Implement vision-based object detection pipeline (intermediate, 50-60 min, with solution)
- [ ] T160 [US2] Write Exercise 2: Create natural language command parser (intermediate, 45-55 min, with solution)
- [ ] T161 [US2] Write Exercise 3: Build complete VLA system for simple pick-and-place (advanced, 90-120 min, with solution)

#### Module 4 Appendix Contributions

- [ ] T162 [P] [US2] Add VLA terms to docs/resources/glossary.md: Vision-Language-Action, Multi-modal Fusion, Object Detection, Natural Language Processing (8-10 entries)
- [ ] T163 [P] [US2] Add vision library installation to docs/resources/software-setup.md (OpenCV, PyTorch for YOLO)
- [ ] T164 [P] [US2] Add VLA and computer vision references to docs/resources/further-reading.md

**Checkpoint**: Modules 2, 3, 4 (User Story 2) complete - learner can progress through all modules with clear prerequisites and cross-references

---

## Phase 5: User Story 3 - Capstone Project (Autonomous Humanoid)

**Goal**: Enable advanced learners to undertake capstone project integrating all module concepts

**Independent Test**: Learner with Modules 1-4 knowledge can complete capstone by following requirements, implementing components, and passing evaluation criteria

### Capstone Structure

- [ ] T165 [US3] Create docs/capstone/index.md with frontmatter (capstone overview, integration of all modules, 4-6 week duration)
- [ ] T166 [US3] Update sidebars.js to add Capstone category

### Capstone Content

- [ ] T167 [P] [US3] Create docs/capstone/requirements.md with frontmatter detailing project specifications
- [ ] T168 [US3] Write capstone requirements: autonomous humanoid system with perception, planning, and execution capabilities
- [ ] T169 [US3] Add required components list: ROS 2 integration, simulation in Gazebo/Isaac, vision system, language interface, action execution
- [ ] T170 [US3] Add technical specifications: minimum sensor requirements, computational requirements, performance targets

- [ ] T171 [P] [US3] Create docs/capstone/milestones.md with frontmatter for project checkpoints
- [ ] T172 [US3] Write Milestone 1: System architecture design and component identification (reference Module 1 concepts)
- [ ] T173 [US3] Write Milestone 2: Perception system implementation (reference Module 2 sensors, Module 4 vision)
- [ ] T174 [US3] Write Milestone 3: Decision-making and planning (reference Module 3 RL, Module 4 language)
- [ ] T175 [US3] Write Milestone 4: Action execution and integration (reference Module 1 actions, Module 2 simulation)
- [ ] T176 [US3] Write Milestone 5: Testing and validation (simulation and optional real hardware)

- [ ] T177 [P] [US3] Create docs/capstone/evaluation.md with frontmatter for grading rubric
- [ ] T178 [US3] Write evaluation criteria: architecture quality, code completeness, documentation, testing, innovation
- [ ] T179 [US3] Add evaluation rubric table with point allocations for each criterion
- [ ] T180 [US3] Add pass/fail thresholds and grade boundaries

- [ ] T181 [P] [US3] Create docs/capstone/troubleshooting.md with frontmatter
- [ ] T182 [US3] Write common issues section: integration problems, sensor failures, planning issues
- [ ] T183 [US3] Add troubleshooting guide with solutions for each common issue
- [ ] T184 [US3] Add cross-module integration patterns section referencing all Modules 1-4

### Capstone Appendix Contributions

- [ ] T185 [P] [US3] Add capstone-specific hardware specifications to docs/resources/hardware-specs.md (humanoid components, costs)
- [ ] T186 [P] [US3] Add integration testing guidance to docs/resources/software-setup.md

**Checkpoint**: Capstone (User Story 3) complete - learners have comprehensive project guide with milestones and evaluation criteria

---

## Phase 6: User Story 4 - Instructor Course Integration

**Goal**: Provide instructor resources for using textbook in formal education settings

**Independent Test**: Instructor can create syllabus mapping modules to course weeks, assign exercises with grading criteria, and facilitate capstone project

### Instructor Resources

- [ ] T187 [P] [US4] Add instructor notes section to docs/intro.md with course structure suggestions (15-week semester mapping)
- [ ] T188 [US4] Add suggested pacing guide to docs/intro.md: weeks per module, lab time allocation
- [ ] T189 [US4] Add assessment suggestions to docs/intro.md: exercise grading, capstone evaluation weight
- [ ] T190 [P] [US4] Enhance docs/resources/hardware-specs.md with institutional procurement section (bulk pricing, vendors, lab setup)
- [ ] T191 [US4] Add equipment list for class sizes (10, 20, 30 students) with budget estimates
- [ ] T192 [P] [US4] Add teaching tips to each module index.md: common student misconceptions, prerequisite review suggestions
- [ ] T193 [US4] Add exercise answer keys notation to exercise files (instructor access information)
- [ ] T194 [P] [US4] Add syllabus template to docs/resources/ as downloadable resource (course objectives, grading scheme, schedule)

### Instructor Appendix Contributions

- [ ] T195 [US4] Add instructor resources section to docs/resources/further-reading.md: teaching robotics, lab management resources

**Checkpoint**: Instructor resources (User Story 4) complete - educators have materials for course planning and student support

---

## Phase 7: Content Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple modules and ensuring overall quality

- [ ] T196 [P] Complete glossary entries: ensure all technical terms used 3+ times have definitions in docs/resources/glossary.md
- [ ] T197 [P] Validate all internal cross-reference links: run markdown-link-check on all docs/*.md files
- [ ] T198 [P] Add missing diagram placeholders: review all chapters for concepts needing visual support
- [ ] T199 [P] Complete hardware specifications: ensure all mentioned hardware in modules has entry in docs/resources/hardware-specs.md
- [ ] T200 [P] Verify code example dependencies: ensure all code blocks have documented versions and install commands
- [ ] T201 [P] Add alt text to all images: audit static/img/ directory and verify accessibility
- [ ] T202 Standardize code block titles: ensure consistent naming conventions across all modules
- [ ] T203 Verify collapsible section formatting: test all details/summary tags work correctly
- [ ] T204 Test code tabs: verify all Tabs components render correctly with synchronized selection
- [ ] T205 Validate math equations: verify all KaTeX equations render in both light and dark modes
- [ ] T206 [P] Review exercise difficulty progression: ensure beginner → intermediate → advanced flow within modules
- [ ] T207 [P] Complete docs/resources/software-setup.md: verify all dependencies and versions documented
- [ ] T208 [P] Expand docs/resources/further-reading.md: ensure comprehensive reference coverage
- [ ] T209 Update docs/intro.md: finalize homepage with complete module descriptions and navigation guidance
- [ ] T210 [P] Add navigation breadcrumbs verification: test sidebar navigation flow across all modules

---

## Phase 8: Build Validation & Testing

**Purpose**: Comprehensive testing before deployment

- [ ] T211 Run full Docusaurus build (npm run build) and fix any MDX parse errors
- [ ] T212 Verify search functionality: test local search plugin indexes all content correctly
- [ ] T213 Test sidebar navigation: verify all chapters appear in correct order with working links
- [ ] T214 Validate collapsibles: manually test all admonitions and details/summary sections expand correctly
- [ ] T215 Verify code tabs: test all Tabs components with multiple TabItems display and switch correctly
- [ ] T216 Test light/dark mode: verify all content, code blocks, diagrams readable in both themes
- [ ] T217 Check mobile responsiveness: test site on mobile viewport sizes (320px, 768px, 1024px)
- [ ] T218 Run Lighthouse accessibility audit: ensure score ≥ 90 (npm run build && npx @lhci/cli autorun)
- [ ] T219 Run axe-core accessibility check: verify WCAG 2.1 AA compliance (npx @axe-core/cli build/**/*.html)
- [ ] T220 Perform keyboard navigation test: verify all interactive elements accessible via Tab/Enter/Space
- [ ] T221 Run markdown link checker: verify no broken internal or external links (npx markdown-link-check docs/**/*.md)
- [ ] T222 Test copy-to-clipboard: verify copy buttons work on all code blocks
- [ ] T223 Verify Mermaid diagrams: ensure all diagrams render correctly in both themes
- [ ] T224 Test math rendering: verify all KaTeX equations display correctly
- [ ] T225 Check build performance: verify build completes in < 5 minutes
- [ ] T226 Test production server: run npm run serve and verify site functions correctly

---

## Phase 9: Deployment

**Purpose**: Deploy site to production hosting

- [ ] T227 Create .github/workflows/deploy.yml for GitHub Actions deployment workflow
- [ ] T228 Configure deployment workflow: build site, deploy to gh-pages branch
- [ ] T229 Configure GitHub Pages: enable Pages, set source to gh-pages branch, verify custom domain (if applicable)
- [ ] T230 Push all changes to main branch: git add, commit with descriptive message, push to origin
- [ ] T231 Trigger deployment: verify GitHub Actions workflow runs successfully
- [ ] T232 Verify deployed site: check GitHub Pages URL loads correctly with all content
- [ ] T233 Test deployed site functionality: verify search, navigation, code blocks, diagrams work on deployed version
- [ ] T234 [P] Document Vercel alternative: add instructions to README.md for deploying to Vercel as alternative
- [ ] T235 [P] Add deployment badge to README.md: link to deployed site with status badge

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Module 1 is MVP
- **User Story 2 (Phase 4)**: Depends on Foundational - Modules 2-4 can proceed after Foundational (references Module 1 but independently testable)
- **User Story 3 (Phase 5)**: Depends on User Stories 1 AND 2 - Capstone requires all module concepts
- **User Story 4 (Phase 6)**: Depends on User Stories 1-3 - Instructor resources reference complete curriculum
- **Polish (Phase 7)**: Depends on desired user stories being complete
- **Build & Test (Phase 8)**: Depends on all content phases
- **Deployment (Phase 9)**: Depends on Build & Test passing

### User Story Dependencies

- **User Story 1 (Module 1)**: Can start after Foundational - No dependencies on other stories ✅ MVP
- **User Story 2 (Modules 2-4)**: Can start after Foundational - Cross-references Module 1 but independently testable
- **User Story 3 (Capstone)**: Depends on User Stories 1 AND 2 - Integrates all module concepts
- **User Story 4 (Instructor)**: Depends on User Stories 1-3 - References complete content

### Within Each User Story

- Module index before chapters
- Chapters can be created in parallel (marked [P])
- Content within each chapter follows: frontmatter → content → code examples → exercises
- Exercises file created in parallel with chapters
- Appendix contributions can happen in parallel with chapter work

### Parallel Opportunities

- **Phase 1 (Setup)**: T003, T004, T005, T008, T009, T010, T011, T012 all parallel
- **Phase 2 (Foundational)**: T016, T017, T018, T019, T021, T022, T024, T025, T026 all parallel
- **Within User Story 1 (Module 1)**: Each chapter file creation parallel (T029, T035, T043, T050, T056, T063)
- **Within each chapter**: Code examples, diagrams, appendix contributions parallel where file dependencies don't exist
- **User Stories 2.1, 2.2, 2.3**: Modules 2, 3, 4 can work in parallel after Module 1 complete (different teams)
- **Phase 7 (Polish)**: Most tasks parallel (T196-T210)

---

## Parallel Example: User Story 1 (Module 1)

```bash
# After Foundational phase complete, these can run in parallel:

# Module structure
Task: "Create docs/module-1-ros2/index.md" (T027)

# All chapter files can be created simultaneously:
Task: "Create docs/module-1-ros2/1-1-ros2-intro.md" (T029)
Task: "Create docs/module-1-ros2/1-2-nodes-topics.md" (T035)
Task: "Create docs/module-1-ros2/1-3-services-actions.md" (T043)
Task: "Create docs/module-1-ros2/1-4-parameters.md" (T050)
Task: "Create docs/module-1-ros2/1-5-urdf.md" (T056)
Task: "Create docs/module-1-ros2/exercises/module-1-exercises.md" (T063)

# Appendix contributions parallel:
Task: "Add ROS 2 terms to glossary" (T074)
Task: "Add ROS 2 install to software-setup" (T075)
Task: "Add ROS 2 refs to further-reading" (T076)
```

---

## Implementation Strategy

### MVP First (Module 1 Only - User Story 1)

1. Complete Phase 1: Setup (T001-T014)
2. Complete Phase 2: Foundational (T015-T026)
3. Complete Phase 3: User Story 1 - Module 1 (T027-T076)
4. Complete Phase 7: Polish (subset for Module 1 only)
5. Complete Phase 8: Build & Test
6. **STOP and VALIDATE**: Test Module 1 independently - learner can complete self-study
7. Complete Phase 9: Deploy MVP (Module 1 only)

**MVP Success Criteria**:
- Module 1 accessible online
- All 5 chapters with runnable code examples
- 5 exercises with solutions
- Glossary entries for ROS 2 terms
- Search, navigation, themes working
- Accessibility score ≥ 90

### Incremental Delivery

1. **Release 1 (MVP)**: Module 1 + Resources (User Story 1)
2. **Release 2**: Add Module 2 (Simulation)
3. **Release 3**: Add Modules 3 & 4 (Isaac + VLA)
4. **Release 4**: Add Capstone Project (User Story 3)
5. **Release 5**: Add Instructor Resources (User Story 4)

Each release is independently valuable and testable.

### Parallel Team Strategy

With multiple content creators:

1. **Team completes Setup + Foundational together** (T001-T026)
2. **Once Foundational done**:
   - **Creator A**: Module 1 (T027-T076)
   - **Creator B**: Module 2 (T077-T111) - starts after Creator A reaches checkpoint
   - **Creator C**: Module 3 (T112-T137) - starts after Creator A reaches checkpoint
3. **After Modules 1-4 complete**:
   - **Creator D**: Capstone (T165-T186)
   - **Creator E**: Instructor Resources (T187-T195)
4. **All creators**: Polish, Build & Test, Deployment together

---

## Task Summary

**Total Tasks**: 235 tasks

**Task Count by Phase**:
- Phase 1 (Setup): 14 tasks
- Phase 2 (Foundational): 12 tasks
- Phase 3 (US1 - Module 1): 50 tasks
- Phase 4 (US2 - Modules 2-4): 88 tasks
  - Module 2: 35 tasks
  - Module 3: 26 tasks
  - Module 4: 27 tasks
- Phase 5 (US3 - Capstone): 22 tasks
- Phase 6 (US4 - Instructor): 9 tasks
- Phase 7 (Polish): 15 tasks
- Phase 8 (Build & Test): 16 tasks
- Phase 9 (Deployment): 9 tasks

**Parallel Opportunities**: 80+ tasks marked [P] can run in parallel

**MVP Scope** (Minimum Viable Product):
- Phases 1-3 only (User Story 1 - Module 1)
- 76 tasks total for MVP
- Estimated effort: 3-4 weeks for single creator, 1-2 weeks for team

**Independent Test Criteria**:
- **User Story 1**: Learner completes Module 1 self-study with working code examples
- **User Story 2**: Learner with Module 1 knowledge completes any Module 2-4 independently
- **User Story 3**: Learner with Modules 1-4 completes capstone project following requirements
- **User Story 4**: Instructor creates course syllabus and assigns exercises

---

## Format Validation

✅ **All tasks follow required checklist format**:
- Checkbox: `- [ ]`
- Task ID: T001-T235 (sequential)
- [P] marker: Present only on parallelizable tasks
- [Story] label: Present on User Story phase tasks (US1, US2, US3, US4)
- Description: Clear action with file path where applicable

**Example compliance check**:
- ✅ `- [ ] T027 [US1] Create docs/module-1-ros2/index.md with frontmatter`
- ✅ `- [ ] T029 [P] [US1] Create docs/module-1-ros2/1-1-ros2-intro.md with frontmatter`
- ✅ `- [ ] T074 [P] [US1] Add ROS 2 terms to docs/resources/glossary.md`
