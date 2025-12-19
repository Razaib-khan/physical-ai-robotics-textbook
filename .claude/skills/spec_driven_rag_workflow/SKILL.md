# Skill: Spec-Driven RAG Workflow

## Metadata

- **Name**: spec_driven_rag_workflow
- **Version**: 1.0.0
- **Description**: Complete Spec-Driven Development workflow for RAG chatbot projects using Specify templates, slash commands, and systematic planning

## Role/Persona

You are an expert in Spec-Driven Development (SDD) for RAG applications. You specialize in:
- Using Specify framework (/sp.* commands) for systematic development
- Creating comprehensive specifications and plans before implementation
- Breaking down complex RAG projects into testable tasks
- Documenting architectural decisions and prompt history
- Maintaining project constitution and principles

## Trigger/Intent

Use this skill when the user requests:
- "Use spec-driven development for my RAG project"
- "Create a specification for RAG chatbot integration"
- "Plan RAG implementation using Specify"
- "Fix bugs in RAG system using spec approach"
- "Document RAG architecture decisions"
- Any request involving /sp.* commands for RAG projects

## Procedures

### Phase 1: Project Constitution

**Objective**: Establish project principles and coding standards.

**Command**: `/sp.constitution`

**Steps**:

1. **Capture Project Principles**
   - Code quality standards (Python/JavaScript)
   - Testing requirements
   - Performance goals (response time < 5s)
   - Security principles (API key management)
   - Architecture guidelines (RAG pattern)

2. **Document Active Technologies**
   - Backend: Python 3.12+, FastAPI, OpenAI Agents SDK, Cohere, Qdrant
   - Frontend: React 18, Docusaurus 3.5.2
   - AI Models: Gemini 2.0 Flash, Cohere embed-english-v3.0

3. **Save to Constitution File**
   - Location: `.specify/memory/constitution.md`
   - Update as project evolves

**Success Criteria**:
- ✅ Constitution file exists and is comprehensive
- ✅ All team members understand principles
- ✅ Active technologies are documented

### Phase 2: Feature Specification

**Objective**: Create detailed specification for RAG chatbot feature.

**Command**: `/sp.specify <feature-description>`

**Steps**:

1. **Provide Feature Description**
   ```
   /sp.specify "Fix RAG chatbot integration between frontend (Docusaurus),
   backend (FastAPI), and Qdrant vector database. Ensure proper API endpoint
   validation, input validation, timeout handling, and error messaging."
   ```

2. **Clarify Requirements** (if prompted)
   - Use `/sp.clarify` to answer clarification questions
   - Provide specific details on:
     - API endpoints and contracts
     - Error handling expectations
     - Performance requirements
     - Edge cases

3. **Review Generated Spec**
   - Location: `specs/<feature-name>/spec.md`
   - Contains:
     - User scenarios with priorities
     - Functional requirements
     - Success criteria
     - Assumptions and constraints
     - Out of scope items

**Success Criteria**:
- ✅ Spec file is complete with all sections filled
- ✅ User stories have clear acceptance scenarios
- ✅ Requirements are testable and measurable
- ✅ Edge cases are documented

### Phase 3: Implementation Planning

**Objective**: Create architecture and implementation plan.

**Command**: `/sp.plan`

**Steps**:

1. **Execute Planning Command**
   ```
   /sp.plan
   ```

2. **Review Generated Plan**
   - Location: `specs/<feature-name>/plan.md`
   - Contains:
     - Technical context (languages, dependencies, platform)
     - Constitution check (verify compliance)
     - Project structure
     - Complexity tracking

3. **Generate Research Document**
   - Location: `specs/<feature-name>/research.md`
   - Contains:
     - Bug analysis (for fix features)
     - API contract validation
     - Technology stack confirmation
     - Best practices research

4. **Create Supporting Artifacts**
   - `data-model.md`: Key entities (UserQuery, ChatMessage, EmbeddingVector)
   - `quickstart.md`: Testing procedures
   - `contracts/`: API contracts (YAML format)

**Success Criteria**:
- ✅ Plan passes constitution check
- ✅ Research identifies root causes (for bug fixes)
- ✅ API contracts are documented
- ✅ All design decisions are justified

### Phase 4: Task Breakdown

**Objective**: Generate dependency-ordered task list.

**Command**: `/sp.tasks`

**Steps**:

1. **Execute Tasks Command**
   ```
   /sp.tasks
   ```

2. **Review Generated Tasks**
   - Location: `specs/<feature-name>/tasks.md`
   - Task format: `[ID] [P?] [Story] Description`
   - Organized by:
     - Phase (Setup, Foundational, User Stories, Polish)
     - User story (US1, US2, US3)
     - Dependencies

3. **Understand Task Organization**
   - **Setup**: Project initialization (usually already complete)
   - **Foundational**: Blocking prerequisites (must complete first)
   - **User Stories**: Independent features (can parallelize)
   - **Polish**: Cross-cutting concerns (final validation)

4. **Identify Parallel Opportunities**
   - Tasks marked `[P]` can run in parallel
   - User stories can proceed independently after foundational phase

**Example Task List Structure**:
```
Phase 2: Foundational (Blocking Prerequisites)
- T001 Fix backend error response format (CRITICAL)
- T002 Import JSONResponse from fastapi.responses
- T003 Update error response to use JSONResponse

Phase 3: User Story 1 - End-to-End Query Processing (P1)
- T007 [US1] Fix frontend response handling
- T008 [US1] Remove array format handling
- T009 [US1] Update response parsing

Phase 4: User Story 2 - API Endpoint Validation (P2)
- T013 [P] [US2] Add input validation function
- T014 [P] [US2] Add state for validation errors
```

**Success Criteria**:
- ✅ All tasks have clear descriptions with file paths
- ✅ Dependencies are explicitly stated
- ✅ Tasks map to user stories
- ✅ Parallel opportunities are identified

### Phase 5: Implementation

**Objective**: Execute tasks systematically.

**Command**: `/sp.implement`

**Steps**:

1. **Execute Implementation Command**
   ```
   /sp.implement
   ```

2. **Follow Task Order**
   - Complete Foundational phase first (blocks everything)
   - Then proceed with user stories (in priority order or parallel)
   - Finish with Polish phase

3. **Test After Each Phase**
   - Foundational: Verify backend returns proper JSON
   - User Story 1: Test end-to-end query flow
   - User Story 2: Test input validation
   - User Story 3: Test error handling

4. **Commit After Logical Groups**
   - Use meaningful commit messages
   - Reference task IDs in commits

**Success Criteria**:
- ✅ All tasks are completed
- ✅ Each user story is independently functional
- ✅ Tests pass per quickstart.md
- ✅ Code follows constitution principles

### Phase 6: Documentation & History

**Objective**: Create Prompt History Records and ADRs.

**Commands**:
- Create PHR for each major interaction
- `/sp.adr <decision-title>` for architectural decisions

**Steps**:

1. **Automatic PHR Creation**
   - PHRs are created after every user prompt
   - Location: `history/prompts/<category>/`
   - Categories:
     - `constitution/`: Constitution changes
     - `<feature-name>/`: Feature-specific work
     - `general/`: General interactions

2. **PHR Contains**:
   - Full user prompt (verbatim)
   - Assistant response (representative)
   - Stage (spec, plan, tasks, implementation, etc.)
   - Files modified
   - Links to spec/ADR/PR

3. **Create ADRs for Significant Decisions**
   - Test for significance:
     - Long-term impact? (framework, data model, API)
     - Multiple alternatives considered?
     - Cross-cutting scope?
   - If ALL true, run: `/sp.adr <decision-title>`

4. **Example ADR Scenarios**:
   - Choosing FastAPI over Flask
   - Using Cohere embed-english-v3.0 vs alternatives
   - Implementing caching strategy
   - Selecting Qdrant over Pinecone

**Success Criteria**:
- ✅ PHRs exist for all major interactions
- ✅ PHRs contain complete prompts (not truncated)
- ✅ ADRs document significant architectural decisions
- ✅ All decisions have clear rationale

### Phase 7: Git Workflow

**Objective**: Commit work and create pull request.

**Command**: `/sp.git.commit_pr`

**Steps**:

1. **Execute Git Command**
   ```
   /sp.git.commit_pr
   ```

2. **Review Staged Changes**
   - Agent runs `git status` and `git diff`
   - Reviews all changes for commit

3. **Generate Commit Message**
   - Format: Conventional commits
   - Includes: feature context, scope, description
   - Example: `feat(rag-integration): fix backend response format and add input validation`

4. **Create Pull Request**
   - Agent uses `gh pr create`
   - PR description includes:
     - Summary of changes
     - Test plan
     - Links to spec and tasks

**Success Criteria**:
- ✅ Commit message is descriptive
- ✅ PR is created successfully
- ✅ PR description is comprehensive

## Examples

### Example 1: Starting a New RAG Feature

**User Request**: "I want to add a RAG chatbot to my documentation site using spec-driven development"

**Assistant Response**:
```
I'll guide you through the spec-driven approach for your RAG chatbot. Here's the workflow:

**Step 1: Establish Constitution**
Run: `/sp.constitution`
- Define code quality standards
- Document tech stack (FastAPI, Gemini, Cohere, Qdrant)
- Set performance goals (response time < 5s)

**Step 2: Create Feature Specification**
Run: `/sp.specify "Add RAG chatbot integration with Docusaurus frontend,
FastAPI backend, Gemini agent using OpenAI Agents SDK, Cohere embeddings,
and Qdrant vector database"`

This will create `specs/rag-chatbot/spec.md` with:
- User scenarios (self-study, error handling, etc.)
- Functional requirements
- Success criteria

**Step 3: Generate Implementation Plan**
Run: `/sp.plan`

Creates `specs/rag-chatbot/plan.md` with:
- Architecture decisions
- Technology choices and rationale
- API contracts

**Step 4: Break Down Tasks**
Run: `/sp.tasks`

Creates `specs/rag-chatbot/tasks.md` with:
- Ordered task list
- Dependencies
- Testing checkpoints

**Step 5: Implement**
Run: `/sp.implement`

Executes tasks systematically with automatic testing.

Ready to start with Step 1?
```

### Example 2: Fixing Bugs with Spec-Driven Approach

**User Request**: "Fix the bugs in my RAG backend using spec-driven development"

**Assistant Response**:
```
Let's fix your RAG backend bugs systematically:

**Step 1: Create Bug Fix Specification**
Run: `/sp.specify "Fix RAG chatbot integration bugs: API endpoint validation,
response format compatibility, input validation, timeout handling, and error messaging"`

**Step 2: Research & Bug Analysis**
Run: `/sp.plan`

This generates `specs/fix-rag-integration/research.md` analyzing:
- Issue 1: Backend response format mismatch
- Issue 2: Missing frontend input validation
- Issue 3: No request timeout
- Issue 4: Backend error response format
- Issue 5: Request validation in backend
- Issue 6: CORS configuration security

**Step 3: Generate Task Breakdown**
Run: `/sp.tasks`

Creates prioritized tasks:
- **Foundational** (T001-T006): Fix backend response formats (BLOCKING)
- **User Story 1** (T007-T012): End-to-end query processing
- **User Story 2** (T013-T020): Input validation
- **User Story 3** (T021-T029): Error handling

**Step 4: Implement Fixes**
Run: `/sp.implement`

Executes in order:
1. Foundational fixes first (backend returns proper JSON)
2. User stories independently (can parallelize)
3. Polish & validation

**Step 5: Create PR**
Run: `/sp.git.commit_pr`

Shall we start with the specification?
```

### Example 3: Documenting Architectural Decision

**User Request**: "Why did we choose Cohere over OpenAI embeddings?"

**Assistant Response**:
```
Great question! This is an architecturally significant decision. Let me document it:

Run: `/sp.adr choosing-cohere-embeddings-for-rag`

This creates `history/adr/XXXX-choosing-cohere-embeddings-for-rag.md` with:

**Decision**: Use Cohere embed-english-v3.0 instead of OpenAI text-embedding-ada-002

**Alternatives Considered**:
1. OpenAI text-embedding-ada-002 (1536 dimensions)
2. Cohere embed-english-v3.0 (1024 dimensions)
3. Open-source models (Sentence Transformers)

**Rationale**:
- **Cost**: Cohere is ~50% cheaper per token
- **Performance**: Similar quality for English text
- **Dimensions**: 1024 is sufficient for textbook content
- **API Reliability**: Both have comparable uptime

**Trade-offs**:
- ✅ Lower cost for high-volume queries
- ✅ Smaller vector size = faster search
- ❌ Locked into Cohere (migration requires re-embedding)
- ❌ Less ecosystem support than OpenAI

**Impact**: Long-term (affects all embeddings, migration costly)

**Status**: Accepted

This ADR ensures future team members understand the decision context.
```

## Self-Check/Validation

Before completing spec-driven workflow, verify:

### Constitution Phase
- [ ] `.specify/memory/constitution.md` exists
- [ ] All active technologies are documented
- [ ] Code standards are clear and specific
- [ ] Recent changes section is updated

### Specification Phase
- [ ] `specs/<feature-name>/spec.md` exists
- [ ] All user stories have acceptance scenarios
- [ ] Requirements are testable (measurable outcomes)
- [ ] Edge cases are documented
- [ ] Out of scope items are listed

### Planning Phase
- [ ] `specs/<feature-name>/plan.md` exists
- [ ] Constitution check passes (no violations)
- [ ] `research.md` identifies root causes (for bug fixes)
- [ ] `data-model.md` defines key entities
- [ ] `contracts/` contains API specifications
- [ ] All decisions have clear rationale

### Tasks Phase
- [ ] `specs/<feature-name>/tasks.md` exists
- [ ] Tasks are grouped by phase and user story
- [ ] Dependencies are explicitly stated
- [ ] Parallel opportunities identified ([P] marker)
- [ ] Each task has file paths referenced
- [ ] Checkpoints are defined

### Implementation Phase
- [ ] All tasks are completed in order
- [ ] Foundational phase completed first
- [ ] Each user story tested independently
- [ ] Code follows constitution principles
- [ ] Acceptance scenarios pass

### Documentation Phase
- [ ] PHRs exist in `history/prompts/`
- [ ] PHRs contain full user prompts
- [ ] ADRs created for significant decisions
- [ ] ADRs include alternatives and rationale
- [ ] Links between spec/tasks/ADR/PHR are correct

### Git Phase
- [ ] Changes are committed with meaningful messages
- [ ] PR is created with comprehensive description
- [ ] PR links to spec and tasks
- [ ] Test plan is included in PR

**If all checks pass**: ✅ Spec-driven workflow complete!

**If checks fail**: Review failed phase and use Examples section for guidance.

## Best Practices

1. **Always Start with Constitution**
   - Prevents scope creep
   - Ensures consistency
   - Documents constraints

2. **Clarify Before Planning**
   - Use `/sp.clarify` to resolve ambiguities
   - Better specs = better implementation
   - Ask about edge cases early

3. **Research Thoroughly**
   - For bug fixes: analyze root causes
   - For new features: validate assumptions
   - Document alternatives considered

4. **Break Down Systematically**
   - Foundational tasks block everything
   - User stories should be independent
   - Polish comes last

5. **Test at Checkpoints**
   - After Foundational: verify core fixes
   - After each User Story: test independently
   - After Polish: comprehensive validation

6. **Document Decisions**
   - Create ADRs for significant choices
   - Link ADRs to specs and tasks
   - Include rationale and trade-offs

7. **Use Context7 MCP for Documentation**
   - Get latest docs for libraries
   - Verify API compatibility
   - Check best practices

## Integration with docusaurus_rag_chatbot Skill

This skill (spec_driven_rag_workflow) **complements** the docusaurus_rag_chatbot skill:

- **Use docusaurus_rag_chatbot**: When user wants step-by-step technical implementation
- **Use spec_driven_rag_workflow**: When user wants systematic planning and documentation
- **Use both together**: For comprehensive RAG projects

**Example combined usage**:
1. User: "I want to build a RAG chatbot for my docs"
2. Start with **spec_driven_rag_workflow** to create specification
3. Use **docusaurus_rag_chatbot** for technical implementation
4. Return to **spec_driven_rag_workflow** for documentation and ADRs
