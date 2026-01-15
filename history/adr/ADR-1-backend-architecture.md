# ADR-1: RAG System Backend Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-21
- **Feature:** 3-book-embeddings
- **Context:** Need to implement a backend system that crawls the Physical AI & Humanoid Robotics textbook, extracts content, generates embeddings using Cohere, and stores them in Qdrant Cloud for semantic search capabilities. The system must be efficient, maintainable, and follow the Physical AI & Humanoid Robotics Book Constitution principles.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

### Technology Stack:
- Language: Python 3.11 (for ML/AI ecosystem compatibility)
- Package Manager: UV (fast, modern Python package manager)
- Web Crawling: requests + BeautifulSoup4 (efficient, lightweight)
- Embeddings: Cohere multilingual-22-12 model (high quality, multilingual support)
- Vector Database: Qdrant Cloud (scalable, reliable vector storage)
- Configuration: python-dotenv (secure environment variable management)

### Architecture Pattern:
- Single-file implementation (main.py) with function-based design
- Pipeline approach: crawl → extract → chunk → embed → store → verify
- Function components: get_all_urls, extract_text_from_urls, chunk_text, embed, create_collection, save_chunk_to_qdrant, main

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Fast development and deployment with minimal setup complexity
- Cohesive, integrated solution with well-defined interfaces
- Efficient processing pipeline with clear separation of concerns
- Cloud-native vector storage with automatic scaling
- Single-file approach reduces complexity for initial implementation
- Good performance with Python's rich ML/AI ecosystem

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Single-file approach may become unwieldy as features grow
- Dependency on external APIs (Cohere, Qdrant) creates potential failure points
- Cloud-based solution introduces potential costs at scale
- Rate limiting constraints on web crawling
- Vendor lock-in to specific services (Cohere, Qdrant)

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

### Alternative Stack A: Open-source approach
- Embeddings: Sentence Transformers (all-MiniLM-L6-v2) + local vector DB (FAISS)
- Tradeoffs: More complex setup, less reliable but no vendor lock-in

### Alternative Stack B: Modular approach
- Multiple files/modules instead of single file
- Tradeoffs: Better maintainability but more complex initial setup

### Alternative Stack C: Different vector database
- Pinecone or Weaviate instead of Qdrant
- Tradeoffs: Similar capabilities but different pricing and ecosystem

Why rejected: The chosen approach balances simplicity, performance, and reliability for the initial implementation while following the user's specific requirement for single-file implementation.

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: specs/3-book-embeddings/spec.md
- Implementation Plan: specs/3-book-embeddings/plan.md
- Related ADRs: None
- Evaluator Evidence: specs/3-book-embeddings/research.md