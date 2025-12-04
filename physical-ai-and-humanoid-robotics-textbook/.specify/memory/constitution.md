# Physical AI & Humanoid Robotics: Project Constitution

## 1. Project Vision & Governance
*Mission:* Create an embodied intelligence textbook where "Digital Code meets Physical Reality".
*Official Requirement:* Strictly follow the Panaversity Hackathon I guidelines.

*Core Governance:*
- *Protocol-First Development:* Development strictly follows sp.specify → sp.plan → sp.implement.
- *Minimal Viable Architecture (MVA):* Focus on Core Deliverables first (Book + Chatbot), then Bonus Features (Auth).

## 2. Technology Stack (Mandatory & Fixed)
To ensure compliance with Hackathon requirements, the following stack is enforced:
- *Presentation:* Docusaurus 3.9 (Classic) + React.
- *Backend Logic:* Python 3.10+, FastAPI.
- *AI Brain:* OpenAI Agents SDK (Logic) + Gemini Flash (via Router for Code Gen).
- *Vector Database:* Qdrant Cloud (For RAG/Embeddings).
- *Relational Database:* *Neon Serverless Postgres* (Mandatory for Chat Logs/User Data).
- *Deployment:* GitHub Pages.

## 3. The "Autonomous Ops" Strategy (Bonus Objectives)
Instead of manual scripts, we build *Reusable Agent Skills* located in skills/.
* **Skill A: skills/librarian.py** – Automated ingestion to Qdrant.
* **Skill B: skills/publisher.py** – Automated deployment to GitHub Pages.

## 4. Feature Roadmap (Prioritized)
*Phase 1: Core (Must Complete)*
- RAG Chatbot answering from the book.
- "Select Text" feature (Context-Aware API).

*Phase 2: Bonus (Target: 50+50 Points)*
- *Authentication:* Implement Better-Auth for User Signup/Login.
- *Personalization:* If User is logged in, show personalized content based on their hardware (as defined in user_profile table in Neon).

## 5. Content & Hardware Guidelines
- *Hardware Constraint:* The Developer (You) does not need special hardware.
- *Content Scope:* The Textbook Content must instruct students that they require NVIDIA RTX GPUs and Jetson Orin kits.
- *Quality:* Content must follow Flesch-Kincaid Grade 10-12 standards.

*Version: 1.0.3 (Official Compliant) | **Status*: Active