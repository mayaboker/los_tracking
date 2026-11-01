# Documentation diagrams

- In Mermaid sequence diagrams, never put a raw semicolon in message text.
  Mermaid treats semicolons as statement separators, which can produce a parse
  error from the remaining label text. Rephrase with a comma or "and", or use
  `#59;` when a literal semicolon is necessary.
- Avoid the bare word `end` in sequence-diagram labels; use "completion" or
  another phrase. Reserve standalone `end` for closing Mermaid blocks.
- When adding or changing a Mermaid diagram, validate it with a Mermaid parser
  or renderer, preferably the version used by the documentation preview. Balanced
  Markdown fences alone do not validate Mermaid syntax. If no renderer is
  available, report that limitation instead of claiming rendering was verified.
- Keep temporary renderer dependencies and generated verification artifacts
  outside the package unless the task explicitly requires committing them.

Syntax reference: [Mermaid sequence diagrams](https://mermaid.js.org/syntax/sequenceDiagram.html).
