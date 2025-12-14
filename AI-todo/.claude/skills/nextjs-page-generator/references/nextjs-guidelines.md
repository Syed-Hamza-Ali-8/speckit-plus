# Next.js Guidelines (Reference Doc)

## Use App Router
All pages must live inside /frontend/app/...

## Use Server Components by Default
- Add "use client" only when required.
- Forms and interactivity always require client components.

## TailwindCSS
Use classes like:
- flex
- gap-4
- p-4
- text-gray-700

## Better Auth
Protected pages require:
import { auth } from "@/lib/auth";
const session = await auth();
