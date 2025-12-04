/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx,md,mdx}",
    "./blog/**/*.{js,jsx,ts,tsx,md,mdx}",
    "./docs/**/*.{js,jsx,ts,tsx,md,mdx}",
  ],
  theme: {
    extend: {
      fontSize: {
        'fluid-xs': 'clamp(0.75rem, 0.45vw + 0.6rem, 0.875rem)',
        'fluid-sm': 'clamp(0.875rem, 0.5vw + 0.725rem, 1rem)',
        'fluid-base': 'clamp(1rem, 0.55vw + 0.85rem, 1.125rem)',
        'fluid-lg': 'clamp(1.125rem, 0.6vw + 0.95rem, 1.25rem)',
        'fluid-xl': 'clamp(1.25rem, 0.7vw + 1.05rem, 1.5rem)',
        'fluid-2xl': 'clamp(1.5rem, 0.8vw + 1.25rem, 1.875rem)',
        'fluid-3xl': 'clamp(1.875rem, 1vw + 1.5rem, 2.25rem)',
        'fluid-4xl': 'clamp(2.25rem, 1.2vw + 1.75rem, 3rem)',
        'fluid-5xl': 'clamp(3rem, 1.5vw + 2.25rem, 4rem)',
        'fluid-6xl': 'clamp(3.75rem, 2vw + 2.75rem, 5rem)',
        'fluid-7xl': 'clamp(4.5rem, 2.5vw + 3.25rem, 6rem)',
        'fluid-8xl': 'clamp(6rem, 3vw + 4.5rem, 7rem)',
        'fluid-9xl': 'clamp(8rem, 4vw + 6rem, 9rem)',
      },
    },
  },
  plugins: [require('tailwindcss-text-balance')()],
}