import { Link } from 'react-router-dom';
import { motion } from 'framer-motion';
import {
  CheckSquare,
  Sparkles,
  Zap,
  Shield,
  Clock,
  ArrowRight,
  Play,
  Star,
} from 'lucide-react';
import { GlassButton } from '@/components/ui/GlassButton';
import { GlassCard } from '@/components/ui/GlassCard';
import { ROUTES } from '@/routes';
import { cn } from '@/lib/utils';

const features = [
  {
    icon: Zap,
    title: 'Lightning Fast',
    description: 'Instant task creation and updates with real-time sync across all your devices.',
    gradient: 'from-amber-500 to-orange-500',
  },
  {
    icon: Shield,
    title: 'Secure by Design',
    description: 'Enterprise-grade security with end-to-end encryption for all your data.',
    gradient: 'from-emerald-500 to-green-500',
  },
  {
    icon: Clock,
    title: 'Time Tracking',
    description: 'Built-in time tracking to monitor your productivity and stay on schedule.',
    gradient: 'from-blue-500 to-cyan-500',
  },
  {
    icon: Sparkles,
    title: 'Smart Insights',
    description: 'AI-powered insights to help you work smarter, not harder.',
    gradient: 'from-purple-500 to-pink-500',
  },
];

const testimonials = [
  {
    name: 'Sarah Chen',
    role: 'Product Manager',
    avatar: 'S',
    content: 'TaskGPT transformed how our team manages projects. The UI is absolutely beautiful!',
    rating: 5,
  },
  {
    name: 'Marcus Rodriguez',
    role: 'Software Engineer',
    avatar: 'M',
    content: 'Finally, a task manager that looks as good as it works. Highly recommended.',
    rating: 5,
  },
  {
    name: 'Emily Watson',
    role: 'Startup Founder',
    avatar: 'E',
    content: 'The best productivity tool I have ever used. It just gets out of your way.',
    rating: 5,
  },
];

export function LandingPage() {
  return (
    <div className="relative min-h-screen overflow-hidden">
      {/* Background gradient orbs */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div
          className={cn(
            'absolute -top-40 -right-40 w-96 h-96',
            'bg-purple-500/30 dark:bg-purple-500/20',
            'rounded-full blur-3xl'
          )}
        />
        <div
          className={cn(
            'absolute top-1/2 -left-40 w-96 h-96',
            'bg-blue-500/30 dark:bg-blue-500/20',
            'rounded-full blur-3xl'
          )}
        />
        <div
          className={cn(
            'absolute -bottom-40 right-1/3 w-96 h-96',
            'bg-indigo-500/30 dark:bg-indigo-500/20',
            'rounded-full blur-3xl'
          )}
        />
      </div>

      {/* Hero Section */}
      <section className="relative pt-20 pb-32 px-4 sm:px-6 lg:px-8">
        <div className="max-w-7xl mx-auto">
          <div className="text-center">
            {/* Badge */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5 }}
              className="inline-flex items-center gap-2 mb-8"
            >
              <span
                className={cn(
                  'inline-flex items-center gap-2 px-4 py-2 rounded-full',
                  'bg-purple-500/10 dark:bg-purple-500/20',
                  'border border-purple-500/20 dark:border-purple-500/30',
                  'text-sm font-medium text-purple-600 dark:text-purple-400'
                )}
              >
                <Sparkles className="h-4 w-4" />
                Introducing TaskGPT 2.0
                <ArrowRight className="h-3 w-3" />
              </span>
            </motion.div>

            {/* Main headline */}
            <motion.h1
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: 0.1 }}
              className="text-5xl sm:text-6xl lg:text-7xl font-bold tracking-tight mb-6"
            >
              <span className="text-foreground">Manage tasks with </span>
              <span className="text-gradient">elegance</span>
            </motion.h1>

            {/* Subheadline */}
            <motion.p
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: 0.2 }}
              className="text-xl sm:text-2xl text-muted-foreground max-w-2xl mx-auto mb-10"
            >
              The modern task management app designed for individuals and teams who refuse to compromise on aesthetics.
            </motion.p>

            {/* CTA Buttons */}
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: 0.3 }}
              className="flex flex-col sm:flex-row items-center justify-center gap-4"
            >
              <Link to={ROUTES.REGISTER}>
                <GlassButton variant="premium" size="lg" glow rightIcon={<ArrowRight className="h-5 w-5" />}>
                  Start for free
                </GlassButton>
              </Link>
              <GlassButton variant="ghost" size="lg" leftIcon={<Play className="h-5 w-5" />}>
                Watch demo
              </GlassButton>
            </motion.div>

            {/* Social proof */}
            <motion.div
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ duration: 0.5, delay: 0.5 }}
              className="mt-12 flex items-center justify-center gap-4 text-sm text-muted-foreground"
            >
              <div className="flex -space-x-2">
                {['#8B5CF6', '#3B82F6', '#10B981', '#F59E0B'].map((color, i) => (
                  <div
                    key={i}
                    className="w-8 h-8 rounded-full border-2 border-background flex items-center justify-center text-white text-xs font-medium"
                    style={{ backgroundColor: color }}
                  >
                    {String.fromCharCode(65 + i)}
                  </div>
                ))}
              </div>
              <span>Trusted by 10,000+ users worldwide</span>
            </motion.div>
          </div>

          {/* Hero Image / Preview */}
          <motion.div
            initial={{ opacity: 0, y: 40 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.7, delay: 0.4 }}
            className="mt-20 relative"
          >
            <div className="absolute inset-0 bg-gradient-to-t from-background via-transparent to-transparent z-10 pointer-events-none" />
            <GlassCard variant="elevated" hover={false} padding="none" className="overflow-hidden">
              <div className="relative">
                {/* Mock app screenshot */}
                <div className="aspect-[16/9] bg-gradient-to-br from-gray-100 to-gray-200 dark:from-gray-800 dark:to-gray-900 p-4 sm:p-8">
                  {/* Mock header */}
                  <div className="flex items-center gap-4 mb-6">
                    <div className="flex gap-2">
                      <div className="w-3 h-3 rounded-full bg-red-500" />
                      <div className="w-3 h-3 rounded-full bg-amber-500" />
                      <div className="w-3 h-3 rounded-full bg-green-500" />
                    </div>
                    <div className="flex-1 h-8 bg-white/50 dark:bg-gray-700/50 rounded-lg" />
                  </div>

                  {/* Mock content */}
                  <div className="grid grid-cols-4 gap-4">
                    <div className="col-span-1 space-y-3">
                      {[1, 2, 3, 4].map((i) => (
                        <div
                          key={i}
                          className="h-10 bg-white/60 dark:bg-gray-700/60 rounded-lg"
                        />
                      ))}
                    </div>
                    <div className="col-span-3 space-y-3">
                      {[1, 2, 3, 4, 5].map((i) => (
                        <motion.div
                          key={i}
                          initial={{ opacity: 0, x: 20 }}
                          animate={{ opacity: 1, x: 0 }}
                          transition={{ delay: 0.5 + i * 0.1 }}
                          className={cn(
                            'h-16 rounded-xl p-3 flex items-center gap-3',
                            'bg-white/80 dark:bg-gray-800/80',
                            'border border-white/30 dark:border-white/10'
                          )}
                        >
                          <div
                            className={cn(
                              'w-5 h-5 rounded-md border-2',
                              i <= 2
                                ? 'bg-emerald-500 border-emerald-500'
                                : 'border-purple-400'
                            )}
                          />
                          <div className="flex-1">
                            <div className="h-3 bg-gray-300 dark:bg-gray-600 rounded w-3/4 mb-2" />
                            <div className="h-2 bg-gray-200 dark:bg-gray-700 rounded w-1/2" />
                          </div>
                          <div
                            className={cn(
                              'px-2 py-1 rounded-full text-xs',
                              i <= 2
                                ? 'bg-emerald-100 text-emerald-600 dark:bg-emerald-900/30 dark:text-emerald-400'
                                : 'bg-amber-100 text-amber-600 dark:bg-amber-900/30 dark:text-amber-400'
                            )}
                          >
                            {i <= 2 ? 'Done' : 'Todo'}
                          </div>
                        </motion.div>
                      ))}
                    </div>
                  </div>
                </div>
              </div>
            </GlassCard>
          </motion.div>
        </div>
      </section>

      {/* Features Section */}
      <section className="relative py-24 px-4 sm:px-6 lg:px-8">
        <div className="max-w-7xl mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl sm:text-4xl font-bold mb-4">
              Everything you need to <span className="text-gradient">stay productive</span>
            </h2>
            <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
              Powerful features wrapped in a beautiful interface. TaskGPT adapts to your workflow, not the other way around.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-6">
            {features.map((feature, index) => (
              <motion.div
                key={feature.title}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
              >
                <GlassCard variant="default" className="h-full">
                  <div
                    className={cn(
                      'w-12 h-12 rounded-xl flex items-center justify-center mb-4',
                      `bg-gradient-to-br ${feature.gradient}`,
                      'shadow-lg'
                    )}
                  >
                    <feature.icon className="h-6 w-6 text-white" />
                  </div>
                  <h3 className="text-lg font-semibold mb-2">{feature.title}</h3>
                  <p className="text-sm text-muted-foreground">{feature.description}</p>
                </GlassCard>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* Testimonials Section */}
      <section className="relative py-24 px-4 sm:px-6 lg:px-8">
        <div className="max-w-7xl mx-auto">
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            whileInView={{ opacity: 1, y: 0 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
            className="text-center mb-16"
          >
            <h2 className="text-3xl sm:text-4xl font-bold mb-4">
              Loved by <span className="text-gradient">thousands</span>
            </h2>
            <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
              See what our users have to say about their experience with TaskGPT.
            </p>
          </motion.div>

          <div className="grid grid-cols-1 md:grid-cols-3 gap-6">
            {testimonials.map((testimonial, index) => (
              <motion.div
                key={testimonial.name}
                initial={{ opacity: 0, y: 20 }}
                whileInView={{ opacity: 1, y: 0 }}
                viewport={{ once: true }}
                transition={{ duration: 0.5, delay: index * 0.1 }}
              >
                <GlassCard variant="default" className="h-full">
                  <div className="flex gap-1 mb-4">
                    {Array.from({ length: testimonial.rating }).map((_, i) => (
                      <Star key={i} className="h-4 w-4 fill-amber-400 text-amber-400" />
                    ))}
                  </div>
                  <p className="text-foreground mb-6">&quot;{testimonial.content}&quot;</p>
                  <div className="flex items-center gap-3">
                    <div
                      className={cn(
                        'w-10 h-10 rounded-full',
                        'bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600',
                        'flex items-center justify-center',
                        'text-white font-bold'
                      )}
                    >
                      {testimonial.avatar}
                    </div>
                    <div>
                      <div className="font-medium text-sm">{testimonial.name}</div>
                      <div className="text-xs text-muted-foreground">{testimonial.role}</div>
                    </div>
                  </div>
                </GlassCard>
              </motion.div>
            ))}
          </div>
        </div>
      </section>

      {/* CTA Section */}
      <section className="relative py-24 px-4 sm:px-6 lg:px-8">
        <div className="max-w-4xl mx-auto">
          <motion.div
            initial={{ opacity: 0, scale: 0.95 }}
            whileInView={{ opacity: 1, scale: 1 }}
            viewport={{ once: true }}
            transition={{ duration: 0.5 }}
          >
            <GlassCard variant="gradient" hover={false} padding="lg" className="text-center">
              <div className="relative">
                {/* Floating logo */}
                <motion.div
                  animate={{ y: [0, -10, 0] }}
                  transition={{ repeat: Infinity, duration: 3, ease: 'easeInOut' }}
                  className="inline-flex items-center justify-center w-16 h-16 rounded-2xl bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600 shadow-premium mb-6"
                >
                  <CheckSquare className="h-8 w-8 text-white" />
                </motion.div>

                <h2 className="text-3xl sm:text-4xl font-bold mb-4">
                  Ready to transform your productivity?
                </h2>
                <p className="text-lg text-muted-foreground mb-8 max-w-xl mx-auto">
                  Join thousands of professionals who have already made the switch. Start your free trial today.
                </p>
                <div className="flex flex-col sm:flex-row items-center justify-center gap-4">
                  <Link to={ROUTES.REGISTER}>
                    <GlassButton variant="premium" size="lg" glow>
                      Get started for free
                    </GlassButton>
                  </Link>
                  <Link to={ROUTES.LOGIN}>
                    <GlassButton variant="outline" size="lg">
                      Sign in
                    </GlassButton>
                  </Link>
                </div>
              </div>
            </GlassCard>
          </motion.div>
        </div>
      </section>

      {/* Footer */}
      <footer className="relative py-12 px-4 sm:px-6 lg:px-8 border-t border-white/10">
        <div className="max-w-7xl mx-auto">
          <div className="flex flex-col md:flex-row items-center justify-between gap-4">
            <div className="flex items-center gap-2">
              <div className="w-8 h-8 rounded-lg bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600 flex items-center justify-center">
                <CheckSquare className="h-4 w-4 text-white" />
              </div>
              <span className="font-bold">
                <span className="text-gradient">Task</span>Flow
              </span>
            </div>
            <p className="text-sm text-muted-foreground">
              &copy; {new Date().getFullYear()} TaskGPT. All rights reserved.
            </p>
          </div>
        </div>
      </footer>
    </div>
  );
}

export default LandingPage;
