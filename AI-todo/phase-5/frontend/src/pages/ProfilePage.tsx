import { useState, useMemo } from 'react';
import { motion } from 'framer-motion';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import {
  User,
  Mail,
  Calendar,
  Edit2,
  Save,
  X,
  CheckCircle,
  Clock,
  ListTodo,
  TrendingUp,
} from 'lucide-react';
import { toast } from 'sonner';
import { cn } from '@/lib/utils';
import { GlassButton } from '@/components/ui/GlassButton';
import { Input } from '@/components/ui/input';
import { Label } from '@/components/ui/label';
import { Skeleton } from '@/components/ui/skeleton';
import { useGetCurrentUserQuery, useUpdateProfileMutation } from '@/services/userApi';
import { useGetTasksQuery } from '@/services/taskApi';
import { profileSchema, type ProfileFormData } from '@/lib/validations/profile';

export function ProfilePage() {
  const { data: user, isLoading: isUserLoading, error: userError } = useGetCurrentUserQuery();
  const { data: tasksData } = useGetTasksQuery({});
  const [updateProfile, { isLoading: isUpdating }] = useUpdateProfileMutation();
  const [isEditing, setIsEditing] = useState(false);

  const {
    register,
    handleSubmit,
    reset,
    formState: { errors },
  } = useForm<ProfileFormData>({
    resolver: zodResolver(profileSchema),
    defaultValues: {
      firstName: '',
      lastName: '',
    },
  });

  // Reset form when user data loads or when entering edit mode
  const handleEditClick = () => {
    // Split the name into first and last name for editing
    const nameParts = (user?.name || '').split(' ');
    const firstName = nameParts[0] || '';
    const lastName = nameParts.slice(1).join(' ') || '';

    reset({
      firstName,
      lastName,
    });
    setIsEditing(true);
  };

  const handleCancel = () => {
    reset();
    setIsEditing(false);
  };

  const onSubmit = async (data: ProfileFormData) => {
    try {
      // Combine first and last name into a single name field
      const fullName = [data.firstName, data.lastName].filter(Boolean).join(' ');
      await updateProfile({
        name: fullName || null,
      }).unwrap();
      toast.success('Profile updated successfully!');
      setIsEditing(false);
    } catch {
      toast.error('Failed to update profile');
    }
  };

  // Calculate task stats
  const stats = useMemo(() => {
    const tasks = tasksData?.items ?? [];
    const total = tasks.length;
    const completed = tasks.filter((t) => t.status === 'completed').length;
    const pending = tasks.filter((t) => t.status === 'pending').length;
    const completionRate = total > 0 ? Math.round((completed / total) * 100) : 0;
    return { total, completed, pending, completionRate };
  }, [tasksData?.items]);

  // Format date
  const formatDate = (dateString: string) => {
    return new Date(dateString).toLocaleDateString('en-US', {
      year: 'numeric',
      month: 'long',
      day: 'numeric',
    });
  };

  // Get initials for avatar
  const getInitials = (name?: string) => {
    if (!name) return 'U';
    return name
      .split(' ')
      .map((n) => n[0])
      .join('')
      .toUpperCase()
      .slice(0, 2);
  };

  if (userError) {
    return (
      <div className="flex items-center justify-center min-h-[60vh]">
        <div className="text-center">
          <p className="text-lg text-red-500">Failed to load profile</p>
          <GlassButton
            variant="outline"
            onClick={() => window.location.reload()}
            className="mt-4"
          >
            Retry
          </GlassButton>
        </div>
      </div>
    );
  }

  return (
    <div className="relative min-h-screen">
      {/* Background gradient orbs */}
      <div className="absolute inset-0 overflow-hidden pointer-events-none">
        <div
          className={cn(
            'absolute -top-40 -right-40 w-96 h-96',
            'bg-purple-500/20 dark:bg-purple-500/10',
            'rounded-full blur-3xl'
          )}
        />
        <div
          className={cn(
            'absolute bottom-0 -left-40 w-96 h-96',
            'bg-blue-500/20 dark:bg-blue-500/10',
            'rounded-full blur-3xl'
          )}
        />
      </div>

      <div className="relative p-4 sm:p-6 lg:p-8">
        <div className="max-w-4xl mx-auto space-y-6">
          {/* Page header */}
          <motion.div
            initial={{ opacity: 0, y: -20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3 }}
          >
            <h1 className="text-3xl font-bold">
              <span className="text-gradient">My Profile</span>
            </h1>
            <p className="text-sm text-gray-600 dark:text-gray-400 mt-1">
              Manage your account settings
            </p>
          </motion.div>

          {/* Profile Card */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3, delay: 0.1 }}
            className={cn(
              'p-6 rounded-2xl',
              'bg-white/70 dark:bg-gray-900/70',
              'backdrop-blur-xl',
              'border border-white/30 dark:border-white/10',
              'shadow-glass dark:shadow-glass-dark'
            )}
          >
            {isUserLoading ? (
              <div className="space-y-4">
                <div className="flex items-center gap-4">
                  <Skeleton className="w-20 h-20 rounded-full" />
                  <div className="space-y-2">
                    <Skeleton className="h-6 w-40" />
                    <Skeleton className="h-4 w-60" />
                  </div>
                </div>
              </div>
            ) : user ? (
              <div className="space-y-6">
                {/* Avatar and basic info */}
                <div className="flex items-start justify-between">
                  <div className="flex items-center gap-4">
                    {/* Avatar */}
                    <div
                      className={cn(
                        'w-20 h-20 rounded-full',
                        'bg-gradient-to-br from-purple-500 via-blue-500 to-indigo-600',
                        'flex items-center justify-center',
                        'text-2xl font-bold text-white',
                        'shadow-premium'
                      )}
                    >
                      {getInitials(user.name)}
                    </div>

                    {/* Name and email */}
                    <div>
                      <h2 className="text-xl font-semibold text-gray-900 dark:text-gray-100">
                        {user.name || 'User'}
                      </h2>
                      <div className="flex items-center gap-2 mt-1 text-sm text-gray-600 dark:text-gray-400">
                        <Mail className="w-4 h-4" />
                        {user.email}
                      </div>
                      <div className="flex items-center gap-2 mt-1 text-sm text-gray-600 dark:text-gray-400">
                        <Calendar className="w-4 h-4" />
                        Member since {formatDate(user.created_at)}
                      </div>
                    </div>
                  </div>

                  {/* Edit button */}
                  {!isEditing && (
                    <GlassButton
                      variant="outline"
                      size="sm"
                      onClick={handleEditClick}
                      leftIcon={<Edit2 className="w-4 h-4" />}
                    >
                      Edit
                    </GlassButton>
                  )}
                </div>

                {/* Edit form */}
                {isEditing && (
                  <motion.form
                    initial={{ opacity: 0, height: 0 }}
                    animate={{ opacity: 1, height: 'auto' }}
                    exit={{ opacity: 0, height: 0 }}
                    onSubmit={handleSubmit(onSubmit)}
                    className="space-y-4 pt-4 border-t border-white/20 dark:border-white/10"
                  >
                    <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
                      <div className="space-y-2">
                        <Label htmlFor="firstName">First Name</Label>
                        <Input
                          id="firstName"
                          placeholder="Enter your first name"
                          {...register('firstName')}
                          className={cn(errors.firstName && 'border-red-500')}
                        />
                        {errors.firstName && (
                          <p className="text-sm text-red-500">{errors.firstName.message}</p>
                        )}
                      </div>
                      <div className="space-y-2">
                        <Label htmlFor="lastName">Last Name</Label>
                        <Input
                          id="lastName"
                          placeholder="Enter your last name"
                          {...register('lastName')}
                          className={cn(errors.lastName && 'border-red-500')}
                        />
                        {errors.lastName && (
                          <p className="text-sm text-red-500">{errors.lastName.message}</p>
                        )}
                      </div>
                    </div>

                    <div className="flex items-center gap-2 pt-2">
                      <GlassButton
                        type="submit"
                        variant="premium"
                        size="sm"
                        disabled={isUpdating}
                        leftIcon={<Save className="w-4 h-4" />}
                      >
                        {isUpdating ? 'Saving...' : 'Save Changes'}
                      </GlassButton>
                      <GlassButton
                        type="button"
                        variant="ghost"
                        size="sm"
                        onClick={handleCancel}
                        leftIcon={<X className="w-4 h-4" />}
                      >
                        Cancel
                      </GlassButton>
                    </div>
                  </motion.form>
                )}
              </div>
            ) : null}
          </motion.div>

          {/* Task Statistics */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3, delay: 0.2 }}
          >
            <h3 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Task Statistics</h3>
            <div className="grid grid-cols-2 sm:grid-cols-4 gap-4">
              {[
                {
                  label: 'Total Tasks',
                  value: stats.total,
                  icon: ListTodo,
                  gradient: 'from-purple-500 to-indigo-500',
                },
                {
                  label: 'Completed',
                  value: stats.completed,
                  icon: CheckCircle,
                  gradient: 'from-emerald-500 to-green-500',
                },
                {
                  label: 'Pending',
                  value: stats.pending,
                  icon: Clock,
                  gradient: 'from-amber-500 to-orange-500',
                },
                {
                  label: 'Completion Rate',
                  value: `${stats.completionRate}%`,
                  icon: TrendingUp,
                  gradient: 'from-blue-500 to-cyan-500',
                },
              ].map((stat, index) => (
                <motion.div
                  key={stat.label}
                  initial={{ opacity: 0, scale: 0.9 }}
                  animate={{ opacity: 1, scale: 1 }}
                  transition={{ delay: 0.2 + index * 0.05 }}
                  className={cn(
                    'p-4 rounded-2xl',
                    'bg-white/70 dark:bg-gray-900/70',
                    'backdrop-blur-xl',
                    'border border-white/30 dark:border-white/10',
                    'shadow-glass dark:shadow-glass-dark'
                  )}
                >
                  <div className="flex items-center gap-3">
                    <div
                      className={cn(
                        'w-10 h-10 rounded-xl',
                        'flex items-center justify-center',
                        `bg-gradient-to-br ${stat.gradient}`
                      )}
                    >
                      <stat.icon className="w-5 h-5 text-white" />
                    </div>
                    <div>
                      <p className="text-2xl font-bold text-gray-900 dark:text-gray-100">{stat.value}</p>
                      <p className="text-xs text-gray-600 dark:text-gray-400">{stat.label}</p>
                    </div>
                  </div>
                </motion.div>
              ))}
            </div>
          </motion.div>

          {/* Account Info */}
          <motion.div
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.3, delay: 0.3 }}
            className={cn(
              'p-6 rounded-2xl',
              'bg-white/70 dark:bg-gray-900/70',
              'backdrop-blur-xl',
              'border border-white/30 dark:border-white/10',
              'shadow-glass dark:shadow-glass-dark'
            )}
          >
            <h3 className="text-lg font-semibold text-gray-900 dark:text-gray-100 mb-4">Account Information</h3>
            <div className="space-y-3">
              <div className="flex items-center gap-3">
                <User className="w-4 h-4 text-gray-600 dark:text-gray-400" />
                <span className="text-sm text-gray-600 dark:text-gray-400">User ID:</span>
                <code className="text-xs bg-muted px-2 py-1 rounded">
                  {user?.id || '---'}
                </code>
              </div>
              <div className="flex items-center gap-3">
                <span
                  className={cn(
                    'w-2 h-2 rounded-full',
                    user?.is_active ? 'bg-green-500' : 'bg-red-500'
                  )}
                />
                <span className="text-sm text-gray-600 dark:text-gray-400">Status:</span>
                <span className="text-sm font-medium text-gray-900 dark:text-gray-100">
                  {user?.is_active ? 'Active' : 'Inactive'}
                </span>
              </div>
            </div>
          </motion.div>
        </div>
      </div>
    </div>
  );
}
