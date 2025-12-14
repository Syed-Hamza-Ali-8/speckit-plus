import { create } from 'zustand';
import { persist } from 'zustand/middleware';

/**
 * Theme options
 */
export type Theme = 'light' | 'dark' | 'system';

/**
 * UI state managed by Zustand
 */
export interface UIState {
  // Theme
  theme: Theme;
  setTheme: (theme: Theme) => void;

  // Sidebar
  sidebarOpen: boolean;
  toggleSidebar: () => void;
  setSidebarOpen: (open: boolean) => void;

  // Modals
  createTaskModalOpen: boolean;
  setCreateTaskModalOpen: (open: boolean) => void;
  editTaskModalOpen: boolean;
  editingTaskId: string | null;
  openEditTaskModal: (taskId: string) => void;
  closeEditTaskModal: () => void;
}

/**
 * UI Store with theme persistence
 * Theme is persisted to localStorage for cross-session persistence
 */
export const useUIStore = create<UIState>()(
  persist(
    (set) => ({
      // Theme state
      theme: 'system',
      setTheme: (theme) => set({ theme }),

      // Sidebar state
      sidebarOpen: true,
      toggleSidebar: () => set((state) => ({ sidebarOpen: !state.sidebarOpen })),
      setSidebarOpen: (open) => set({ sidebarOpen: open }),

      // Create task modal state
      createTaskModalOpen: false,
      setCreateTaskModalOpen: (open) => set({ createTaskModalOpen: open }),

      // Edit task modal state
      editTaskModalOpen: false,
      editingTaskId: null,
      openEditTaskModal: (taskId) =>
        set({ editTaskModalOpen: true, editingTaskId: taskId }),
      closeEditTaskModal: () =>
        set({ editTaskModalOpen: false, editingTaskId: null }),
    }),
    {
      name: 'ui-store', // localStorage key
      partialize: (state) => ({ theme: state.theme }), // Only persist theme
    }
  )
);
