import { configureStore } from '@reduxjs/toolkit';
import { api } from '@/services/api';

/**
 * Redux store configuration
 * Includes RTK Query middleware for API caching and invalidation
 */
export const store = configureStore({
  reducer: {
    // RTK Query API reducer
    [api.reducerPath]: api.reducer,
  },
  middleware: (getDefaultMiddleware) =>
    getDefaultMiddleware().concat(api.middleware),
});

// Infer the `RootState` and `AppDispatch` types from the store itself
export type RootState = ReturnType<typeof store.getState>;
export type AppDispatch = typeof store.dispatch;
