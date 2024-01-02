import { configureStore } from '@reduxjs/toolkit'
import { createSelectorHook, TypedUseSelectorHook, useDispatch, useSelector } from 'react-redux'
import { extendedApi } from './rtk.slice'
import { settingsSlice } from './settings.slice'
import { subsystemsSlice } from './subsystems.slice'

const store=  configureStore({
  reducer: {
    subsystems: subsystemsSlice.reducer,
    settings: settingsSlice.reducer,
    [extendedApi.reducerPath]: extendedApi.reducer
  },
  middleware: getDefaultMiddleware =>
    getDefaultMiddleware().concat(extendedApi.middleware)
})

export default store




export type RootState = ReturnType<typeof store.getState>
// Inferred type: {posts: PostsState, comments: CommentsState, users: UsersState}
export type AppDispatch = typeof store.dispatch
