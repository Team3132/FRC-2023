import { createSlice, PayloadAction } from '@reduxjs/toolkit';

export enum LogLevels {
  Debug,
  Info,
  Warning,
  Error,

  
 
}

// export type LogLevels = keyof typeof LogLevels;

export const settingsSlice = createSlice({
  name: 'settings',
  initialState: {
    autoScroll: true,
    search: '',
    loglevel: LogLevels.Debug,
  },
  reducers: {
    setAutoScroll: (state, action: PayloadAction<boolean>) => {
      state.autoScroll = action.payload;
    },
    toggleAutoScroll: (state) => {
      state.autoScroll = !state.autoScroll;
    },
    setSearch: (state, action: PayloadAction<string>) => {
      state.search = action.payload;
    },
    setLogLevel: (state, action: PayloadAction<LogLevels>) => {
      state.loglevel = action.payload;
    },
  },
});

export const { setAutoScroll, setSearch, setLogLevel, toggleAutoScroll } =
  settingsSlice.actions;
