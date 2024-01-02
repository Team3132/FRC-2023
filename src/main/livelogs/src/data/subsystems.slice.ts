import { createEntityAdapter, createSlice } from '@reduxjs/toolkit';
import type { RootState } from './store';

interface Subsystem {
  name: string;
  enabled: boolean;
}

export const subsystemsAdapter = createEntityAdapter<Subsystem>({
  selectId: (subsystem) => subsystem.name,
  sortComparer: (a, b) => a.name.localeCompare(b.name),
});

export const subsystemsSlice = createSlice({
  name: 'subsystems',
  initialState: subsystemsAdapter.getInitialState(),
  reducers: {
    setSubsystems: subsystemsAdapter.setAll,
    addSubsystem: subsystemsAdapter.addOne,
    upsertSubsystem: subsystemsAdapter.upsertOne,
    upsertSubsystems: subsystemsAdapter.upsertMany,
  },
});

export const {
  setSubsystems,
  addSubsystem,
  upsertSubsystem,
  upsertSubsystems,
} = subsystemsSlice.actions;

const subsystemSelectors = subsystemsAdapter.getSelectors<RootState>(
  (state) => state.subsystems,
);

export const selectAllSubsystems = subsystemSelectors.selectAll;
