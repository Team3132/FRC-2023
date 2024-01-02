import { createSelector } from '@reduxjs/toolkit';
import { extendedApi } from './rtk.slice';
import type { RootState } from './store';
import { selectAllSubsystems } from './subsystems.slice';

const selectMessages = (state: RootState) =>
  extendedApi.endpoints.getNotifications.select()(state).data ?? [];

const selectSearch = (state: RootState) => state.settings.search;

const selectLogLevel = (state: RootState) => state.settings.loglevel;

const selectActiveSubsystem = (state: RootState) =>
  selectAllSubsystems(state).filter((subsystem) => subsystem.enabled);

// new selector that gets the data from the rtk.slice.ts file and filters and sort based on settings.slice.ts
export const selectFilteredMessages = createSelector(
  [selectMessages, selectSearch, selectLogLevel, selectActiveSubsystem],
  (messages, search, loglevel, subsystems) => {
    return messages
      .filter((message) => message.severity >= loglevel)
      .filter((message) => message.content.includes(search))
      .filter((message) =>
        subsystems.some((subsystem) => message.subsystem === subsystem.name),
      );
  },
);
