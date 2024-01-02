import {
  BaseQueryFn,
  createApi,
  fetchBaseQuery,
} from '@reduxjs/toolkit/query/react';
import { uniqueId } from 'lodash';
import type { Message } from '../utils';
import { LogLevels } from './settings.slice';
import { addSubsystem, upsertSubsystem } from './subsystems.slice';

const host = window.location.hostname;

const emptyArrayBaseQuery = fetchBaseQuery({
  baseUrl: `http://${host}:5803`,
  prepareHeaders: (headers) => {
    headers.set('accept', 'application/json');
    return headers;
  },
});

const customBaseQuery: BaseQueryFn = async (
  args,
  { signal, dispatch, getState },
  extraOptions,
) => {
  return { data: [] };
};



export const extendedApi = createApi({
  baseQuery: customBaseQuery,
  endpoints: (builder) => ({
    getNotifications: builder.query<Message[], void>({
      query: () => '',
      async onCacheEntryAdded(
        arg,
        { updateCachedData, cacheDataLoaded, cacheEntryRemoved, dispatch },
      ) {
        // create a websocket connection when the cache subscription starts
        const ws = new WebSocket(`ws://${host}:5803`);
        try {
          // wait for the initial query to resolve before proceeding
          await cacheDataLoaded;

          // when data is received from the socket connection to the server,
          // update our query result with the received message
          const listener = (message: MessageEvent<any>) => {
            if (!message) return;
            var splitmsg = message.data.split(
              /^([0-9.]+) \((.+)\) \[([^]+)] (.*)/,
            );
            var timestamp: number = splitmsg[1];
            var textSeverity: string = splitmsg[2] ?? 'Error';
            const severity: LogLevels =
              textSeverity === 'Error'
                ? LogLevels.Error
                : textSeverity === 'Warning'
                ? LogLevels.Warning
                : textSeverity === 'Info'
                ? LogLevels.Info
                : LogLevels.Debug;
            var subsystem: string = splitmsg[3] ?? 'Unknown';
            var content: string = splitmsg[4] ?? message.data;
            var id = uniqueId();

            dispatch(
              addSubsystem({
                name: subsystem,
                enabled: true,
              }),
            );

            updateCachedData((draft) => {
              // Insert all received notifications from the websocket
              // into the existing RTKQ cache array
              draft.push({
                timestamp,
                severity,
                subsystem,
                content,
                id,
              });
            });
          };

          ws.addEventListener('message', listener);
        } catch {
          // no-op in case `cacheEntryRemoved` resolves before `cacheDataLoaded`,
          // in which case `cacheDataLoaded` will throw
        }
        // cacheEntryRemoved will resolve when the cache subscription is no longer active
        await cacheEntryRemoved;
        // perform cleanup steps once the `cacheEntryRemoved` promise resolves
        ws.close();
      },
    }),
  }),
});

export const { useGetNotificationsQuery } = extendedApi;
