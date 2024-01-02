import {
  useColorMode,
  Code,
  Center,
  Stack,
  Heading,
  Text,
} from '@chakra-ui/react';
import { useRef, useMemo } from 'react';
import { Virtuoso } from 'react-virtuoso';
import { useGetNotificationsQuery } from '../data/rtk.slice';
import { selectFilteredMessages } from '../data/selectors';
import { LogLevels } from '../data/settings.slice';
import { useAppDispatch, useAppSelector } from '../hooks/redux-hooks';
import ItemContent from './ItemContent';


export default function Messages() {
  const { data } = useGetNotificationsQuery();

  /**
   * The local storage hook to determine if autoscroll is enabled and store it's state.
   */
  const autoscrollEnabled = useAppSelector(
    (state) => state.settings.autoScroll,
  );

  /**
   * The reference so that the scroll location can be changed.
   */
  const virtuosoRef = useRef(null);

  const memoisedFilteredMessages = useMemo(() => selectFilteredMessages, []);

  const filteredMessages = useAppSelector(memoisedFilteredMessages);

  

  return (
    <Code w="100%" flex="1" colorScheme={'blackAlpha'}>
      <Virtuoso
        data={filteredMessages}
        itemContent={ItemContent}
        ref={virtuosoRef}
        followOutput={autoscrollEnabled ? 'smooth' : false}
        components={{
          EmptyPlaceholder: () => (
            <Center h="full" textAlign={'center'}>
              <Stack>
                <Heading textColor={'chakra-body-text'}>No messages</Heading>
                <Heading as="h5" size="sm" textColor={'chakra-body-text'}>
                  Maybe try to reload the page
                </Heading>
              </Stack>
            </Center>
          ),
        }}
      />
    </Code>
  );
}
