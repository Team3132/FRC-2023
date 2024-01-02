import type { Message } from '../utils';
import type * as CSS from 'csstype';
import { styled, Text, useColorMode } from '@chakra-ui/react';
import { LogLevels } from '../data/settings.slice';
import type { ItemContent } from 'react-virtuoso';

export default function ItemContent(index: number, data: Message, context: any) {
  const { colorMode } = useColorMode();
  const { id, timestamp, severity, subsystem, content } = data;
  var color: CSS.Property.Color;
  var shade = colorMode === 'dark' ? '300' : '500';

  switch (severity) {
    case LogLevels.Error:
      color = 'red';
      break;
    case LogLevels.Warning:
      color = 'yellow';
      break;
    case LogLevels.Info:
      color = 'blue';
      break;
    case LogLevels.Debug:
      color = 'green';
      break;
  }

  return (
    <Text textColor={`${color}.${shade}`} w="100%" id={id}>
      {timestamp} (
      {severity === LogLevels.Error
        ? 'Error'
        : severity === LogLevels.Warning
        ? 'Warning'
        : severity === LogLevels.Info
        ? 'Info'
        : severity === LogLevels.Debug
        ? 'Debug'
        : 'Unknown'}
      ) [{subsystem}] {content}
    </Text>
  );
}
