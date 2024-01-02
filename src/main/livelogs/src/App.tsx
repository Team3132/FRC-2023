
import {

  Flex,

} from '@chakra-ui/react';
import React from 'react';

import Header from './components/Header';
import Messages from './components/Messages';
function App() {
  

  /**
   * The reusable row (containing a single line from the log) for virtuoso. Virtuoso dynamically fetches the data for every new Row of items as they are made visible. This row defines how said row should look and where the data should be fetched from.
   * @param index The index of the current displayed message
   * @returns UI message component to be displayed.
   */
  

  /**
   * Main render for the app.
   */
  return (
    <Flex h="100vh" direction="column">
      <Header />
      <Messages />
    </Flex>
  );
}

export default App;
