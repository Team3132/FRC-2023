import { ChevronDownIcon, MoonIcon, SunIcon, HamburgerIcon } from "@chakra-ui/icons";
import { Center, Flex, FlexProps, Heading, IconButton, Spacer, useColorMode, useDisclosure } from "@chakra-ui/react";
import { useRef } from "react";
import { useDispatch } from "react-redux";
import { setAutoScroll } from "../data/settings.slice";
import { useAppSelector } from "../hooks/redux-hooks";
import SettingsDrawer from "./SettingsDrawer";

export default function Header (props: FlexProps) {
    const dispatch = useDispatch();

  /**
   * Handle the opening and closing of the drawer.
   */
  const { isOpen, onOpen, onClose } = useDisclosure();

   /**
   * Button ref for the drawer component.
   */
   const btnRef = useRef();

   const autoscrollEnabled = useAppSelector(
    (state) => state.settings.autoScroll,
  );

  const { colorMode, toggleColorMode } = useColorMode();

    return <Flex {...props}>
    <Center pl={4}>
      <Heading size={'lg'}>Robot Logs</Heading>
    </Center>
    <Spacer />

    <SettingsDrawer
      isOpen={isOpen}
      placement="right"
      onClose={onClose}
      finalFocusRef={btnRef}
    />

    <IconButton
      aria-label="autoscroll"
      icon={<ChevronDownIcon />}
      onClick={() => dispatch(setAutoScroll(!autoscrollEnabled))}
      colorScheme={autoscrollEnabled ? 'blue' : undefined}
      m={2}
    />
    <IconButton
      aria-label="color mode"
      icon={colorMode === 'dark' ? <MoonIcon /> : <SunIcon />}
      onClick={toggleColorMode}
      m={2}
    />
    <IconButton
      aria-label="drawer"
      icon={<HamburgerIcon />}
      ref={btnRef}
      onClick={onOpen}
      m={2}
    />
  </Flex>
}