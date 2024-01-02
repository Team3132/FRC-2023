import {
  Button,
  Checkbox,
  Drawer,
  DrawerBody,
  DrawerCloseButton,
  DrawerContent,
  DrawerFooter,
  DrawerHeader,
  DrawerOverlay,
  DrawerProps,
  FormControl,
  FormLabel,
  Input,
  Select,
  Stack,
} from '@chakra-ui/react';
import { useDispatch } from 'react-redux';
import { setLogLevel, LogLevels, setSearch } from '../data/settings.slice';
import {
  selectAllSubsystems,
  upsertSubsystem,
  upsertSubsystems,
} from '../data/subsystems.slice';
import { useAppSelector } from '../hooks/redux-hooks';

interface SettingsDrawerProps extends Omit<DrawerProps, 'children'> {}

export default function SettingsDrawer(props: SettingsDrawerProps) {
  const dispatch = useDispatch();

  /**
   * Subsystems
   */
  const subsystems = useAppSelector(selectAllSubsystems);

  /** Log Level */
  const loglevel = useAppSelector((s) => s.settings.loglevel);

  const searchString = useAppSelector((s) => s.settings.search);

  const allChecked = subsystems.every((s) => s.enabled);

  const isIndeterminate = !allChecked && subsystems.some((s) => s.enabled);

  return (
    <Drawer {...props}>
      <DrawerOverlay />
      <DrawerContent>
        <DrawerCloseButton />
        <DrawerHeader>Settings</DrawerHeader>
        <DrawerBody>
          <FormControl>
            <FormLabel htmlFor="loglevel">Log Level</FormLabel>
            <Select
              onChange={(e) => {
                dispatch(setLogLevel(parseInt(e.target.value) as LogLevels));
              }}
              w={'10em'}
              id="loglevel"
              value={loglevel}
            >
              <option value={LogLevels.Error}>Error</option>
              <option value={LogLevels.Warning}>Warning</option>
              <option value={LogLevels.Info}>Info</option>
              <option value={LogLevels.Debug}>Debug</option>
            </Select>
          </FormControl>
          <FormControl>
            <FormLabel htmlFor="email">Subsystems</FormLabel>
            <Checkbox
              isChecked={allChecked}
              isIndeterminate={isIndeterminate}
              onChange={(e) => {
                dispatch(
                  upsertSubsystems(
                    subsystems.map((subsystem) => ({
                      name: subsystem.name,
                      enabled: e.target.checked,
                    })),
                  ),
                );
              }}
            >
              All
            </Checkbox>
            <Stack pl={6} mt={1} spacing={1}>
              {subsystems.map((subsystem, index) => (
                <Checkbox
                  isChecked={subsystem.enabled}
                  onChange={(e) => {
                    dispatch(
                      upsertSubsystem({
                        ...subsystem,
                        enabled: e.target.checked,
                      }),
                    );
                  }}
                >
                  {subsystem.name}
                </Checkbox>
              ))}
            </Stack>
          </FormControl>
          <FormControl>
            <FormLabel htmlFor="search">Search</FormLabel>
            <Input
              id="search"
              placeholder="Search query"
              onChange={(e) => dispatch(setSearch(e.target.value))}
              value={searchString}
            />
          </FormControl>
        </DrawerBody>
        <DrawerFooter>
          <Button variant="outline" mr={3} onClick={props.onClose}>
            Cancel
          </Button>
        </DrawerFooter>
      </DrawerContent>
    </Drawer>
  );
}
