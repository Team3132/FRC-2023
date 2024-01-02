import type { LogLevels } from "./data/settings.slice";


/**
 * Interface for log messages
 * @example
 * 100.00 (Info) [Climber] the climber is climbing
 */
export interface Message {
  timestamp: number;
  severity: LogLevels;
  subsystem: string;
  content: string;
  id: string;
}

/**
 * Filters to content of a single message by it's content based on a search string.
 * @param message The log message object
 * @param filterString The search pattern to use
 * @returns true if the given message contains the given filter or false if the given message doesn't contain the search string.
 */
export const filterByMessageContent = (message: Message, filterString) => {
  var lowerCaseFilterString = filterString.toLowerCase();
  return message.content.toLowerCase().includes(lowerCaseFilterString);
};

/**
 * Filters the string by the active subsystems.
 *
 * @param subsystem Message from the websocket
 * @returns Boolean based on if active subsytem exists in message
 */
export const filterByActiveSubsystems = (
  message: Message,
  activeSubsystems,
) => {
  if (activeSubsystems.length !== 0) {
    return activeSubsystems
      .map((activeSubsystem) =>
        message.subsystem ? message.subsystem === activeSubsystem : false,
      )
      .some((item) => item);
  }
  return true;
};

/**
 * Gives a color based on the state of the websocket.
 * @returns Color value used by components.
 */
export const websocketStateColor = (websocketState) => {
  switch (websocketState) {
    case 'open':
      return 'green';
    case 'connecting':
      return 'yellow';
    case 'closed':
      return 'red';
    default:
      return undefined; // Defaults to the default button colour if none is defined.
  }
};

/**
 * Add and remove the subsystems from the active list depending on checkbox state.
 *
 * @param event The checkbox change event
 * @param subsystem The subsystem the checkbox belongs to
 */
export const setSubsystemActive = (
  event: React.ChangeEvent<HTMLInputElement>,
  subsystem: string | undefined,
  setActiveSubsystems,
) => {
  if (event.target.checked) {
    setActiveSubsystems((prevState) => [...prevState, subsystem]);
  } else {
    setActiveSubsystems((prevState) => [
      ...prevState.filter((item) => {
        return item !== subsystem;
      }),
    ]);
  }
};
