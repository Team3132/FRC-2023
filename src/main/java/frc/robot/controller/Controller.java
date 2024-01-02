package frc.robot.controller;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.DashboardUpdater;
import frc.robot.interfaces.LogHelper;
import frc.robot.subsystems.Subsystems;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Iterator;
import org.strongback.components.Clock;

/**
 * @formatter:off
 * 
 * The controller runs one or more routines in response to events (such as button
 * presses etc to change the state of the robot.
 * 
 * This allows higher level code to specify just the states that the robot needs to
 * pass through but it doesn't need to care how it gets there - this code will
 * ensure it gets there safely.
 * 
 * This is very similar to commands, with the differences to a command-based
 * approach being:
 *  - Unlike commands, the activation logic is concentrated in one place,
 *    making it much safer to add new functionality.
 *  - Every state doesn't need to be aware of every other state (much simpler).
 *  - Creating strings of routines is much simpler and shorter than commands.
 *  - Arbitrary combinations of parallel and sequential commands aren't supported,
 *    only a series of parallel operations.
 * 
 * @formatter:on
 */
public class Controller implements Runnable, DashboardUpdater, LogHelper {
    private final Subsystems subsystems;
    private final Clock clock;
    private boolean isAlive = true; // For unit tests
    private boolean enabled = false;

    // The routines that are either running or waiting to run.
    private ArrayList<RoutineRunner> runners = new ArrayList<>();

    public Controller(Subsystems subsystems) {
        this.subsystems = subsystems;
        this.clock = subsystems.clock;
        (new Thread(this)).start();
    }

    synchronized public void run(Routine routine) {
        if (!enabled) {
            return;
        }
        for (RoutineRunner existing : runners) {
            if (existing.routine == routine) {
                // This routine is already running, don't start it again.
                return;
            }
        }
        RoutineRunner runner = new RoutineRunner(routine, clock, subsystems);
        // Check to see if this new routine conflicts with any existing running
        // routines.
        for (RoutineRunner existing : runners) {
            if (existing.doesConflict(runner)) {
                // These two routines will conflict. Abort the existing one
                // so the new one can start.
                existing.abort();
            }
        }
        runners.add(runner);
    }

    /**
     * Main entry point which processes each Runner/Routine is run.
     * 
     * Runs in its own thread.
     */
    @Override
    public void run() {
        try {
            while (true) {
                synchronized (this) {
                    // Check for any routines that can now be run.
                    // Do them in order that they were added.
                    for (RoutineRunner runner : runners) {
                        if (!runner.isWaitingToStart()) {
                            continue; // Already running, skip this runner.
                        }
                        // Check it against the already running routines
                        boolean canStart = true;
                        for (RoutineRunner other : runners) {
                            if (other.isRunning() && runner.doesConflict(other)) {
                                // These two routines will conflict, and the running one
                                // will have been told to abort, so wait until is finishes.
                                canStart = false;
                            }
                        }
                        if (canStart) {
                            // Doesn't conflict with any running routines, start it.
                            runner.start();
                        }
                    }
                    // Try to apply the current state for each running each routine.
                    Iterator<RoutineRunner> iter = runners.iterator();
                    while (iter.hasNext()) {
                        RoutineRunner runner = iter.next();
                        if (!runner.run()) {
                            // This runner/routine is done.
                            iter.remove();
                        }
                    }
                }
                // Sleep briefly so it don't use all of the cpu.
                // 10 ms sleep = 100 updates / second.
                clock.sleepMilliseconds(10);
            }
        } catch (Exception e) {
            // The controller is dying, write the exception to the logs.
            exception("Controller caught an unhandled exception", e);

            // Used by the unit tests to detect if the controller thread is still running
            // see isAlive()
            isAlive = false;
        }
    }

    /**
     * Disable running new routines and abort any currently running routines.
     */
    public synchronized void disable() {
        enabled = false;
        for (RoutineRunner runner : runners) {
            runner.abort();
        }
    }

    /**
     * Enable running new routines.
     */
    public synchronized void enable() {
        enabled = true;
    }

    /**
     * For use by unit tests only.
     * 
     * @return if an unhandled exception has occurred in the controller
     */
    public boolean isAlive() {
        return isAlive;
    }

    /**
     * Print the status on the smart dashboard
     */
    public synchronized void updateDashboard() {
        HashSet<String> names = new HashSet<>();
        HashSet<String> blockedBy = new HashSet<>();
        for (RoutineRunner runner : runners) {
            if (!runner.isWaitingToStart()) {
                names.add(runner.getName());
                blockedBy.add(runner.getBlockedBy());
            }
        }
        SmartDashboard.putString("Controller: Running routines", String.join(", ", names));
        SmartDashboard.putString("Controller: Blocked by", String.join(", ", blockedBy));
    }

    @Override
    public String getName() {
        return "Controller";
    }
}
