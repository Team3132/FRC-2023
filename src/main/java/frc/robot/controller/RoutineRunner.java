package frc.robot.controller;



import frc.robot.Config;
import frc.robot.interfaces.LogHelper;
import frc.robot.lib.LEDColour;
import frc.robot.subsystems.Subsystems;
import java.util.Iterator;
import java.util.Optional;
import org.strongback.components.Clock;

/**
 * Handles the logic of starting, running and aborting a single Routine.
 * Used by the Controller to execute multiple routines in parallel.
 */
public class RoutineRunner implements LogHelper {
    protected final Routine routine;
    private Iterator<Step> iterator;
    protected RunStatus status = RunStatus.WAITING_TO_START;
    private Step desiredStep;
    private final Clock clock;
    private final Subsystems subsystems;
    private double stepStartTimeSec = 0;
    private double nextLogTimeSec = 0;
    private double timeBetweenLogsSec = 0.25;
    private String blockedBy = "";
    private boolean firstApplyStep = true;
    double armAngle = 0;
    double armExtension = 0;

    enum RunStatus {
        WAITING_TO_START, // Waiting for to be allowed to start
        RUNNING, // Running normally
        WAITING_TO_START_THEN_ABORT, // When routine is allowed to run, it should immediately abort
        ABORTING, // Waiting for current step to finish before applying the end state.
        ABORTED, // The end state is being applied
        FINISHED // The routine is done.
    }

    public RoutineRunner(Routine routine, Clock clock, Subsystems subsystems) {
        this.routine = routine;
        this.clock = clock;
        this.subsystems = subsystems;
        debug("Routine %s queued to start", routine.getName());

        // TODO: Find a less brittle way of doing this
        if (routine.getName() == "Start cube intaking"
                && subsystems.arm.getTargetAngle() == Config.arm.position.cube.intake.angle) {
            debug("Routine %s aborted early manually", routine.getName());
            status = RunStatus.WAITING_TO_START_THEN_ABORT;
        }
    }

    /**
     * Check if this routine conflicts with another routine by using the
     * any of the same domains/subsystems.
     * 
     * @param other the other Runner/Routine to check.
     * @return true if conflicts
     */
    synchronized public boolean doesConflict(RoutineRunner other) {
        if (this == other) {
            return false; // Can't conflict with itself
        }
        return routine.doesConflict(other.routine);
    }

    /**
     * Tells this runner that next time run() is called it can start executing the Routine.
     */
    synchronized public void start() throws Exception {
        switch (status) {
            case WAITING_TO_START:
                status = RunStatus.RUNNING;
                debug("Routine %s started", routine.getName());
                break;
            case WAITING_TO_START_THEN_ABORT:
                status = RunStatus.ABORTING;
                debug("Starting aborted routine %s", routine.getName());
                break;
            default:
                throw new Exception("Start called on routine " + routine.getName()
                        + " in unexpected state " + status);
        }
        iterator = routine.iterator();
        next();
    }

    /**
     * Move to the next step in the current routine or, if aborted,
     * return finished.
     * 
     * @return true if there is another step to be executed.
     */
    synchronized public boolean next() {
        if (!iterator.hasNext()) {
            debug("Routine %s is complete", routine.getName());
            status = RunStatus.FINISHED;
            return false;
        }
        switch (status) {
            case FINISHED:
                return false;
            case WAITING_TO_START:
            case WAITING_TO_START_THEN_ABORT:
                return true;
            case ABORTING:
                // The step that was running when the routine was aborted
                // has now finished applying. Apply the end state.
                status = RunStatus.ABORTED;
                setDesiredStep(routine.getEndState());
                return true;
            case ABORTED:
                // The end state has been fully applied. Change status to finished.
                status = RunStatus.FINISHED;
                return false;
            case RUNNING:
                // Normal running, move to the next step.
                setDesiredStep(iterator.next());
                break;
        }
        return true;
    }

    /**
     * Change the desired step and set the other tracking parameters.
     * 
     * @param step the step to change to.
     */
    private void setDesiredStep(Step step) {
        desiredStep = step;
        blockedBy = "";
        stepStartTimeSec = clock.currentTime();
        timeBetweenLogsSec = 0.25;
        nextLogTimeSec = stepStartTimeSec + timeBetweenLogsSec;
        firstApplyStep = true;
    }

    /**
     * Aborts the current routine by allowing the current step to apply and
     * then applies the routines end state.
     */
    synchronized public void abort() {
        switch (status) {
            case FINISHED:
            case ABORTING:
            case WAITING_TO_START_THEN_ABORT:
            case ABORTED:
                return;
            case WAITING_TO_START:
                // Needs to apply end state
                debug("Aborting queued routine %s", routine.getName());
                status = RunStatus.WAITING_TO_START_THEN_ABORT;
                return;
            case RUNNING:
                debug("Aborting routine %s", routine.getName());
                status = RunStatus.ABORTING;
                break;
        }
    }

    /**
     * run() attempts to apply the current step. If that succeeds it will move
     * to the next step so that can be applied next time.
     * 
     * @return true if the routine is still running.
     */
    synchronized public boolean run() {
        switch (status) {
            case WAITING_TO_START:
            case WAITING_TO_START_THEN_ABORT:
                return true;
            case FINISHED:
                return false;
            case RUNNING:
            case ABORTING:
            case ABORTED:
                try {
                    // Try to apply the current step to the subsystems and see if they all
                    // managed to apply it. Returns true if it was successful.
                    if (tryApplyStep()) {
                        // The current step was successfully applied, go to the next step.
                        subsystems.ledStrip.setAlliance();
                        // next() returns false if the routine is finished.
                        return next();
                    }
                } catch (Exception e) {
                    exception("Exception thrown trying to apply step", e);
                    return false; // Stop the processing of this routine.
                }
                break;
        }
        // A subsystem needs more time.
        return true;
    }

    /**
     * Does the simple, dumb and most importantly, safe thing.
     * Note: This years robot doesn't have any subsystems that can impact others,
     * so no logic is needed here to protect from dangerous steps.
     * 
     * Note if the step asks for something which will cause harm to the robot, the
     * request will be ignored. For example if the lift was moved into a position
     * the intake could hit it and then the intake was moved into the lift, the
     * intake move would be ignored.
     * 
     * This returns true if the step was completely applied, so it will need to
     * be repeatedly called until it returns true.
     * 
     * This relies on conflicting subsystems being part of the same domain so
     * that routines that use either of them will conflict and abort the previous routine.
     * 
     * @throws Exception Some unhandled case (null step etc) was experienced.
     * @return true if the step was successfully applied, false if more time is needed.
     */
    synchronized private boolean tryApplyStep() throws Exception {
        boolean isFirstApplyStep = firstApplyStep;
        firstApplyStep = false;

        if (status != RunStatus.RUNNING && status != RunStatus.ABORTING
                && status != RunStatus.ABORTED) {
            throw new Exception("RoutineRunner(" + routine.getName() + ") is in unexpected step "
                    + status.name());
        }

        if (desiredStep == null) {
            throw new Exception("desiredStep is null");
        }

        if (isFirstApplyStep) {
            armAngle = subsystems.arm.getAngle();
            armExtension = subsystems.arm.getExtension();
            debug("Applying requested step: %s", desiredStep);
            if (desiredStep.logString.isPresent()) {
                info("Step:");
                info("Step: %s ", desiredStep.logString);
                info("Step:");
            }
        }

        // First we tell the subsystems what they should do so they can do it in parallel before
        // checking that they have finished.

        // Start driving if necessary.
        if (desiredStep.drive.isPresent()) {
            subsystems.drivebase.setMode(desiredStep.drive.get());
        }
        if (desiredStep.currentPose.isPresent()) {
            subsystems.location.setCurrentPose(desiredStep.currentPose.get());
        }

        // Arm
        if (desiredStep.armAngle.isPresent()) {
            subsystems.arm.setTargetAngle(desiredStep.armAngle.get());
        } else if (desiredStep.armAngleDelta.isPresent()) {
            subsystems.arm.setTargetAngle(armAngle + desiredStep.armAngleDelta.get());
        }

        // Ensure we only call force calibration once
        if (desiredStep.forceCalibration.isPresent() && isFirstApplyStep) {
            subsystems.arm.forceCalibration();
        }

        if (desiredStep.armExtension.isPresent()) {
            subsystems.arm.setExtension(desiredStep.armExtension.get());
        } else if (desiredStep.armExtensionDelta.isPresent()) {
            subsystems.arm.setExtension(armExtension + desiredStep.armExtensionDelta.get());
        }

        // Intake
        if (desiredStep.intakeDutyCycle.isPresent()) {
            subsystems.intake.setDutyCycle(desiredStep.intakeDutyCycle.get());
        }

        // LED
        if (desiredStep.ledColour.isPresent()) {
            subsystems.ledStrip.setColour(desiredStep.ledColour.get());
        }

        // Gamepad Rumble
        if (desiredStep.gamepadRumbleIntensity.isPresent()) {
            this.subsystems.gamepad.setRumbleLeft(desiredStep.gamepadRumbleIntensity.get());
            this.subsystems.gamepad.setRumbleRight(desiredStep.gamepadRumbleIntensity.get());
        }

        // Check which subsystems have yet to finish applying the requested step.

        // Arm
        // FIXME: This could be misleading as both the angle and telescpe use the same isInPosition
        // method, and we have no way of knowing which one is actually blocking.
        if (notFinished(desiredStep.armAngle, subsystems.arm.isInPosition(),
                LEDColour.GREEN,
                "armAngle")) {
            return false;
        }

        if (notFinished(desiredStep.armExtension, subsystems.arm.isInPosition(), LEDColour.GREEN,
                "armExtension")) {
            return false;
        }

        // Drivebase
        if (notFinished(desiredStep.drive, subsystems.drivebase.hasFinished(),
                LEDColour.CYAN, "driving")) {
            if (isAborting()) {
                // A new routine has been started, give up on the drive routine.
                subsystems.drivebase.setArcadeDrive();
                return true; // Finished driving.
            }
            return false; // Still driving
        }

        // Last thing: wait for the delay time if it's set.
        // The time beyond which we are allowed to move onto the next step
        if (desiredStep.timeAction.isPresent()) {
            double endTime = desiredStep.timeAction.get().calculateEndTime(stepStartTimeSec);
            if (notFinished(desiredStep.timeAction, clock.currentTime() >= endTime,
                    LEDColour.WHITE, "time")) {
                return false; // Waiting for the required amount of time.
            }
        }

        blockedBy = "";
        return true; // Not waiting on anything, can move to the next step.
    }

    /**
     * Helper to check if a subsystem has finished applying a change.
     * 
     * @param expected if empty, then don't check this subsystem.
     * @param finished if the subsystem has finished applying any change
     * @param colour what colour to set the LED strip to if the subsystem hasn't finished
     * @param subsystem the name of the subsystem that is being checked
     * @return true if this subsystem is still applying the change and more time is needed
     */
    private <T> boolean notFinished(Optional<T> expected, boolean finished, LEDColour colour,
            String subsystem) {
        if (expected.isEmpty() || finished) {
            return false; // No change expected or finished applying change
        }
        // We're waiting for the subsystem to finish the requested update.

        // Set the LED colour to reflect what the robot is blocked by.
        // This won't work well if there are multiple routines running, but
        // if something is blocked for a long time, then the other routines
        // will finish and leave the LED strip alone.
        subsystems.ledStrip.setColour(colour);

        // More time is needed to apply this step.
        // Check to see if it should log a message of what it is blocked on.
        double now = clock.currentTime();
        if (now > nextLogTimeSec) {
            debug("Waiting on %s, has waited %.1f secs so far", subsystem,
                    now - stepStartTimeSec);
            timeBetweenLogsSec *= 2; // Wait longer and longer between updates.
            nextLogTimeSec = now + timeBetweenLogsSec;
        }
        // A subsystem needs more time, record it for the smart dashboard.
        blockedBy = subsystem;
        return true;
    }

    // This Routine has yet to be started, possibly because it's waiting for other
    // routines to finish running.
    synchronized public boolean isWaitingToStart() {
        return status == RunStatus.WAITING_TO_START
                || status == RunStatus.WAITING_TO_START_THEN_ABORT;
    }

    synchronized public boolean isRunning() {
        return status == RunStatus.RUNNING
                || status == RunStatus.ABORTING
                || status == RunStatus.ABORTED;
    }

    synchronized public boolean isAborting() {
        return status == RunStatus.ABORTING;
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }
        if (other instanceof RoutineRunner) {
            RoutineRunner runner = (RoutineRunner) other;
            return runner.routine == routine;
        }
        return false;
    }

    public String getBlockedBy() {
        return blockedBy;
    }

    @Override
    public String getName() {
        return routine.getName();
    }
}
