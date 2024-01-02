package frc.robot.controller;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Config;
import frc.robot.drive.modes.Mode;
import frc.robot.drive.modes.Mode.Parameters;
import frc.robot.drive.modes.Mode.Type;
import frc.robot.lib.LEDColour;
import frc.robot.lib.TimeAction;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;

/**
 * A single step in a Routine. This step needs to be completely applied before the Routine can
 * move to the next step.
 * 
 * Examples:
 * - deploy the intake, which might take half a second to move.
 * - set the shooter wheel speed
 * - drive forward at half speed
 * - wait for 500 milliseconds
 */

public class Step {
    // Double and Boolean are used instead of double and boolean
    // so that null can be used to indicate that the step shouldn't
    // be changed and the current step be preserved.

    // Time
    // How we should/shouldn't delay at the end of this step
    public Optional<TimeAction> timeAction = Optional.empty();

    // Message to be logged when this is executed.
    public Optional<String> logString = Optional.empty();

    // Location
    public Optional<Pose2d> currentPose = Optional.empty();

    // Intake
    public Optional<Double> intakeDutyCycle = Optional.empty();

    // Driving
    public Optional<Parameters> drive = Optional.empty();

    // Arm
    public Optional<Double> armAngle = Optional.empty();
    public Optional<Double> armAngleDelta = Optional.empty();
    public Optional<Double> armExtension = Optional.empty();
    public Optional<Double> armExtensionDelta = Optional.empty();
    public Optional<Boolean> forceCalibration = Optional.empty();

    // LED strip
    public Optional<LEDColour> ledColour = Optional.empty();

    // Gamepad Rumble
    public Optional<Double> gamepadRumbleIntensity = Optional.empty();

    // Time
    /**
     * Set absolute time that the robot has to wait until.
     * Use this or setDelayDelta(), not both.
     * 
     * @param time measured in seconds, eg time_t.
     */
    public Step setDelayUntilTime(double time) {
        timeAction = Optional.of(new TimeAction(TimeAction.Type.DELAY_UNTIL, time));
        return this;
    }

    /**
     * Wait for delta seconds.
     * Use this or setDelayUntilTime(), not both.
     * 
     * @param seconds to apply to the current time.
     */
    public Step setDelayDelta(double seconds) {
        timeAction = Optional.of(new TimeAction(TimeAction.Type.DELAY_DELTA, seconds));
        return this;
    }

    /**
     * Set Status Message
     * 
     * @param Status to get as a string.
     */
    public Step setLog(String message) {
        logString = Optional.of(message);
        return this;
    }

    // Arm

    /** Set Arm Position in Degrees */
    public Step setArmAngle(double degrees) {
        armAngle = Optional.of(degrees);
        domains.add(Domain.ARM);
        return this;
    }

    public Step adjustArmAngle(double degrees) {
        armAngleDelta = Optional.of(degrees);
        domains.add(Domain.ARM);
        return this;
    }

    /** Set Telescope Position in Metres */
    public Step setArmExtension(double metres) {
        armExtension = Optional.of(metres);
        domains.add(Domain.ARM);
        return this;
    }

    public Step adjustArmExtension(double metres) {
        armExtensionDelta = Optional.of(metres);
        domains.add(Domain.ARM);
        return this;
    }

    /**
     * Tell the arm to calibrate
     */
    public Step forceArmCalibration() {
        forceCalibration = Optional.of(true);
        domains.add(Domain.ARM);
        return this;
    }

    // Location
    public Step setCurrentPosition(Pose2d pose) {
        currentPose = Optional.of(pose);
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    // Intake
    public Step setIntakeDutyCycle(double dutyCycle) {
        intakeDutyCycle = Optional.of(dutyCycle);
        domains.add(Domain.INTAKE);
        return this;
    }

    public Step setGamepadRumbleIntensity(double value) {
        gamepadRumbleIntensity = Optional.of(value);
        return this;
    }

    // LED strip
    public Step setColour(LEDColour c) {
        ledColour = Optional.of(c);
        domains.add(Domain.LED);
        return this;
    }

    // Drive base
    /**
     * Set the power levels on the drive base.
     * Used to drive the robot forward or backwards in a
     * "straight" line for the climb.
     */
    public Step setDrivebasePower(double power) {
        drive = Optional.of(Mode.getConstantPower(power));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    /**
     * Set the speed on the drive base.
     * Used to drive the robot forward or backwards in a
     * "straight" line for the L3 climb.
     */
    public Step setDrivebaseSpeed(double speed) {
        drive = Optional.of(Mode.getConstantSpeed(speed));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    public Step doDefaultDrive() {
        drive = Optional.of(new Parameters(Type.get(Config.drivebase.driveMode)));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    public Step doBalanceDrive(boolean forward) {
        drive = Optional.of(Mode.getBalance(forward));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    /**
     * Put the drive base in arcade drive mode using velocity control for the driver.
     */
    public Step doArcadeVelocityDrive() {
        drive = Optional.of(new Parameters(Type.ARCADE_VELOCITY));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    /**
     * Put the drive base in arcade drive mode for the driver.
     */
    public Step doArcadeDrive() {
        drive = Optional.of(new Parameters(Type.ARCADE_DUTY_CYCLE));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    /**
     * Put the drive base in DDR Pad drive mode for the driver.
     */
    public Step doDDRDrive() {
        drive = Optional.of(new Parameters(Type.DDRPAD_DRIVE));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    /**
     * Put the drive base in cheesy drive mode for the driver.
     */
    public Step doCheesyDrive() {
        drive = Optional.of(new Parameters(Type.CHEESY));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    public Step doPositionPIDArcade() {
        drive = Optional.of(new Parameters(Type.POSITION_PID_ARCADE));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    public Step doTurnToHeading(double heading) {
        drive = Optional.of(new Parameters(Type.TURN_TO_BEARING));
        drive.get().value = heading;
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    public Step driveRelativeWaypoints(String filename) throws IOException {
        logString = Optional.of(String.format("Running path: %s", filename));
        drive = Optional.of(Mode.getDriveWaypoints(filename));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    /**
     * Add waypoints for the drive base to drive through.
     * Note: The robot will come to a complete halt after each list
     * of Waypoints, so each Step will cause the robot to drive and then
     * halt ready for the next step. This should be improved.
     * Waypoints are relative to the robots position.
     * 
     * @param start the assumed starting point and angle.
     * @param waypoints list of Waypoints to drive through.
     * @param end the end point and angle.
     * @param forward drive forward through waypoints.
     */
    public Step driveRelativeWaypoints(Pose2d start, List<Translation2d> interiorWaypoints,
            Pose2d end,
            boolean forward) {
        drive = Optional
                .of(Mode.getDriveWaypoints(start, interiorWaypoints, end, forward, true));
        domains.add(Domain.DRIVEBASE);
        return this;
    }

    /**
     * Auto fill the endStep to be applied when a sequence is interrupted
     */
    public void fillInterrupt(Step newStep) {
        intakeDutyCycle = fillParam(intakeDutyCycle, newStep.intakeDutyCycle);
        armAngle = fillParam(armAngle, newStep.armAngle);
        armAngleDelta = fillParam(armAngleDelta, newStep.armAngleDelta);
        armExtension = fillParam(armExtension, newStep.armExtension);
        armExtensionDelta = fillParam(armExtensionDelta, newStep.armExtensionDelta);
        forceCalibration = fillParam(forceCalibration, newStep.forceCalibration);
        domains.addAll(newStep.domains);
    }

    /**
     * Set value to newValue if newValue is non null
     */
    private static <T> Optional<T> fillParam(Optional<T> value, Optional<T> newValue) {
        if (newValue.isPresent())
            return newValue;
        else
            return value;
    }

    /**
     * Append the description and value for this parameter if value is non null.
     * 
     * @param name of the parameter.
     * @param value of the parameter. May be null.
     * @param result - StringBuilder to add to.
     */
    private static <T> void maybeAdd(String name, Optional<T> value, ArrayList<String> result) {
        if (!value.isPresent())
            return; // Ignore this value.
        result.add(name + ":" + value.get());
    }

    @Override
    public String toString() {
        ArrayList<String> result = new ArrayList<String>();
        maybeAdd("timeAction", timeAction, result);
        maybeAdd("logString", logString, result);
        maybeAdd("currentPose", currentPose, result);
        maybeAdd("intakeDutyCycle", intakeDutyCycle, result);
        maybeAdd("drive", drive, result);
        maybeAdd("armAngle", armAngle, result);
        maybeAdd("armAngleDelta", armAngleDelta, result);
        maybeAdd("armExtension", armExtension, result);
        maybeAdd("armExtensionDelta", armExtensionDelta, result);
        maybeAdd("ledColour", ledColour, result);
        maybeAdd("gamepadRumbleIntensity", gamepadRumbleIntensity, result);
        maybeAdd("forceCalibration", forceCalibration, result);
        return "[" + String.join(",", result) + "]";
    }

    // These are the different domains that can be controlled. Routines
    // will abort any current running routines if they have any overlap
    // in domains.
    private EnumSet<Domain> domains = EnumSet.noneOf(Domain.class);

    public EnumSet<Domain> getDomains() {
        return domains;
    }
}
