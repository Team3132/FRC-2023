/**
 * Routines for doing most actions on the robot.
 * 
 * If you add a new routine, add it to allRoutines at the end of this file.
 */
package frc.robot.controller;

import static frc.robot.lib.PoseHelper.createPose2d;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Config;
import frc.robot.controller.Routine.RoutineBuilder;
import frc.robot.lib.LEDColour;
import java.util.List;

/**
 * Control routines for most robot operations.
 */
public class Routines {

    /**
     * Do nothing routine.
     */
    public static Routine getEmptyRoutine() {
        if (emptySeq == null) {
            emptySeq = new RoutineBuilder("empty").build();
        }
        return emptySeq;
    }

    private static Routine emptySeq = null;

    /**
     * The first routine run in the autonomous period.
     */
    public static Routine getStartRoutine() {
        if (startSeq == null) {
            startSeq = new RoutineBuilder("start").build();
        }
        return startSeq;
    }

    private static Routine startSeq = null;

    /**
     * Returns the routine to reset the robot. Used to stop ejecting etc. The lift
     * is at intake height, the intake is stowed, all motors are off.
     * 
     * @return
     */
    public static Routine getResetRoutine() {
        if (resetSeq == null) {
            RoutineBuilder builder = new RoutineBuilder("empty");
            builder.then().doDefaultDrive();
            resetSeq = builder.build();
        }
        return resetSeq;
    }

    private static Routine resetSeq = null;

    /**
     * Drive to a point on the field, relative to the starting point.
     * 
     * @param angle the final angle (relative to the field) in degrees.
     */
    public static Routine getDriveToWaypointRoutine(double x, double y, double angle) {
        Pose2d start = new Pose2d();
        Pose2d end = createPose2d(x, y, angle);
        RoutineBuilder builder = new RoutineBuilder(String.format("drive to %s", end));
        builder.then().driveRelativeWaypoints(start, List.of(), end, true);
        driveToWaypointSeq = builder.build();
        return driveToWaypointSeq;
    }

    private static Routine driveToWaypointSeq = null;

    public static Routine setDrivebaseToArcade() {
        RoutineBuilder builder = new RoutineBuilder("Arcade Drive Routine");
        builder.then().doArcadeDrive();
        return builder.build();
    }

    public static Routine setDrivebaseToDefault() {
        RoutineBuilder builder = new RoutineBuilder("Default Drive Routine");
        builder.then().doDefaultDrive();
        return builder.build();
    }

    public static Routine setDrvebaseToPositionalDrive() {
        RoutineBuilder builder = new RoutineBuilder("PositionalPID Drive Routine");
        builder.then().doPositionPIDArcade();
        return builder.build();
    }

    /**
     * Set the drivebase to routine that automatically balances on the charge station.
     */
    public static Routine setDrivebaseToBalance(boolean forward) {
        RoutineBuilder builder = new RoutineBuilder("Balance Drive Routine");
        builder.then().doBalanceDrive(forward);
        return builder.build();
    }

    /**
     * Extends the intake and then runs the motor to intake the cargo.
     * 
     * @return
     */

    public static Routine startConeIntaking() {
        RoutineBuilder builder = new RoutineBuilder("Start cone intaking");
        builder.appendRoutine(Routines.moveArm(Config.arm.position.cone.intake.angle,
                Config.arm.telescope.minExtension, "cone chute"));
        builder.then().setIntakeDutyCycle(-Config.intake.coneDutyCycle);
        builder.then().setColour(LEDColour.YELLOW);
        return builder.build();
    }

    public static Routine reverseConeIntaking() {
        RoutineBuilder builder = new RoutineBuilder("Reverse cone intaking");
        builder.then().setIntakeDutyCycle(Config.intake.coneOutDutyCycle);
        return builder.build();
    }

    public static Routine stopConeIntaking() {
        RoutineBuilder builder = new RoutineBuilder("Stop cone intaking");
        builder.then().setIntakeDutyCycle(-Config.intake.coneHoldDutyCycle);
        return builder.build();
    }

    public static Routine stopConeOuttaking() {
        RoutineBuilder builder = new RoutineBuilder("Stop cone outtaking");
        builder.then().setIntakeDutyCycle(0);
        return builder.build();
    }

    public static Routine startCubeIntaking() {
        RoutineBuilder builder = new RoutineBuilder("Start cube intaking");
        builder.appendRoutine(moveArmToIntakePosition());
        builder.then().setIntakeDutyCycle(Config.intake.cubeDutyCycle);
        builder.then().setColour(LEDColour.PURPLE);
        // Create an interrupt state that will only start the intake, but not move the arm, allowing
        // the routine to be aborted on change of routine or by the current special case in the
        // routine runner without moving the arm to a potentially dangerous position.
        builder.onInterrupt().setIntakeDutyCycle(Config.intake.cubeDutyCycle)
                .setColour(LEDColour.PURPLE);
        return builder.build();
    }

    public static Routine reverseCubeIntaking() {
        RoutineBuilder builder = new RoutineBuilder("Reverse cube intaking");
        builder.then().setIntakeDutyCycle(-Config.intake.cubeOutDutyCycle);
        return builder.build();
    }

    public static Routine stopCubeIntaking() {
        RoutineBuilder builder = new RoutineBuilder("Stop cube intaking");
        builder.appendRoutine(moveArmFromIntakePosition());
        builder.then().setIntakeDutyCycle(0);
        builder.createInterruptState();
        return builder.build();
    }

    public static Routine stopIntake() {
        RoutineBuilder builder = new RoutineBuilder("Stop intaking");
        builder.then().setIntakeDutyCycle(0);
        return builder.build();
    }

    public static Routine constantDrivePower(double power) {
        RoutineBuilder builder = new RoutineBuilder("Constant drive power " + power);
        builder.then().setDrivebasePower(power);
        return builder.build();
    }

    public static Routine setLEDColour(LEDColour c) {
        RoutineBuilder builder = new RoutineBuilder("Set LEDS to " + c);
        builder.then().setColour(c);
        return builder.build();
    }

    public static Routine setArmAngle(double degrees) {
        RoutineBuilder builder =
                new RoutineBuilder(String.format("Arm angle %.1f", degrees));
        builder.then().setArmAngle(degrees);
        return builder.build();
    }

    public static Routine adjustArmAngle(double degrees) {
        RoutineBuilder builder =
                new RoutineBuilder(String.format("Adjust arm angle %.1f", degrees));
        builder.then().adjustArmAngle(degrees);
        return builder.build();
    }


    public static Routine setArmExtension(double position) {
        RoutineBuilder builder =
                new RoutineBuilder(String.format("Arm extension %.1f", position));
        builder.then().setArmExtension(position);
        return builder.build();
    }

    public static Routine adjustArmExtension(double position) {
        RoutineBuilder builder =
                new RoutineBuilder(String.format("Adjust arm extension %.1f", position));
        builder.then().adjustArmExtension(position);
        return builder.build();
    }

    public static Routine moveArm(double angle, double extension, String name) {
        RoutineBuilder builder = new RoutineBuilder(
                String.format("Arm to ", name));
        builder.then().setArmAngle(angle).setArmExtension(extension);
        return builder.build();
    }

    // Needs to overrideSafety as this routine intends to move the extended arm past the safe angle
    public static Routine moveArmToIntakePosition() {
        RoutineBuilder builder =
                new RoutineBuilder(String.format("Arm to intake pos"));
        builder.then().setArmAngle(Config.arm.telescope.maxSafeExtensionAngle);
        builder.then().setArmExtension(Config.arm.position.cube.intake.extension);
        builder.then().setArmAngle(Config.arm.position.cube.intake.angle);
        builder.then().setColour(LEDColour.PURPLE);
        return builder.build();
    }

    public static Routine moveArmFromIntakePosition() {
        RoutineBuilder builder =
                new RoutineBuilder(String.format("Arm to intake pos"));
        builder.then().setArmAngle(Config.arm.telescope.maxSafeExtensionAngle);
        builder.then().setArmExtension(Config.arm.telescope.minExtension);
        return builder.build();
    }

    public static Routine forceArmCalibration() {
        RoutineBuilder builder = new RoutineBuilder(String.format("Force arm to calibrate"));
        builder.then().forceArmCalibration();
        return builder.build();
    }

    // Move and extend the arm to the high goal and outtake a cone
    public static Routine scoreConeHigh() {
        RoutineBuilder builder = new RoutineBuilder("Score cone high");
        // Set a low dutyCycle to keep the cone in place
        builder.then().setIntakeDutyCycle(-Config.intake.coneHoldDutyCycle);
        // Move and extend arm to high goal
        builder.appendRoutine(moveArm(Config.arm.position.cone.high.angle,
                Config.arm.position.cone.high.extension, "score cone high"));
        builder.appendRoutine(Routines.reverseConeIntaking());
        builder.then().setDelayDelta(0.5);
        builder.appendRoutine(Routines.stopConeOuttaking());
        return builder.build();
    }

    // For testing. Needs to be at the end of the file.
    public static Routine[] allRoutines = new Routine[] {
            getEmptyRoutine(), getStartRoutine(), getResetRoutine(),
            startConeIntaking(), stopConeIntaking(), reverseConeIntaking(),
            startCubeIntaking(), stopCubeIntaking(), reverseCubeIntaking(),
            moveArmFromIntakePosition(), moveArmToIntakePosition(),
    };
}
