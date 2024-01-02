package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controller.Controller;
import frc.robot.controller.Routine;
import frc.robot.controller.Routine.RoutineBuilder;
import frc.robot.controller.Routines;
import frc.robot.interfaces.LogHelper;
import frc.robot.lib.AutoPaths;
import java.io.IOException;
import java.util.concurrent.Callable;

/**
 * Handles auto routine selection.
 * 
 * Auto routines should be defined in Routines.java
 */
public class Auto implements LogHelper {
    private SendableChooser<Routine> autoProgram = new SendableChooser<Routine>();

    public Auto() {
        addAutoRoutines();
        initAutoChooser();
    }

    public void executedSelectedRoutine(Controller controller) {
        Routine seq = autoProgram.getSelected();
        info("Starting selected auto program %s", seq.getName());
        controller.run(seq);
    }

    private void addAutoRoutines() {
        autoProgram.setDefaultOption("Nothing", Routines.getEmptyRoutine());
        addAutoRoutine(
                String.format("Score cone high then intake"),
                () -> scoreConeHighGoal(Config.timedAuto.scoreConeThenIntake.secondsOutward,
                        Config.timedAuto.scoreConeThenIntake.secondsInward));
        addAutoRoutine("Score cone high then balance", () -> scoreConeHighGoalThenBalance(
                Config.timedAuto.scoreConeThenBalance.secondsToDrive));
        addAutoRoutine("Score cone", () -> scoreOnly());
        addAutoRoutine("Pathweaver: Score cone, mobility", () -> scoreMobility());
        addAutoRoutine("PathWeaver: Score cone, mobility, engage", () -> scoreMobilityEngage());
        addAutoRoutine("PathWeaver: BLUE BUMP Score two pieces", () -> scoreTwoPieces(true));
        addAutoRoutine("PathWeaver: RED BUMP Score two pieces", () -> scoreTwoPieces(false));
        addAutoRoutine("PathWeaver: BLUE CLEAR Score two pieces", () -> scoreTwoPieces(false));
        addAutoRoutine("PathWeaver: RED CLEAR Score two pieces", () -> scoreTwoPieces(true));
        // addAutoRoutine("SysId test", () -> addSysIdTestRoutine());
    }

    /**
     * Tries to build an autonomous routine.
     * If it fails it will put replace it with an empty routine on the chooser.
     * 
     * @param name the name of the routine
     * @param autoRoutine
     */
    void addAutoRoutine(String name, Callable<Routine> autoRoutine) {
        try {
            Routine routine = autoRoutine.call();
            autoProgram.addOption(name, routine);
        } catch (Exception e) {
            exception(String.format("Failed to create routine %s", name), e);
            autoProgram.addOption(String.format("FAILED - %s", name), Routines.getEmptyRoutine());
            return;
        }
    }

    private void initAutoChooser() {
        SmartDashboard.putData("Auto program", autoProgram);
    }

    @SuppressWarnings("unused")
    private Routine addSysIdTestRoutine() throws IOException {
        RoutineBuilder builder = new RoutineBuilder("SysId Test");

        builder.then().setCurrentPosition(new Pose2d(2, 1, new Rotation2d(0)));
        builder.then().driveRelativeWaypoints(AutoPaths.kSysid);

        return builder.build();
    }

    private Routine scoreConeHighGoal(double secondsOutward, double secondsInward) {
        RoutineBuilder builder = new RoutineBuilder("Score cone high then drive backward");
        builder.then().setIntakeDutyCycle(-Config.intake.coneHoldDutyCycle);
        builder.appendRoutine(Routines.moveArm(Config.arm.position.cone.high.angle,
                Config.arm.position.cone.high.extension, "score cone high"));
        builder.then().setIntakeDutyCycle(Config.intake.coneOutDutyCycle);
        builder.then().setDelayDelta(0.5);
        builder.then().setIntakeDutyCycle(0);
        builder.appendRoutine(
                Routines.moveArm(Config.arm.position.stowedAngle,
                        Config.arm.telescope.minExtension,
                        "stow arm"));
        builder.then().setDrivebasePower(-0.6);
        builder.appendRoutine(Routines.startCubeIntaking());
        builder.then().setDelayDelta(secondsOutward);
        builder.then().setDrivebasePower(0);
        builder.appendRoutine(Routines.stopCubeIntaking());
        builder.then().setDrivebasePower(0.4);
        builder.then().setDelayDelta(secondsInward);
        builder.then().setDrivebasePower(0);
        return builder.build();
    }

    private Routine scoreConeHighGoalThenBalance(double secondsToDrive) {
        RoutineBuilder builder = new RoutineBuilder("Score cone high then balance");
        builder.then().setIntakeDutyCycle(-Config.intake.coneHoldDutyCycle);
        builder.then().setArmAngle(Config.arm.position.cone.high.angle);
        builder.then().setArmExtension(Config.arm.position.cone.high.extension);
        builder.then().setIntakeDutyCycle(Config.intake.coneOutDutyCycle);
        builder.then().setDelayDelta(0.5);
        builder.then().setIntakeDutyCycle(0);
        builder.appendRoutine(
                Routines.moveArm(Config.arm.position.stowedAngle,
                        Config.arm.telescope.minExtension,
                        "stow arm"));
        builder.then().setDrivebasePower(-0.4);
        builder.then().setDelayDelta(secondsToDrive);
        builder.appendRoutine(Routines.setDrivebaseToBalance(false));

        return builder.build();
    }

    /**
     * Score a cone in the high goal
     * Drive over the charge station into the community
     * Drive back onto the charge station to engage via auto-balancing
     * 
     * @return the build Routine
     * @throws IOException when the Routine cannot be created
     */
    private Routine scoreMobilityEngage() throws IOException {
        RoutineBuilder builder = new RoutineBuilder("PathWeaver: Score cone, mobility, engage");
        builder.then().setCurrentPosition(new Pose2d(2.18, 3.3, new Rotation2d(3.141592653589793)));

        // Score a cone in the high goal
        builder.appendRoutine(Routines.scoreConeHigh());
        builder.then().setArmAngle(Config.arm.position.stowedAngle)
                .setArmExtension(Config.arm.telescope.minExtension)
                .driveRelativeWaypoints(AutoPaths.kScoreMobilityEngageA);

        // Drive out of community
        builder.then().driveRelativeWaypoints(AutoPaths.kScoreMobilityEngageB);

        builder.then().doBalanceDrive(true);

        return builder.build();
    }

    /**
     * Score a cone in the high goal
     * Drive out of the community
     * Intake a cube
     * Score it in the low goal
     * 
     * @return the resulting routine
     * @throws IOException when the routine cannot be created
     */
    private Routine scoreTwoPieces(boolean blue) throws IOException {
        String name =
                blue ? "PathWeaver: BLUE Score two pieces" : "PathWeaver: RED Score two pieces";
        Pose2d startPose = blue ? new Pose2d(2.18, 0.5, new Rotation2d(3.141592653589793))
                : new Pose2d(14.36, 0.5, new Rotation2d(0));
        // Start in line with the high cone goal closest to the edge of the field (the side with the
        // bump) and drive towards the cube closest to the edge
        String pathA = blue ? AutoPaths.kTwoPieceBlueA : AutoPaths.kTwoPieceRedA;
        // Drive right up to the cube to intake it
        String pathB = blue ? AutoPaths.kTwoPieceBlueB : AutoPaths.kTwoPieceRedB;
        // Drive to the high cube goal next to the previously scored cone
        String pathC = blue ? AutoPaths.kTwoPieceBlueC : AutoPaths.kTwoPieceRedC;

        RoutineBuilder builder = new RoutineBuilder(name);

        // Score a cone in the high goal
        builder.appendRoutine(Routines.scoreConeHigh());

        // Drive out of community and pick up cube
        builder.then().setCurrentPosition(startPose);
        builder.then().driveRelativeWaypoints(pathA)
                .setArmAngle(Config.arm.telescope.maxSafeExtensionAngle);

        builder.then().setArmExtension(Config.arm.position.cube.intake.extension);
        builder.then().setArmAngle(Config.arm.position.cube.intake.angle);

        builder.then().setIntakeDutyCycle(Config.intake.cubeDutyCycle)
                .driveRelativeWaypoints(pathB);

        // Drive back to score
        builder.then().setIntakeDutyCycle(Config.intake.cubeHoldDutyCycle)
                .driveRelativeWaypoints(pathC)
                .setArmAngle(Config.arm.position.cube.high.angle)
                .setArmExtension(Config.arm.position.cube.high.extension);

        // Score cube in the low goal
        builder.appendRoutine(Routines.reverseCubeIntaking());
        builder.then().setDelayDelta(1);
        builder.appendRoutine(Routines.stopIntake());

        return builder.build();
    }

    /**
     * Score a cone in the high goal
     * Retract the arm
     * 
     * @return the resulting routine
     * @throws IOException when the routine cannot be created
     */
    private Routine scoreOnly() throws IOException {
        RoutineBuilder builder = new RoutineBuilder("Score cone, retract arm");

        // Score a cone in the high goal
        builder.appendRoutine(Routines.scoreConeHigh());
        builder.then().setArmAngle(Config.arm.position.stowedAngle)
                .setArmExtension(Config.arm.telescope.minExtension);

        return builder.build();
    }

    /**
     * Score a cone in the high goal
     * Drive over the charge station into the community
     * 
     * @return the build Routine
     * @throws IOException when the Routine cannot be created
     */
    private Routine scoreMobility() throws IOException {
        RoutineBuilder builder = new RoutineBuilder("PathWeaver: Score cone, mobility");
        builder.then().setCurrentPosition(new Pose2d(2.18, 3.3, new Rotation2d(3.141592653589793)));

        // Score a cone in the high goal
        builder.appendRoutine(Routines.scoreConeHigh());
        builder.then().setArmAngle(Config.arm.position.stowedAngle)
                .setArmExtension(Config.arm.telescope.minExtension)
                .driveRelativeWaypoints(AutoPaths.kScoreMobilityEngageA);

        // Drive out of community
        builder.then().driveRelativeWaypoints(AutoPaths.kScoreMobilityEngageB);

        return builder.build();
    }



    @Override
    public String getName() {
        return "Auto";
    }
}
