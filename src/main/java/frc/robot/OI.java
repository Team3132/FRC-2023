package frc.robot;

import frc.robot.controller.Controller;
import frc.robot.controller.Routine;
import frc.robot.controller.Routines;
import frc.robot.drive.modes.*;
import frc.robot.interfaces.*;
import frc.robot.lib.GamepadButtonsX;
import frc.robot.lib.log.Log;
import frc.robot.subsystems.*;
import org.strongback.Executor.Priority;
import static frc.robot.drive.modes.Mode.Type;
import org.strongback.Strongback;
import org.strongback.SwitchReactor;
import org.strongback.components.Clock;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.Switch;
import org.strongback.components.ui.*;

public class OI {

    private SwitchReactor reactor = Strongback.switchReactor();
    private Controller controller;
    private Subsystems subsystems;

    public OI(Controller controller, Subsystems subsystems) {
        this.controller = controller;
        this.subsystems = subsystems;
    }

    /**
     * Configure the driver interface.
     * 
     * @param left the drivers left joystick
     * @param right the drivers right joystick
     */
    public void configureDriverJoysticks(FlightStick left, FlightStick right) {
        // Intake
        left.trigger().onPress(run(Routines.startConeIntaking()));
        left.trigger().onRelease(run(Routines.stopConeIntaking()));

        right.trigger().onPress(run(Routines.startCubeIntaking()));
        right.trigger().onRelease(run(Routines.stopIntake()));

        // Positional PID drive for more accurate slow driving
        right.thumb().onPress(run(Routines.setDrvebaseToPositionalDrive()));
        right.thumb().onRelease(run(Routines.setDrivebaseToArcade()));

        // Arcade
        right.button(3).onPress(run(Routines.setDrivebaseToBalance(false)));
        right.button(4).onPress(run(Routines.setDrivebaseToDefault()));
    }

    /**
     * Configure the operators interface.
     * 
     * @param gamepad the operators joystick
     */
    public void configureOperatorJoystick(Gamepad gamepad) {
        ModeSwitch mode = onMode(
                gamepad.isTriggered(GamepadButtonsX.LEFT_TRIGGER, GamepadButtonsX.AXIS_THRESHOLD),
                gamepad.isTriggered(GamepadButtonsX.LEFT_TRIGGER, GamepadButtonsX.AXIS_THRESHOLD)
                        .invert(),
                "cube/cone",
                Routines.getEmptyRoutine(), Routines.getEmptyRoutine());

        // X Button / Square - Mid Score
        mode.onPress(gamepad.getButton(GamepadButtonsX.X_BUTTON),
                Routines.moveArm(
                        Config.arm.position.cube.mid.angle,
                        Config.arm.position.cube.mid.extension, "score cube mid"),
                Routines.moveArm(
                        Config.arm.position.cone.mid.angle,
                        Config.arm.position.cone.mid.extension, "score cone mid"));

        // Y Button / Triangle - High Score
        mode.onPress(gamepad.getButton(GamepadButtonsX.Y_BUTTON),
                Routines.moveArm(
                        Config.arm.position.cube.high.angle,
                        Config.arm.position.cube.high.extension, "score cube high"),
                Routines.moveArm(
                        Config.arm.position.cone.high.angle,
                        Config.arm.position.cone.high.extension, "score cone high"));

        mode.onPress(gamepad.getButton(GamepadButtonsX.A_BUTTON),
                Routines.moveArm(
                        Config.arm.position.cube.backMid.angle,
                        Config.arm.position.cube.backMid.extension, "score cube back mid"),
                Routines.getEmptyRoutine());

        mode.onPress(gamepad.getButton(GamepadButtonsX.B_BUTTON),
                Routines.moveArm(
                        Config.arm.position.cube.backHigh.angle,
                        Config.arm.position.cube.backHigh.extension, "score cube back high"),
                Routines.getEmptyRoutine());

        gamepad.backButton().onPress(run(Routines.moveArm(
                Config.arm.position.stowedAngle,
                Config.arm.telescope.minExtension,
                "stow")));

        gamepad.startButton().onPress(run(Routines.forceArmCalibration()));

        // Right Bumper - Intake
        mode.onPress(gamepad.getButton(GamepadButtonsX.RIGHT_BUMPER),
                Routines.startCubeIntaking(),
                Routines.startConeIntaking());
        mode.onRelease(gamepad.getButton(GamepadButtonsX.RIGHT_BUMPER),
                Routines.stopCubeIntaking(),
                Routines.stopConeIntaking());

        // Right Trigger - Score
        mode.onPress(
                gamepad.isTriggered(GamepadButtonsX.RIGHT_TRIGGER, GamepadButtonsX.AXIS_THRESHOLD),
                Routines.reverseCubeIntaking(),
                Routines.reverseConeIntaking());
        mode.onRelease(
                gamepad.isTriggered(GamepadButtonsX.RIGHT_TRIGGER, GamepadButtonsX.AXIS_THRESHOLD),
                Routines.stopIntake(),
                Routines.stopConeOuttaking());

        gamepad.DPadAxis().east().onPress(run(Routines.adjustArmAngle(5)));
        gamepad.DPadAxis().west().onPress(run(Routines.adjustArmAngle(-5)));

        gamepad.DPadAxis().north().onPress(run(Routines.adjustArmExtension(0.02)));
        gamepad.DPadAxis().south().onPress(run(Routines.adjustArmExtension(-0.01)));
    }

    public void configureDDRPad(Dancepad dancepad) {}

    public void configureDiagBox(DiagnosticBox box) {}

    /**
     * Registers all of the available drive routines that can be requested by the controller.
     */
    public void registerDriveModes(FlightStick left, FlightStick right,
            Dancepad dancepad) {
        // Convenience variables.
        Drivebase drivebase = subsystems.drivebase;
        Location location = subsystems.location;
        Clock clock = subsystems.clock;

        // Add the supported drive routines
        drivebase.register(Type.CONSTANT_POWER,
                new ConstantDrive("Constant Power", ControlMode.DutyCycle));
        drivebase.register(Type.CONSTANT_SPEED,
                new ConstantDrive("Constant Speed", ControlMode.Speed));

        // The old favorite arcade drive with throttling if a button is pressed.
        drivebase.register(Type.ARCADE_DUTY_CYCLE,
                new ArcadeDrive("ArcadeDutyCycle", ControlMode.DutyCycle, 1.0,
                        // Throttle.
                        left.getAxis(1).invert().deadband(
                                Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger()
                                        .isTriggered() ? 0.36
                                                : 1),
                        // Turn power.
                        right.getAxis(0).invert().deadband(
                                Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger()
                                        .isTriggered() ? 0.36
                                                : 1)));

        // The old favourite arcade drive with throttling if a button is pressed but
        // using velocity mode.
        drivebase.register(Type.ARCADE_VELOCITY,
                new ArcadeDrive("ArcadeVelocity", ControlMode.Speed,
                        Config.drivebase.maxSpeed,
                        // Throttle
                        left.getAxis(1).invert().deadband(
                                Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger()
                                        .isTriggered() ? 1
                                                : 0.36),
                        // Turn power.
                        right.getAxis(0).invert().deadband(
                                Config.ui.joystick.deadbandMinValue)
                                .squarePreservingSign()
                                .scale(() -> left.getTrigger()
                                        .isTriggered() ? 1
                                                : 0.36)));

        // DDR!
        if (dancepad.getButtonCount() > 0) {
            drivebase.register(Type.DDRPAD_DRIVE,
                    new ArcadeDrive("DDRPadDrive", ControlMode.DutyCycle, 1.0,
                            dancepad.getAxis(1).invert().scale(0.5), // Throttle.
                            dancepad.getAxis(0).invert().scale(0.4) // Turn
                                                                    // power.
                    ));
        }

        // Cheesy drive.
        drivebase.register(Type.CHEESY,
                new CheesyDpadDrive("CheesyDPad", left.getDPad(0), // DPad
                        left.getAxis(GamepadButtonsX.LEFT_Y_AXIS), // Throttle
                        left.getAxis(GamepadButtonsX.RIGHT_X_AXIS), // Wheel
                                                                    // (turn?)
                        left.getButton(GamepadButtonsX.RIGHT_TRIGGER_AXIS))); // Is
                                                                              // quick
                                                                              // turn

        // Drive through supplied waypoints using splines.
        drivebase.register(Type.TRAJECTORY,
                new TrajectoryDrive(location, clock));

        // Turns on the spot to a specified bearing.
        drivebase.register(Type.TURN_TO_BEARING,
                new TurnToBearing(drivebase, location, clock));

        // Map joysticks in arcade mode for testing/tuning.
        drivebase.register(Type.POSITION_PID_ARCADE,
                new PIDDrive("PIDDrive",
                        () -> left.getAxis(1).invert()
                                .squarePreservingSign()
                                .scale(Config.drivebase.maxPIDSpeed).read(),
                        () -> right.getAxis(0).squarePreservingSign()
                                .scale(Config.drivebase.maxPIDTurn).read(),
                        drivebase, clock));

        Balance balance = new Balance(drivebase, location, clock);
        drivebase.register(Type.BALANCE, balance);
        Strongback.executor().register(balance, Priority.HIGH);
    }

    /**
     * Three position switch showing up as two buttons. Allows switching between
     * automatic, manual and disabled modes.
     * 
     * @param box the button box as a joystick.
     * @param colour the colour of the override switch.
     * @param subsystem the subystem to set the mode on.
     */
    private void mapOverrideSwitch(DiagnosticBox box, DiagnosticBox.Colour colour,
            OverridableSubsystem<?> subsystem) {
        box.overrideSwitch(colour, () -> subsystem.setAutomaticMode(),
                () -> subsystem.setManualMode(),
                () -> subsystem.turnOff());
    }

    /**
     * Changes the routines mapped to buttons depending on a mode. The mode can be
     * enabled or disabled based on a button. An example would be to have an
     * intaking or shooting mode, where the buttons run different routines
     * depending on which buttons are pressed. Turning on the mode is one button and
     * turning it off is another.
     * 
     * Example:
     * 
     * <pre>
     * {@code
     * onMode(rightStick.getButton(5), rightStick.getButton(6), "climb/drive",
     *         Routines.enableClimbMode(), Routines.enableDriveMode())
     *                 .onPress(rightStick.getButton(1),
     *                         Routines.releaseClimberRatchet(),
     *                         Routines.startSlowDriveForward())
     *                 .onRelease(rightStick.getButton(2),
     *                         Routines.releaseClimberRatchet(),
     *                         Routines.driveFast());
     * }
     * </pre>
     * 
     * Button 5 enabled climb mode, button 6 enables drive mode. If in climb mode,
     * buttons 1 and 2 run different routines.
     * 
     * @param switchOn condition used to enable the mode. Normally a button
     *        press.
     * @param switchOff condition used to disable the mode. Normally a button
     *        press.
     * @param name used for logging when the mode changes.
     * @param activateSeq routine to run when the mode is actived.
     * @param deactiveSeq routine to run when the mode is deactived.
     * @return the ModeSwitch for further chaining of more buttons based on the
     *         mode.
     */
    @SuppressWarnings("unused")
    private ModeSwitch onMode(Switch switchOn, Switch switchOff, String name,
            Routine activateSeq,
            Routine deactiveSeq) {
        return new ModeSwitch(switchOn, switchOff, name, activateSeq, deactiveSeq);
    }

    /**
     * Changes the routines mapped to buttons depending on a mode. The mode can be
     * enabled or disabled based on a button. An example would be to have an
     * intaking or shooting mode, where the buttons run different routines
     * depending on which buttons are pressed. Turning on the mode is one button and
     * turning it off is another.
     */
    @SuppressWarnings("unused")
    private class ModeSwitch {
        private boolean active = false;

        /**
         * Creates a ModeSwitch to track the state and run routines on state change.
         * 
         * @param switchOn condition used to enable the mode. Normally a button
         *        press.
         * @param switchOff condition used to disable the mode. Normally a button
         *        press.
         * @param name used for logging when the mode changes.
         * @param activatedSeq routine to run when the mode is actived.
         * @param deactivedSeq routine to run when the mode is deactived.
         * @return the ModeSwitch for chaining of more buttons based on the mode.
         */
        public ModeSwitch(Switch switchOn, Switch switchOff, String name,
                Routine activatedSeq,
                Routine deactivedSeq) {
            Strongback.switchReactor().onTriggered(switchOn, () -> {
                if (active) {
                    return;
                }
                Log.debug("Routines", "Activating " + name);
                controller.run(activatedSeq);
                active = true;
            });
            Strongback.switchReactor().onTriggered(switchOff, () -> {
                if (!active) {
                    return;
                }
                Log.debug("Routines", "Deactivating " + name);
                controller.run(deactivedSeq);
                active = false;
            });
        }

        /**
         * Run different routines depending on the mode on button press.
         * 
         * @param swtch condition to trigger a routine to run. Normally a button
         *        press.
         * @param activeSeq routine to run if the mode is active.
         * @param inactiveSeq routine to run if the mode is inactive.
         * @return the ModeSwitch for further chaining of more buttons based on the
         *         mode.
         */
        public ModeSwitch onPress(Switch swtch, Routine activeSeq, Routine inactiveSeq) {
            reactor.onTriggered(swtch, () -> {
                if (active) {
                    controller.run(activeSeq);
                } else {
                    controller.run(inactiveSeq);
                }
            });
            return this;
        }

        /**
         * Run different routines depending on the mode on button release.
         * 
         * @param swtch condition to trigger a routine to run. Normally a button
         *        release.
         * @param activeSeq routine to run if the mode is active.
         * @param inactiveSeq routine to run if the mode is inactive.
         * @return the ModeSwitch for further chaining of more buttons based on the
         *         mode.
         */
        public ModeSwitch onRelease(Switch swtch, Routine activeSeq,
                Routine inactiveSeq) {
            reactor.onUntriggered(swtch, () -> {
                if (active) {
                    controller.run(activeSeq);
                } else {
                    controller.run(inactiveSeq);
                }
            });
            return this;
        }

        /**
         * Run different routines depending on the mode while a button is pressed.
         * 
         * @param swtch condition to trigger a routine to run. Normally while a
         *        button is pressed.
         * @param activeSeq routine to run if the mode is active.
         * @param inactiveSeq routine to run if the mode is inactive.
         * @return the ModeSwitch for further chaining of more buttons based on the
         *         mode.
         */
        public ModeSwitch whileTriggered(Switch swtch, Routine activeSeq,
                Routine inactiveSeq) {
            reactor.whileTriggered(swtch, () -> {
                if (active) {
                    controller.run(activeSeq);
                } else {
                    controller.run(inactiveSeq);
                }
            });
            return this;
        }
    }

    /**
     * Converts a routine into something runnable. Returns the routine name in
     * toString() for the code that prints out the button mappings in Trigger.
     * 
     * @param routine the routine to run.
     * @return Runnable that can be executed by the Trigger methods.
     */
    public Runnable run(Routine routine) {
        return new Runnable() {
            @Override
            public void run() {
                controller.run(routine);
            }

            @Override
            public String toString() {
                return routine.getName();
            }
        };

    }
}
