package frc.robot;



import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import frc.robot.lib.ConfigReader;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.strongback.components.PIDF;

/**
 * Class responsible for updating values which are dependent on robot hardware.
 * (e.g. if subsystems are enabled or not) It reads from a text file Currently
 * the supported types are String, int, double, boolean and int array.
 * 
 * Example lines:
 * drivebase/enabled = true
 * drivebase/rampRate = 0.13125
 * drivebase/right/canIDs/withEncoders = 7,6
 * drivebase/right/canIDs/withoutEncoders = 5
 * 
 * The configuration can be overridden on each robot by changing a text file
 * stored on the robot allowing different robots to have different configuration
 * preventing having to modify the code each time it's pushed to a different bit
 * of hardware.
 * 
 * This is very useful for testing when parts of the hardware are not attached,
 * delivered or even broken.
 */
public class Config {

    /*
     * Drivebase parameters
     * 
     * Six wheel drop centre with 6" wheels.
     */
    public static class drivebase {
        public static final boolean enabled = getBoolean("drivebase/enabled", true);
        public static final String motorControllerType =
                getMotorControllerType("drivebase/motorControllerType",
                        motorController.defaultType);
        public static final int[] canIdsLeftWithEncoders =
                getIntArray("drivebase/left/canIDs/withEncoders",
                        new int[] {1, 2});
        public static final int[] canIdsLeftWithoutEncoders =
                getIntArray("drivebase/left/canIDs/withoutEncoders",
                        new int[] {});
        public static final int[] canIdsRightWithEncoders =
                getIntArray("drivebase/right/canIDs/withEncoders",
                        new int[] {3, 4});
        public static final int[] canIdsRightWithoutEncoders =
                getIntArray("drivebase/right/canIDs/withoutEncoders",
                        new int[] {});
        public static final boolean currentLimiting = getBoolean("drivebase/currentLimiting", true);
        public static final int contCurrent = getInt("drivebase/contCurrent", 38);
        public static final int peakCurrent = getInt("drivebase/peakCurrent", 80);
        public static final double rampRate = getDouble("drivebase/rampRate", 0.1);
        public static final PIDF pidf = getPIDF("drivebase", new PIDF(0, 0, 0, 0.7));
        public static final PIDF autoPidf =
                getPIDF("drivebase/autodrive", new PIDF(0.64, 0.01, 0, 0));
        // Max speed it can drive at in m/s
        public static final double maxSpeed = getDouble("drivebase/maxSpeed", 4.0);
        public static final double maxPIDSpeed = getDouble("drivebase/maxPIDSpeed", 2.0);
        public static final double maxPIDTurn = getDouble("drivebase/maxPIDTurn", 0.25);
        // How quickly it can accelerate in m/s/s
        public static final double maxJerk = getDouble("drivebase/maxJerk", 2.0);
        public static final boolean invert = getBoolean("drivebase/invert", false);
        public static final boolean sensorPhase = getBoolean("drivebase/sensor/phase", false);
        // Distance the robot moves per revolution of the wheels.
        public static final double wheelDiameterMetres = 6 * constants.inchesToMetres; // 6" wheels.
        public static final double metresPerRev = wheelDiameterMetres * Math.PI;
        public static final double gearboxRatio = 10.71;
        public static final double trackwidthMeters = 0.57;
        public static final String driveMode =
                getString("drivebase/driveMode", "ARCADE_DUTY_CYCLE");
        // Metres driven during time based auto


        /**
         * Drive routine configuration
         * All units are in metres.
         */


        public static class routine {
            /**
             * Turn to bearing drive routine
             */
            public static class turnToBearing {
                public static final double scale = 0.04;
            }

            /**
             * Drive to the goal.
             */
            public static class visionDrive {
                public static final double speedScale = 0.06;
                public static final double angleScale = 0.6;
                public static final double splineMinDistanceMetres = 1.5;
                public static final double waypointDistanceScale = 0.5; // percentage 0 to 1
                public static final double maxSpeed = 2; // metres/sec
                public static final double distanceBeforeGoal = 1; // metres
            }

            /**
             * Assist the driver to aim at the goal by taking over steering when
             * a target can be seen.
             */
            public static class visionAssist {
                public static final double angleScale = 0.02;
            }

            /**
             * Automatically turn to a vision target, giving up if no target.
             */
            public static class visionAim {
                public static final double speedScale = 0.06;
                public static final double angleToleranceDegrees = 2; // degrees
                public static final double angleScale = 0.03;
            }

            /**
             * Automatically balance the robot on the charge station.
             */
            public static class balance {
                public static final double entrySpeed = // metres/sec
                        getDouble("drivebase/routine/balance/entryDutyCycle", 0.65);
                public static final double balanceSpeed = // metres/sec
                        getDouble("drivebase/routine/balance/balanceDutyCycle", 0.2);
                public static final double toleranceAngle = // degrees
                        getDouble("drivebase/routine/balance/toleranceAngle", 5);
                public static final double approachingZeroToleranceAngle =
                        getDouble("drivebase/routine/balance/ApproachingZeroToleranceAngle", 2.5);
                public static final double minDistance = // metres
                        getDouble("drivebase/routine/balance/minDistance", 0.1);
                public static final double maxDistance = // metres
                        getDouble("drivebase/routine/balance/maxDistance", 1.2);
            }
        }


        /**
         * Spline / trajectory driving.
         */
        public static final class trajectory {

            public static final DifferentialDriveKinematics driveKinematics =
                    new DifferentialDriveKinematics(trackwidthMeters);

            // The Robot Characterization Toolsuite provides a convenient tool for obtaining
            // these values for your robot.
            public static final double ksVolts = 0.1602;
            public static final double kvVoltSecondsPerMeter = 2.3604;
            public static final double kaVoltSecondsSquaredPerMeter = 0.3;

            // PID values.
            public static final double kPDriveVel = 0.01; // should be 12.1
            // kD should be 0

            public static final double maxSpeedMetersPerSecond = 4;
            public static final double maxAccelerationMetersPerSecondSquared = 2;

            // Reasonable baseline values for a RAMSETE follower in units of meters and
            // seconds
            public static final double kRamseteB = 2;
            public static final double kRamseteZeta = 0.7;

            public static final double maxVoltage = 10;

            // Create a voltage constraint to ensure we don't accelerate too fast
            public static final TrajectoryConstraint autoVoltageConstraint =
                    new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(ksVolts,
                            kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter), driveKinematics,
                            maxVoltage);
        }
    }

    /**
     * timedAuto
     * 
     * Parameters for timed auto routines. Will be removed after path auto is functional.
     */
    public static class timedAuto {

        /**
         * Parameters for scoreConeThenIntake.
         * 
         * Scores a cone high goal, drives forward, intakes a cube, drives backward.
         */
        public static class scoreConeThenIntake {
            public static final double secondsOutward =
                    getDouble("drivebase/SecondsOutward", 5.0);
            public static final double secondsInward =
                    getDouble("drivebase/secondsInward", 2.0);

        }

        /**
         * Parameters for scoreConeThenBalance.
         * 
         * Scores a cone high goal, drives forward, then balances on charge station.
         */
        public static class scoreConeThenBalance {
            public static final double secondsToDrive = getDouble("secondsToDrive", 0.7425);
        }
    }

    /**
     * NavX
     * 
     * Using the gyro for autonomous routines.
     */
    public static class navx {
        public static final boolean enabled = getBoolean("navx/enabled", true);
    }

    public static class intake {
        public static final boolean enabled = getBoolean("intake/enabled", true);
        public static final int canID = getInt("intake/canID", 10);
        public static final int continuousCurrent = getInt("intake/continuousCurrent", 15);
        public static final int peakCurrent = getInt("intake/peakCurrent", 20);
        public static final int peakCurrentDuration = getInt("intake/peakCurrentDuration", 100);
        public static final double coneDutyCycle = getDouble("intake/coneDutyCycle", 1);
        public static final double cubeDutyCycle = getDouble("intake/cubeDutyCycle", 0.6);
        public static final double coneOutDutyCycle = getDouble("intake/coneOutDutyCycle", 0.5);
        public static final double cubeOutDutyCycle = getDouble("intake/cubeOutDutyCycle", 0.6);
        public static final double coneHoldDutyCycle = getDouble("intake/coneHoldDutyCycle", 0.3);
        public static final double cubeHoldDutyCycle = getDouble("intake/cubeHoldDutyCycle", 0.3);
    }

    // Encapsulates all constants for both the armPivot and armTelescope.
    // armTelescope needs different pivot PID/feedforward values due to the increased weight.
    public static class arm {
        public static final boolean enabled = getBoolean("arm/pivot/enabled", true);

        public static class pivot {
            public static final int[] canIds = getIntArray("arm/pivot/canIDs", new int[] {20, 21});

            // Can choose if the arm is correctly positioned to being with, or if we want it to run
            // the calibration routine.
            // If turning on calibration, make sure the limit switch works first!
            public static final boolean doCalibration =
                    getBoolean("arm/pivot/doCalibration", true);
            // Duty cycle to use when calibrating the arm.
            public static final double calibrationDutyCycle =
                    getDouble("arm/calibrationDutyCycle", 0.2);
            // Resting position in degrees of the arm once calibration is complete.
            // Currently this is over the top of the battery.
            public static final double calibrationAngle =
                    getDouble("arm/calibrationAngle", 205.0);

            // Current limiting.
            public static final double rampRate = getDouble("armPivot/pivot/rampRate", 0);
            public static final int continuousCurrent = getInt("arm/pivot/current/continuous", 10);
            public static final int peakCurrent = getInt("arm/pivot/current/peak", 20);
            public static final int peakCurrentDuration =
                    getInt("arm/pivot/current/peakDuration", 100);

            // Gearbox ratio for arm.
            // 45 turns of motor to 1 turn of output.
            private static final double versaReduction = 9.0;
            private static final double gearboxReduction = (50 / 14.0) * (50 / 14.0);
            private static final double chainReduction = 48 / 12.0;
            public static final double positionScale =
                    versaReduction * gearboxReduction * chainReduction;

            public static final double angleToleranceDegrees =
                    getInt("arm/pivot/angleToleranceDegrees", 4);

            public static final PIDF pidf = getPIDF("arm/pivot/pidf", new PIDF(40.0, 0, 0, 0));

            // These are for the feedforward. See
            // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html
            // Use https://www.reca.lc/arm to calculate the following values.
            public static class feedforward {
                // There are two sets of FF values. Depending on the arm extension, we pro-rata
                // between the two values.

                // Feedforward values to use with min arm extension.
                public static class min {
                    // The gravity gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kG = getDouble("arm/pivot/feedforward/min/kG", 0.5);
                    // The velocity gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kV = getDouble("arm/pivot/feedforward/min/kV", 4.0);
                    // The acceleration gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kA = getDouble("arm/pivot/feedforward/min/kA", 0.03);
                    // The static gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kS = getDouble("arm/pivot/feedforward/min/kS", 0.25);
                }
                // Feedforward values to use with max arm extension.
                public static class max {
                    // The gravity gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kG = getDouble("arm/pivot/feedforward/max/kG", 0.5);
                    // The velocity gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kV = getDouble("arm/pivot/feedforward/max/kV", 4.0);
                    // The acceleration gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kA = getDouble("arm/pivot/feedforward/max/kA", 0.03);
                    // The static gain. @see edu.wpi.first.math.controller.ArmFeedForward
                    public static final double kS = getDouble("arm/pivot/feedforward/max/kS", 0.25);
                }
            }

            // These define shape of the trapezoid controlling acceleration and max speed of the
            // arm.
            // NOTE: The arm mechanically maxes out at about 75-80 degrees/sec due to the gear ratio
            public static final double kMaxVelocity = getDouble("arm/kMaxVelocity", 120); // degrees/s
            // Take two seconds to get to max velocity.
            public static final double kMaxAcceleration = getDouble("arm/kMaxAcceleration", 100); // degrees/s/s


        }

        public static class telescope {
            public static final boolean enabled = getBoolean("arm/telescope/enabled", true);
            public static final int canID = getInt("arm/telescope/canID", 30);

            public static final PIDF telescopePidf =
                    getPIDF("arm/telescope/pidf", new PIDF(25, 0.025, 0, 0));

            // Motion magic config. Controls the shape of the profile generated as the arm moves.
            public static final double motionCruiseMetresPerSec =
                    getDouble("arm/telescope/motionCruiseMetresPerSec", 1);
            public static final double motionAccelerationMetresPerSecPerSec =
                    getDouble("arm/telescope/motionAccelerationMetresPerSecPerSec", 2);

            public static class feedforward {
                // This is in the range [-1, 1], not in volts like above.
                public static final double kG = getDouble("arm/telescope/feedforward/kG", 0.0);
            }

            public static final double potTicks = getDouble("arm/telescope/potTicks", 1);
            public static final double potSignalToMetres =
                    getDouble("arm/telescope/potSignalToMetres", 319 / 0.423);

            // Current limiting.
            public static final int continuousCurrent =
                    getInt("arm/telescope/current/continuous", 10);
            public static final int peakCurrent = getInt("arm/telescope/current/peak", 20);
            public static final int peakCurrentDuration =
                    getInt("arm/telescope/current/peakDuration", 100);

            // Extension preset positions.
            public static final double maxExtension = getDouble("arm/telescope/maxExtension", 0.4);
            public static final double minExtension = getDouble("arm/telescope/minExtension", 0.01);
            public static final double baseExtension =
                    getDouble("arm/telescope/baseExtension", 0.048281);

            // Total height limit of the robot as stated in the game manual (m)
            public static final double heightLimit = getDouble("arm/pivot/heightLimit", 1.8);
            // Height off the ground at which the pivot is fixed to the robot (m)
            public static final double pivotJointHeight = getDouble("arm/pivotJointHeight", 0.8);
            // Length of the arm minus the telescope (m)
            public static final double baseLength = getDouble("arm/baseLength", 0.93);
            // Tolerance for the telescope to check if it is in poisiton (m)
            public static final double positionErrorTolerance =
                    getDouble("arm/telescope/positionErrorTolerance", 0.03);
            // Angle which further extension won't extend to reduce damage to the intake via
            // contacting the ground
            public static final double minSafeExtensionAngle =
                    getDouble("arm/minSafeExtensionAngle", -25);
            // Angle which further extension won't extend to reduce damage to the battery via the
            // intake or the intake via the bumpers
            public static final double maxSafeExtensionAngle =
                    getDouble("arm/maxSafeExtensionnAngle", 188);
        }
        public static class position {
            public static final double stowedAngle = getDouble("arm/position/stowedAngle", 200.0);

            public static class cube {
                public static class intake {
                    public static final double angle =
                            getDouble("arm/position/cube/intake/angle", 199.0);
                    public static final double extension =
                            getDouble("arm/position/cube/intake/extension", 0.35);
                }
                public static class low {
                    public static final double angle =
                            getDouble("arm/position/cube/low/angle", 150.0);
                    public static final double extension =
                            getDouble("arm/position/cube/low/extension", 0.01);
                }
                public static class mid {
                    public static final double angle = getDouble("arm/position/cube/mid/angle", 7);
                    public static final double extension =
                            getDouble("arm/position/cube/mid/extension", 0.01);
                }
                public static class high {
                    public static final double angle =
                            getDouble("arm/position/cube/high/angle", 20.0);
                    public static final double extension =
                            getDouble("arm/position/cube/high/extension", 0.09);
                }

                // These positions are for when the arm is on the other side of the robot, allowing
                // us to score on either side.
                // TODO: Tune angles
                public static class backMid {
                    public static final double angle =
                            getDouble("arm/position/cube/backMid/angle", 173.0);
                    public static final double extension =
                            getDouble("arm/position/cube/backMid/extension", 0.01);
                }
                public static class backHigh {
                    public static final double angle =
                            getDouble("arm/position/cube/backHigh/angle", 160.0);
                    public static final double extension =
                            getDouble("arm/position/cube/backHigh/extension", 0.09);
                }
            }
            public static class cone {
                public static class intake {
                    public static final double angle =
                            getDouble("arm/position/cone/intake/angle", 183.0);
                    public static final double extension =
                            getDouble("arm/position/cone/intake/extension", 0.01);
                }
                public static class low {
                    public static final double angle =
                            getDouble("arm/position/cone/low/angle", 150.0);
                    public static final double extension =
                            getDouble("arm/position/cone/low/extension", 0.01);
                }
                public static class mid {
                    public static final double angle = getDouble("arm/position/cone/mid/angle", 32);
                    public static final double extension =
                            getDouble("arm/position/cone/low/extension", 0.01);
                }
                public static class high {
                    public static final double angle =
                            getDouble("arm/position/cone/high/angle", 30.0);
                    public static final double extension =
                            getDouble("arm/position/cone/high/extension", 0.29);
                }
            }
        }
    }

    /**
     * Location parameters
     *
     * Tracks current and historical position.
     */
    public static class location {
        public static class history {
            public static final int memorySecs = 5;
            public static final int cycleSpeedHz = 100;
        }
    }

    /**
     * PDP parameters
     * 
     * The motor controller wrappers also monitor current, so this is normally off.
     */
    public static class pdp {
        public static final boolean enabled = getBoolean("pdp/enabled", true);
        public static final int canId = getInt("pdp/canID", 0);
        // By default we do not monitor the PDP (CAN performance concerns)
        public static final boolean monitor = getBoolean("pdp/monitor", true);
        // By default we do not monitor any channels (motor controllers will also monitor)
        public static final int[] channels = getIntArray("pdp/channels", new int[0]);
    }

    /**
     * Pneumatic Control Module parameters
     */
    public static class pcm {
        public static final boolean enabled = getBoolean("pcm/enabled", false);
        public static final int canId = getInt("pcm/canID", 60);
    }

    public static class camera {
        public static final boolean enabled = getBoolean("camera/enabled", true);
    }

    /**
     * LED strip
     * 
     * Used to indicate the state of the robot (balls count, shoot count, issues).
     */
    public static class ledStrip {
        public static final boolean enabled = getBoolean("ledStrip/enabled", true);
        public static final int pwmPort = getInt("ledStrip/pwmPort", 0);
        public static final int numLEDs = getInt("ledStrip/numLEDs", 59);
        public static final int countdown = 15;
        public static final double brightnessPercentage = 0.4;
    }

    /**
     * Charting important values for post match debugging.
     */
    public static class charting {
        public static final boolean enabled = getBoolean("charting/enabled", true);
    }

    /**
     * These things are immutable
     */
    public static class constants {
        public static final double fullCircle = 360.0; // size of a full circle in internal units
                                                       // (degrees)
        public static final double halfCircle = 180.0; // size of a half circle in internal units
                                                       // (degrees)
        public static final double quarterCircle = 90.0; // size of a quarter circle in internal
                                                         // units (degrees)
        public static final double inchesToMetres = 0.0254;

    }

    /**
     * Update intervals
     */
    public static class intervals {
        public static final long executorCycleMSec = 20; // 50Hz
        public static final double dashboardUpdateSec = 0.5;
    }

    /**
     * Motor controller values
     */
    public static class motorController {
        public static final String talonSRX = "TalonSRX";
        public static final String sparkMAX = "SparkMAX";
        public static final String defaultType = talonSRX;

        /**
         * Current limits
         * 
         * Normally used by motor controllers
         */
        public static class currentLimit {
            public static final int defaultContinuousAmps = 30;
            public static final int defaultPeakAmps = 40;
        }
    }

    /**
     * Encoder values
     */
    public static class encoder {
        public static final double falconTicks = 2048; // Falon inbuilt encoders.
        public static final double SparkMAXTicks = 42; // SparkMAX inbuild encoders.
        public static final double s4tTicks = 1440; // ticks per rev.
        public static final double versaIntegratedTicks = 4096; // ticks per rotation
    }

    /**
     * User Interface
     */
    public static class ui {
        public static class joystick {
            // below this we deadband the value away
            public static final double deadbandMinValue = 0.05;
            // Should mock joysticks be created if joystick not plugged in?
            // Stops error messages when joysticks missing
            public static final boolean allowMock = getBoolean("ui/joystick/allowMock", false);
        }
    }

    /**
     * Robot constants
     */
    public static class robot {
        public static final double robotLength = 37.311 * constants.inchesToMetres;
        public static final double halfRobotLength = robotLength / 2;
    }

    /**
     * Server log sync parameters. Allows automatic uploading to a remote webserver.
     */
    public static class logging {
        public static final String flashDrive =
                Files.exists(Paths.get("/media/sda1")) ? "/media/sda1" : "/tmp";
        public static final String basePath = flashDrive; // log files (has to be inside web server)
        public static final String dataExtension = "data";
        public static final String dateExtension = "date";
        public static final String latestExtension = "latest";
        public static final String eventExtension = "event";

        public static class webserver {
            public static final String path = flashDrive; // where web server's data lives
            public static final int port = 5800; // port for graph/log web server
        }

        /**
         * Define parameters that govern the usage of the websocket logging server.
         */
        public static class liveloggingserver {
            public static final int port = 5803;
        }

        public static class rsync {
            // Port to forward to port 22 for transfering robot logs to pc over
            // rsync. Works around limited ports available while on the FMS.
            public static final int port = 5802;
            // Hostname of the robot.
            public static final String hostname = "roborio-3132-FRC.local";
        }
    }

    /**
     * Location on the roborio of the configuration file and config server details.
     */
    public static class config {
        public static final String homeDirectory = System.getProperty("user.home");
        public static final String configFilePath =
                Paths.get(homeDirectory, "config.txt").toString();
        public static final String robotNameFilePath =
                Paths.get(homeDirectory, "robotname.txt").toString();

        /**
         * Allow editing of config file via webserver.
         */
        public static class webserver {
            public static final String root = "/home/lvuser/deploy/www";
            public static final int port = 5801;
        }
    }

    // Only implementation from here onwards.

    private final static ConfigReader reader = new ConfigReader();

    /**
     * Needs to be called after the config is loaded to write out an example config
     * file and to print out details about the config file.
     */
    public static void finishLoadingConfig() {
        reader.finishLoadingConfig();
    }

    protected static String getMotorControllerType(final String parameterName,
            final String defaultValue) {
        return reader.getMotorControllerType(parameterName, defaultValue);
    }

    protected static int getInt(final String key, final int defaultValue) {
        return reader.getInt(key, defaultValue);
    }

    protected static double getDouble(final String key, final double defaultValue) {
        return reader.getDouble(key, defaultValue);
    }

    protected static PIDF getPIDF(final String prefix, final PIDF pidf) {
        return reader.getPIDF(prefix, pidf);
    }

    protected static boolean getBoolean(final String key, final boolean defaultValue) {
        return reader.getBoolean(key, defaultValue);
    }

    protected static String getString(final String key, final String defaultValue) {
        return reader.getString(key, defaultValue);
    }

    protected static int[] getIntArray(final String key, final int[] defaultValue) {
        return reader.getIntArray(key, defaultValue);
    }
}
