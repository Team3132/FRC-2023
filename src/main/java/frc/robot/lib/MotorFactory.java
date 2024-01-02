package frc.robot.lib;



import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Config;
import frc.robot.lib.chart.Chart;
import frc.robot.lib.log.Log;
import org.strongback.components.Clock;
import org.strongback.components.Motor;
import org.strongback.components.Motor.ControlMode;
import org.strongback.components.NetworkTableHelperImpl;
import org.strongback.components.PIDF;
import org.strongback.hardware.Hardware;
import org.strongback.hardware.HardwareSparkMAX;
import org.strongback.hardware.HardwareTalonSRX;

public class MotorFactory {

    public static Motor getDriveMotor(boolean leftMotor, Clock clock) {
        boolean invert = leftMotor ^ Config.drivebase.invert;
        int[] canIds = leftMotor ? Config.drivebase.canIdsLeftWithEncoders
                : Config.drivebase.canIdsRightWithEncoders;

        switch (Config.drivebase.motorControllerType) {
            case Config.motorController.sparkMAX: {
                HardwareSparkMAX spark = getSparkMAX("drive", canIds, invert,
                        NeutralMode.Brake,
                        Config.drivebase.pidf);
                spark.setScale(Config.encoder.SparkMAXTicks,
                        Config.drivebase.gearboxRatio,
                        Config.drivebase.metresPerRev);
                spark.setSensorPhase(Config.drivebase.sensorPhase);

                /*
                 * Setup Current Limiting
                 */
                if (Config.drivebase.currentLimiting) {
                    // Limit to 35 Amps when current exceeds 40 amps for 100ms
                    spark.setSmartCurrentLimit(Config.drivebase.contCurrent,
                            Config.drivebase.contCurrent);
                    spark.setSecondaryCurrentLimit(
                            Config.drivebase.peakCurrent);
                }
                return spark;
            }

            default:
                Log.error("MotorFactory",
                        "Invalid drive motor controller '%s'. Please use 'TalonSRX' or 'SparkMAX'. Using TalonSRX.",
                        Config.drivebase.motorControllerType);
                // Falling through to TalonSRX.

            case Config.motorController.talonSRX:
                HardwareTalonSRX talon = getTalon("drive", canIds, !invert,
                        NeutralMode.Brake,
                        Config.drivebase.pidf);
                talon.setScale(Config.encoder.falconTicks,
                        Config.drivebase.gearboxRatio,
                        Config.drivebase.metresPerRev);
                talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                        0, 10);
                talon.setSensorPhase(Config.drivebase.sensorPhase);
                talon.configClosedloopRamp(Config.drivebase.rampRate, 10);
                talon.configOpenloopRamp(Config.drivebase.rampRate, 10);
                talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 10);

                /*
                 * Setup Current Limiting
                 */
                if (Config.drivebase.currentLimiting) {
                    // limit to 35 Amps when current exceeds 40 amps for 100ms
                    talon.configContinuousCurrentLimit(
                            Config.drivebase.contCurrent, 0);
                    talon.configPeakCurrentLimit(Config.drivebase.peakCurrent,
                            0);
                    talon.configPeakCurrentDuration(100, 0);
                    talon.enableCurrentLimit(true);
                }
                return talon;
        }
    }

    public static HardwareTalonSRX getArmPivotMotor() {
        HardwareTalonSRX motor =
                getTalon("arm", Config.arm.pivot.canIds, true, NeutralMode.Coast,
                        new PIDF(0.0, 0.0, 0.0, 0.0));
        motor.setScale(Config.encoder.falconTicks, Config.arm.pivot.positionScale, 360);
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 10);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyClosed, 10);
        motor.configContinuousCurrentLimit(Config.arm.pivot.continuousCurrent, 10);
        motor.configPeakCurrentLimit(Config.arm.pivot.peakCurrent, 10);
        motor.configPeakCurrentDuration(Config.arm.pivot.peakCurrentDuration, 10);
        motor.enableCurrentLimit(true);
        return motor;
    }

    public static HardwareTalonSRX getTelescopeMotor() {
        HardwareTalonSRX motor = getTalon("telescope", Config.arm.telescope.canID, false,
                NeutralMode.Brake, Config.arm.telescope.telescopePidf);
        // Reads from a pot, no need for gearing
        motor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 10);
        motor.setScale(Config.arm.telescope.potTicks,
                Config.arm.telescope.potSignalToMetres);
        motor.configContinuousCurrentLimit(Config.arm.telescope.continuousCurrent, 10);
        motor.configPeakCurrentLimit(Config.arm.telescope.peakCurrent, 10);
        motor.configPeakCurrentDuration(Config.arm.telescope.peakCurrentDuration, 10);
        // Motion Magic parameters
        motor.configMotionCruiseVelocity(Config.arm.telescope.motionCruiseMetresPerSec, 10);
        motor.configMotionAcceleration(
                Config.arm.telescope.motionAccelerationMetresPerSecPerSec,
                10);
        motor.configAllowableClosedloopError(0, 1, 10);
        motor.set(ControlMode.Speed, 0);
        return motor;
    }

    public static HardwareTalonSRX getIntakeMotor() {
        HardwareTalonSRX motor =
                getTalon("intake", Config.intake.canID, true, NeutralMode.Coast,
                        new PIDF(0, 0, 0, 0));
        motor.configContinuousCurrentLimit(Config.intake.continuousCurrent, 10);
        motor.configPeakCurrentLimit(Config.intake.peakCurrent, 10);
        motor.configPeakCurrentDuration(Config.intake.peakCurrentDuration, 10);
        return motor;
    }

    /**
     * Code to allow us to log output current per Spark MAX and set up followers so that
     * it appears as a single motor but can be an arbitrary number of motors configured
     * in the per robot configuration.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canIDs list of canIDs. First entry is the leader and the rest follow it.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the leader HardwareTalonSRX
     */
    private static HardwareTalonSRX getTalon(String name, int[] canIDs, boolean invert,
            NeutralMode mode,
            PIDF pidf) {
        HardwareTalonSRX leader = Hardware.Motors.talonSRX(abs(canIDs[0]), invert, mode);
        Chart.register(() -> leader.getSupplyCurrent(), "Talons/%d/Current", canIDs[0]);
        leader.configContinuousCurrentLimit(
                Config.motorController.currentLimit.defaultContinuousAmps, 10);
        leader.configPeakCurrentLimit(Config.motorController.currentLimit.defaultPeakAmps,
                10);
        leader.enableCurrentLimit(true);
        leader.setPIDF(0, pidf);
        // Disable tuner to see if it helps with occasional lock-ups and reduce the number of CAN
        // errors in the logs.
        // TunableMotor.tuneMotor(leader, pidf, new NetworkTableHelperImpl(name));

        for (int n = 1; n < canIDs.length; n++) {
            boolean shouldInvert = invert;
            if (canIDs[n] < 0)
                shouldInvert = !shouldInvert;
            HardwareTalonSRX follower =
                    Hardware.Motors.talonSRX(abs(canIDs[n]), shouldInvert,
                            mode);
            follower.getHWTalon().follow(leader.getHWTalon());
            Chart.register(() -> follower.getSupplyCurrent(), "Talons/%d/Current",
                    canIDs[n]);
        }
        return leader;
    }

    /**
     * Code to allow us to log output current for a single talon.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canID CAN ID for this motor controller. Must be unique.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the HardwareTalonSRX motor controller.
     */
    private static HardwareTalonSRX getTalon(String name, int canID, boolean invert,
            NeutralMode mode, PIDF pidf) {
        Log.debug("MotorFactory", "%s: " + canID, "talon");
        int[] canIDs = new int[1];
        canIDs[0] = canID;
        return getTalon(name, canIDs, invert, mode, pidf);
    }

    /**
     * Code to allow us to log output current per Spark MAX and set up followers so that
     * it appears as a single motor but can be an arbitrary number of motors configured
     * 
     * in the per robot configuration.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canIDs list of canIDs. First entry is the leader and the rest follow it.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the leader SparkMAX
     */

    private static HardwareSparkMAX getSparkMAX(String name, int[] canIDs, boolean invert,
            NeutralMode mode, PIDF pidf) {
        HardwareSparkMAX leader =
                Hardware.Motors.sparkMAX(abs(canIDs[0]), MotorType.kBrushless,
                        invert);
        leader.setIdleMode(mode == NeutralMode.Brake ? IdleMode.kBrake : IdleMode.kCoast);
        Chart.register(() -> leader.getOutputCurrent(), "SparkMAX/%d/Current", canIDs[0]);
        leader.setSmartCurrentLimit(
                Config.motorController.currentLimit.defaultContinuousAmps, 10);
        leader.setSecondaryCurrentLimit(Config.motorController.currentLimit.defaultPeakAmps,
                10);
        TunableMotor.tuneMotor(leader, pidf, new NetworkTableHelperImpl(name));

        for (int n = 1; n < canIDs.length; n++) {
            boolean shouldInvert = invert;
            if (canIDs[n] < 0)
                shouldInvert = !shouldInvert;
            HardwareSparkMAX follower =
                    Hardware.Motors.sparkMAX(abs(canIDs[n]),
                            MotorType.kBrushless, shouldInvert);
            follower.getHWSpark().follow(leader.getHWSpark());
            Chart.register(() -> follower.getOutputCurrent(), "SparkMAX/%d/Current",
                    canIDs[n]);
        }
        return leader;
    }

    /**
     * Code to allow us to log output current for a single Spark MAX.
     * 
     * @param name the name to use for saving the PIDF values in the network tables.
     * @param canID CAN ID for this motor controller. Must be unique.
     * @param invert change the direction.
     * @param mode what to do when the the speed is set to zero.
     * @param pidf the P, I, D & F values to use.
     * @param log for registration of the current reporting.
     * @return the HardwareSparkMAX motor controller.
     */
    @SuppressWarnings("unused")
    private static HardwareSparkMAX getSparkMAX(String name, int canID, boolean invert,
            NeutralMode mode, PIDF pidf) {
        Log.debug("MotorFactory", "%s: " + canID, " spark max");
        int[] canIDs = new int[1];
        canIDs[0] = canID;
        return getSparkMAX(name, canIDs, invert, mode, pidf);
    }

    private static int abs(int value) {
        return value >= 0 ? value : -value;
    }
}
