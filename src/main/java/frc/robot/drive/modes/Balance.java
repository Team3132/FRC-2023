package frc.robot.drive.modes;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Config;
import frc.robot.interfaces.DashboardUpdater;
import frc.robot.interfaces.DriveTelemetry;
import frc.robot.interfaces.Location;
import frc.robot.lib.chart.Chart;
import org.strongback.Executable;
import org.strongback.components.Clock;

/**
 * Drives the robot onto the charge station (like a wide seesaw) and then balances the robot such
 * that the charge station is level.
 */
public class Balance extends AutoDriveBase implements Executable, DashboardUpdater {

    private enum State {
        FLOOR(0), // Not yet on the ramp
        BALANCING(2), // On the ramp which is not level
        ENTERING(3), // comment
        LEVEL(4); // On the ramp which is levels

        public final int id;

        State(int id) {
            this.id = id;
        }
    }

    private State state = State.FLOOR;

    private DriveTelemetry telemetry;
    private Location location;
    private double startPosition;
    private double travelled = 0; // Distance travelled since entering ramp
    private String lastLog = ""; // Last log message, used to prevent spamming the log
    private double maxAngle = 0;
    private double sign = 0;
    private double speed = 0;
    private double rampStartTime = 0;
    private double slowDown = 1.0;
    private boolean forward = false;

    public Balance(DriveTelemetry telemetry, Location location, Clock clock) {
        super("Balance", telemetry, clock);
        this.telemetry = telemetry;
        this.location = location;

        Chart.register(() -> state.id, "%s/state", name);
        Chart.register(() -> travelled, "%s/travelled", name);
        Chart.register(
                () -> maxAngle - Config.drivebase.routine.balance.approachingZeroToleranceAngle,
                "%s/motorStopThreshold", name);
        Chart.register(() -> Math.min(5, Clock.system().currentTime() - rampStartTime),
                "%s/balanceTimer", name);
    }

    @Override
    public void reset(Parameters parameters) {
        super.reset(parameters);
        startPosition = 0;
        travelled = 0;
        lastLog = "";
        maxAngle = 0;
        sign = 0;
        slowDown = 1.0;
        state = State.FLOOR;
        forward = parameters.forward;
    }

    @Override
    public double getTargetSpeed() {
        return speed;
    }

    private boolean isLevel(double angle) {
        return Math.abs(angle) < Config.drivebase.routine.balance.toleranceAngle;
    }

    private void infoOnce(String message) {
        if (lastLog == message)
            return;
        lastLog = message;
        info(message);
    }

    @Override
    public double getTargetTurn() {
        return 0;
    }

    @Override
    public boolean hasFinished() {
        return state == State.LEVEL;
    }

    @Override
    public void execute(long timeInMillis) {
        travelled = Math.abs(telemetry.getLeftDistance() - startPosition);
        double angle = location.getPitch();

        switch (state) {
            case FLOOR:
                if (Math.abs(angle) > 2
                        * Config.drivebase.routine.balance.toleranceAngle) {
                    infoOnce("On ramp");
                    state = State.ENTERING;
                    startPosition = telemetry.getLeftDistance();
                    rampStartTime = Clock.system().currentTime();
                }
                speed = Config.drivebase.routine.balance.entrySpeed * (forward ? 1 : -1);
                break;
            case ENTERING:
                if (Clock.system().currentTime() - rampStartTime > 1.7) {
                    infoOnce("Entered, balancing now");
                    state = State.BALANCING;
                }
                break;
            case BALANCING:
                if (Math.signum(angle) != sign) {
                    maxAngle = 0;
                    sign = Math.signum(angle);
                    slowDown = Math.max(0.45, slowDown * 0.9);
                    infoOnce("Changed slowDown to " + slowDown);
                }
                maxAngle = Math.max(Math.abs(angle), maxAngle);

                // if new angle is closer to 0 by more than tolerance, then we are approaching
                // level, so early stop
                if (maxAngle - Math.abs(
                        angle) > Config.drivebase.routine.balance.approachingZeroToleranceAngle) {
                    infoOnce("Approaching level");
                    speed = 0;
                } else if (isLevel(angle)) {
                    infoOnce("Level");
                    state = State.LEVEL;
                } else if (angle < 0) {
                    if (travelled < Config.drivebase.routine.balance.minDistance) {
                        infoOnce("Angle < 0, Travelled < min distance");
                        speed = 0;
                    } else {
                        infoOnce("Angle < 0, Travelled > min distance");
                        speed = Config.drivebase.routine.balance.balanceSpeed * slowDown;
                    }
                } else {
                    // angle > 0
                    if (travelled > Config.drivebase.routine.balance.maxDistance) {
                        infoOnce("Angle > 0, Travelled > max distance");
                        speed = 0;
                    } else {
                        infoOnce("Angle > 0, Travelled < max distance");
                        speed = -Config.drivebase.routine.balance.balanceSpeed * slowDown;
                    }
                }
                break;
            case LEVEL:
                if (isLevel(angle)) {
                    speed = 0;
                } else {
                    infoOnce("No longer level");
                    state = State.BALANCING;
                }
                break;
        }
    }

    @Override
    public void updateDashboard() {
        SmartDashboard.putString("Balance State", state.name());
        SmartDashboard.putNumber("Balance Distance", travelled);
    }
}
