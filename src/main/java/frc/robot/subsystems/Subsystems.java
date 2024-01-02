package frc.robot.subsystems;



import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Config;
import frc.robot.interfaces.*;
import frc.robot.lib.LEDColour;
import frc.robot.lib.MotorFactory;
import frc.robot.lib.NavXGyroscope;
import frc.robot.mock.*;
import org.strongback.Executor.Priority;
import org.strongback.Strongback;
import org.strongback.components.Clock;
import org.strongback.components.Gyroscope;
import org.strongback.components.Motor;
import org.strongback.components.PneumaticsModule;
import org.strongback.components.ui.InputDevice;
import org.strongback.hardware.Hardware;
import org.strongback.mock.Mock;

/**
 * Contains the subsystems for the robot.
 * 
 * Makes it easy to pass all subsystems around.
 */
public class Subsystems implements DashboardUpdater, LogHelper {
    // Not really a subsystem, but used by all subsystems.
    public Clock clock;
    public LEDStrip ledStrip;
    public Location location;
    public Drivebase drivebase;
    public SingleMotor intake;
    public Arm arm;
    public PneumaticsModule pcm;
    public Monitor monitor;
    public InputDevice gamepad;

    public Subsystems(Clock clock, InputDevice gamepad) {
        this.clock = clock;
        this.gamepad = gamepad;
    }

    public void createOverrides() {}

    public void enable() {
        info("Enabling subsystems");
        gamepad.setRumbleLeft(0);
        gamepad.setRumbleRight(0);
        drivebase.enable();
        arm.enable();
        intake.enable();
    }

    public void disable() {
        info("Disabling Subsystems");
        gamepad.setRumbleLeft(0);
        gamepad.setRumbleRight(0);
        drivebase.disable();
        arm.disable();
        intake.disable();
    }

    @Override
    public void updateDashboard() {
        drivebase.updateDashboard();
        arm.updateDashboard();
        intake.updateDashboard();
        location.updateDashboard();
        monitor.updateDashboard();
    }

    /**
     * Create the drivebase subsystem. Creates the motors.
     */
    public void createDrivebase() {
        if (!Config.drivebase.enabled) {
            debug("Using mock drivebase");
            drivebase = new MockDrivebase();
            return;
        }
        Motor leftMotor = MotorFactory.getDriveMotor(true, clock);
        Motor rightMotor = MotorFactory.getDriveMotor(false, clock);

        leftMotor.resetEncoder(0);
        rightMotor.resetEncoder(0);
        try {
            // Let the encoders get the message and have time to send it back to us.
            Thread.sleep(100);
        } catch (InterruptedException e) {
        }
        error("Reset drive encoders to zero, currently are: %f, %f", leftMotor.getPosition(),
                rightMotor.getPosition());
        // metres.
        drivebase =
                new DrivebaseImpl(leftMotor, rightMotor);
        Strongback.executor().register(drivebase, Priority.HIGH);
    }

    /**
     * Create the location subsystem. Creates the gyro.
     */
    public void createLocation() {
        if (!Config.drivebase.enabled) {
            debug("No drivebase, using mock location");
            location = new MockLocation();
            return;
        }
        Gyroscope gyro = new NavXGyroscope("NavX", Config.navx.enabled);
        gyro.zero();
        // Encoders must return metres.
        location = new LocationImpl(drivebase, gyro, clock);
        Strongback.executor().register(location, Priority.HIGH);
    }

    public void createIntake() {
        if (!Config.intake.enabled) {
            intake = new MockSingleMotor("mockIntake");
            debug("Intake not enabled, using a mock intake instead");
            return;
        }

        Motor intakeMotor = MotorFactory.getIntakeMotor();
        intake = new SingleMotorImpl("intake", intakeMotor);
    }

    public void createArm() {
        if (!Config.arm.enabled) {
            arm = new MockArm();
            debug("Created a mock arm!");
            return;
        }
        Motor pivotMotor = MotorFactory.getArmPivotMotor();
        Motor telescopeMotor = MotorFactory.getTelescopeMotor();
        arm = new ArmImpl(pivotMotor, telescopeMotor);
        Strongback.executor().register(arm, Priority.HIGH);
    }


    public void createLEDStrip() {
        if (!Config.ledStrip.enabled) {
            ledStrip = new MockLEDStrip();
            debug("LED Strip not enabled, using a mock LED Strip instead.");
            return;
        }
        ledStrip = new LEDStripImpl(Config.ledStrip.pwmPort, Config.ledStrip.numLEDs);
    }

    public void createMonitor() {
        monitor = new MonitorImpl();
    }

    public void setLEDFinalCountdown(double time) {
        ledStrip.setProgressColour(LEDColour.RED, LEDColour.GREEN,
                1 - (time / Config.ledStrip.countdown));
    }

    /**
     * Create the Pneumatics Control Module (PCM) subsystem.
     */
    public void createPneumatics() {
        if (!Config.pcm.enabled) {
            pcm = Mock.pneumaticsModule(Config.pcm.canId);
            debug("Created a mock compressor");
            return;
        }
        pcm = Hardware.pneumaticsModule(Config.pcm.canId, PneumaticsModuleType.CTREPCM);
    }

    @Override
    public String getName() {
        return "Subsystems";
    }
}
