package org.strongback.components.ui;



import java.util.function.BooleanSupplier;
import org.strongback.Strongback;
import org.strongback.SwitchReactor;

public class Trigger {
    /**
     * Runs a routine when a condition becomes true.
     */
    private String device;
    private String name;
    private BooleanSupplier value;
    SwitchReactor reactor = Strongback.switchReactor();
    private boolean toggled = false;

    public Trigger(String device, String name, BooleanSupplier func) {
        this.device = device;
        this.name = name;
        this.value = func;
    }

    /**
     * Runs supplied function when pressed.
     * 
     * @param func The function to run.
     */
    public void onPress(Runnable func) {
        log(func, "press");
        reactor.onTriggered(() -> value.getAsBoolean(), func);
    }

    /**
     * Runs supplied function while triggered.
     * 
     * @param func The function to run.
     */
    public void whileTriggered(Runnable func) {
        log(func, "while");
        reactor.whileTriggered(() -> value.getAsBoolean(), func);
    }

    /**
     * Runs supplied function when untriggered
     * 
     * @param func The function to run.
     */
    public void onRelease(Runnable func) {
        log(func, "release");
        reactor.onUntriggered(() -> value.getAsBoolean(), func);
    }

    /**
     * Create a toggle switch based on a single button.
     * 
     * <pre>
     * {@code
     * stick.button(6).onToggle("climber", Routines.deployClimber(), Routines.retractClimber());
     * }
     * </pre>
     * 
     * @param name used for logging when the toggle changes.
     * @param on routine to run when the toggle is triggered on.
     * @param off routine to run when the toggle is triggered off.
     * @return the ToggleSwitch for further chaining of more buttons based on the
     *         toggle state.
     */
    public void onToggle(String toggleName, Runnable on, Runnable off) {
        log(on, "toggle on");
        log(off, "toggle off");
        reactor.onTriggered(() -> value.getAsBoolean(), () -> {
            if (!toggled) {
                on.run();;
            } else {
                off.run();
            }
            toggled = !toggled;
        });
    }

    static boolean firstRun = true;

    public void log(Runnable what, String verb) {
        // Assumes that Runnable.toString() has been overridden with something useful.
        if (firstRun) {
            log("Action/Routine", "Joystick", "Action", "Button");
            log("--------------", "--------", "------", "------");
            firstRun = false;
        }
        log(what.toString(), device, verb, name);
    }

    public void log(String what, String device, String action, String button) {
        Strongback.logger()
                .info(String.format("%40s | %14s | %10s | %s", what, device, action, button));
    }
}
