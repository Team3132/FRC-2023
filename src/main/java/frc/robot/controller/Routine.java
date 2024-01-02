package frc.robot.controller;



import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Iterator;

/**
 * A list of State instances that the robot should go through and an end state
 * for when the routine is interrupted.
 * The final state is where the robot will remain once it has done all states.
 * Can be aborted at any time.
 */
public class Routine implements Iterable<Step> {
    private final String name;
    private final ArrayList<Step> states;
    private final Step endState;
    private final EnumSet<Domain> domains;

    private Routine(RoutineBuilder builder) {
        this.name = builder.name;
        this.endState = builder.endState;
        this.states = builder.states;
        this.domains = builder.domains;
    }

    public String getName() {
        return name;
    }

    public Iterator<Step> iterator() {
        return states.iterator();
    }

    public Step getEndState() {
        return endState;
    }

    /**
     * A routine conflicts with another routine if they share any
     * domains/subsystems that are updated.
     * 
     * @param other the other routine to compare it agains.
     * @return true if they share domains
     */
    public boolean doesConflict(Routine other) {
        EnumSet<Domain> copy = domains.clone();
        copy.retainAll(other.domains);
        return copy.size() > 0;
    }

    /**
     * A list of State instances that the robot should go through.
     * Can be aborted at any time.
     * 
     * Every add() call adds a new state that the robot should achieve
     * before moving to the next state. The final state is where the robot
     * will remain once it has done all states.
     * 
     * Any value not set will cause it to not be changed. eg if we don't set the
     * lift height, the height will be unchanged.
     * 
     * An end state can be created to be applied if the routine is interrupted,
     * either autocreated by passing createInterrupt = true
     * or manually set by calling onInterrupt();
     * 
     * Example usage for intaking a cube
     * RoutineBuilder builder = new RoutineBuilder("Intake cube", createInterrupt);
     * builder.add().setLiftHeight(0).setIntakeConfig(NARROW).setOuttakeOpen(true);
     * builder.add().setIntakeMotorOutput(1).setOuttakeHasCube(true);
     * builder.add().setOuttakeOpen(false);
     * builder.add().setIntakeMotorOutput(0).setIntakeConfig(STOWED);
     * return builder.build();
     */
    public static class RoutineBuilder {
        private final String name;
        private boolean createInterrupt = false;
        private ArrayList<Step> states = new ArrayList<Step>();
        private Step endState = new Step();
        private EnumSet<Domain> domains = EnumSet.noneOf(Domain.class);

        /**
         * RoutineBuilder create a new routine.
         * 
         * @param name The name of this routine.
         */
        public RoutineBuilder(String name) {
            this.name = name;
        }

        /**
         * Creates a special State to be run when the routine is interrupted that
         * is applied to leave the robot in the state it would have been if the routine
         * had been allowed to continue.
         */
        public RoutineBuilder createInterruptState() {
            createInterrupt = true;
            return this;
        }

        /**
         * Adds a new state at the end of the routine.
         * 
         * @return new state
         */
        public Step then() {
            states.add(new Step());
            return states.get(states.size() - 1);
        }

        /**
         * Adds a new state at the end of the routine and log a message.
         * 
         * @param debug Message to log.
         * @return new state
         */
        public Step then(String debug) {
            return then().setLog(debug);
        }

        /**
         * Add another routine to the end of this one, only copying the states.
         * 
         * @param other the routine to add.
         */
        public void appendRoutine(Routine other) {
            states.addAll(other.states);
        }

        /**
         * Get the final state when the routine gets interrupted
         */
        public Step onInterrupt() {
            return endState;
        }

        public Routine build() {
            for (Step s : states) {
                domains.addAll(s.getDomains());
            }
            if (createInterrupt) {
                for (Step s : states) {
                    endState.fillInterrupt(s);
                }
            }
            return new Routine(this);
        }


    }
}
