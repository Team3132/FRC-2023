package frc.robot.controller;

/**
 * A list of domains for conflict checking.
 * 
 * Normally each subsystem is its own domain, but subsystems that interfere with
 * each other should be in the same domain so they can be safely updated together
 * without harming the robot.
 * 
 * For example if the carriage and the lift can impact each other, then they should
 * be included in the same domain so that routines that use one or the other will
 * conflict and abort the earlier routine.
 * 
 * This is key to being able to run multiple routines at the same time.
 * 
 * Note there is no time domain as two routines using different delays aren't
 * going to conflict with each other.
 */
enum Domain {
    DRIVEBASE, ARM, INTAKE, LED
    // Note the lack of non-subsystems, eg time.
};
