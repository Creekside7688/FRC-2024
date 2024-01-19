package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutonomousConstants {
    public static final double MAXIMUM_SPEED_METRES_PER_SECOND = 4.8;
    public static final double MAXIMUM_ACCELERATION_METRES_PER_SECOND_SQUARED = 2;
    public static final double MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double MAXIMUM_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 5;

    public static final double TRANSLATION_P = 1;
    public static final double ROTATION_P = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAXIMUM_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

    public static final HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(TRANSLATION_P, 0, 0),
        new PIDConstants(ROTATION_P, 0, 0),
        MAXIMUM_SPEED_METRES_PER_SECOND,
        DriveConstants.CHASSIS_RADIUS,
        new ReplanningConfig());
}