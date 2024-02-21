package frc.robot.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class AutonomousConstants {
    public static final double MAXIMUM_SPEED_METRES_PER_SECOND = 4.8;
    public static final double MAXIMUM_ACCELERATION_METRES_PER_SECOND_SQUARED = 2;
    public static final double MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;
    public static final double MAXIMUM_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 5;

    public static final double PATH_TRANSLATION_P = 1;
    public static final double PATH_TRANSLATION_I = 0;
    public static final double PATH_TRANSLATION_D = 0;

    public static final double PATH_THETA_P = 1;
    public static final double PATH_THETA_I = 0;
    public static final double PATH_THETA_D = 0;

    public static final double X_P = 5;
    public static final double X_I = 0;
    public static final double X_D = 0;

    public static final double Y_P = X_P;
    public static final double Y_I = X_I;
    public static final double Y_D = X_D;

    public static final double THETA_P = 5;
    public static final double THETA_I = 0;
    public static final double THETA_D = 0;

    public static final double TRANSLATION_TOLERANCE = 0.02;
    public static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND,
        MAXIMUM_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED
    );

    public static final HolonomicPathFollowerConfig pathFollowConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(PATH_TRANSLATION_P, PATH_TRANSLATION_I, PATH_TRANSLATION_D),
        new PIDConstants(PATH_THETA_P, PATH_THETA_I, PATH_THETA_D),
        MAXIMUM_SPEED_METRES_PER_SECOND,
        DriveConstants.CHASSIS_RADIUS,
        new ReplanningConfig()
    );
}