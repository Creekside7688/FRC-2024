package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
    // Maximum allowed speeds.
    public static final double MAXIMUM_SPEED_METRES_PER_SECOND = 4.8;
    public static final double MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;

    public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
    public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
    public static final double ROTATION_SLEW_RATE = 2.0; // percent per second (1 = 100%)

    public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); // Distance between left and right wheels on robot
    public static final double WHEEL_BASE = TRACK_WIDTH; // Distance between front and back wheels on robot

    public static final double CHASSIS_RADIUS = Units.inchesToMeters(
        (TRACK_WIDTH / 2) / Math.sin(45 * (Math.PI / 180))
    );

    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    // Angular offset relative to chassis in radians.
    public static final double FL_OFFSET = -Math.PI / 2;
    public static final double FR_OFFSET = 0;
    public static final double BL_OFFSET = Math.PI;
    public static final double BR_OFFSET = Math.PI / 2;

    public static final int FL_DRIVE_MOTOR = 1;
    public static final int FR_DRIVE_MOTOR = 3;
    public static final int BL_DRIVE_MOTOR = 5;
    public static final int BR_DRIVE_MOTOR = 7;

    public static final int FL_TURN_MOTOR = 2;
    public static final int FR_TURN_MOTOR = 4;
    public static final int BL_TURN_MOTOR = 6;
    public static final int BR_TURN_MOTOR = 8;

    public static final boolean GYRO_INVERTED = true;

    public static final double SNAP_P = 0.01;
    public static final double SNAP_I = 0;
    public static final double SNAP_D = 0;

    public static final double FLIP_P = 0.01;
    public static final double FLIP_I = 0;
    public static final double FLIP_D = 0;
}