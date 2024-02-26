package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class VisionConstants {

    /** Physical location of the apriltag camera on the robot, relative to the center of the robot. */
    public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0, 0)
    );

    public static final String LIMELIGHT_NAME = "limelight";

    public static final double FIELD_LENGTH_METERS = 16.54175;
    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite alliance
    public static final Pose2d FLIPPING_POSE = new Pose2d(
        new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI)
    );

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

    public static final double XP = 3.0;
    public static final double XI = 0.0;
    public static final double XD = 0.0;

    public static final double YP = 3.0;
    public static final double YI = 0.0;
    public static final double YD = 0.0;

    public static final double TP = 2.0;
    public static final double TI = 0.0;
    public static final double TD = 0.0;

    
    public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    public static final TrapezoidProfile.Constraints T_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    public static final double POSITION_CONTROLLER_TOLERANCE = 0.2;
    public static final double ROTATION_CONTROLLER_TOLERANCE = 3;

    public static final int RED_AMP = 5;
    public static final int BLUE_AMP = 6;
    public static final Transform3d AMP_DISTANCE = new Transform3d(
        new Translation3d(1.5, 0.0, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI)
    );

    public static final int RED_SPEAKER = 4;
    public static final int BLUE_SPEAKER = 7;
    public static final Transform3d SPEAKER_DISTANCE = new Transform3d(
        new Translation3d(6, 4, 0.0),
        new Rotation3d(0.0, 0.0, Math.PI)
    );
}
