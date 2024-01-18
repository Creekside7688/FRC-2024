package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        // Maximum allowed speeds.
        public static final double MAXIMUM_SPEED_METRES_PER_SECOND = 4.8;
        public static final double MAXIMUM_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * Math.PI;

        public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
        public static final double ROTATION_SLEW_RATE = 2.0; // percent per second (1 = 100%)

        public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); // Distance between left and right wheels on robot
        public static final double WHEEL_BASE = TRACK_WIDTH; // Distance between front and back wheels on robot

        public static final double CHASSIS_RADIUS = Units.inchesToMeters(
            (TRACK_WIDTH / 2) / Math.sin(45 * (Math.PI / 180)));

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        // Angular offset relative to chassis in radians.
        public static final double FL_OFFSET = -Math.PI / 2;
        public static final double FR_OFFSET = 0;
        public static final double BL_OFFSET = Math.PI;
        public static final double BR_OFFSET = Math.PI / 2;

        public static final int FL_DRIVE_MOTOR = 3;
        public static final int FR_DRIVE_MOTOR = 5;
        public static final int BL_DRIVE_MOTOR = 1;
        public static final int BR_DRIVE_MOTOR = 7;

        public static final int FL_TURN_MOTOR = 4;
        public static final int FR_TURN_MOTOR = 6;
        public static final int BL_TURN_MOTOR = 2;
        public static final int BR_TURN_MOTOR = 8;

        public static final boolean GYRO_INVERTED = true;

        public static final double SNAP_P = 0.01;
        public static final double SNAP_I = 0;
        public static final double SNAP_D = 0;

        public static final double FLIP_P = 0.01;
        public static final double FLIP_I = 0;
        public static final double FLIP_D = 0;
    }

    public static final class ModuleConstants {
        // Modules can have 12T, 13T, or 14T pinions. More teeth means it will drive faster.
        public static final int DRIVE_PINION_TEETH = 12;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor in the MAXSwerve Module.
        public static final boolean TURN_ENCODER_INVERTED = true;

        public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NeoConstants.FREE_SPEED_RPM / 60; // Revolutions per second
        public static final double WHEEL_DIAMETER_METRES = Units.inchesToMeters(3);
        public static final double WHEEL_CIRCUMFERENCE_METRES = WHEEL_DIAMETER_METRES * Math.PI;

        // Bevel 45T, First Stage 22T, Bevel Pinion 15T
        public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METRES) / DRIVE_MOTOR_REDUCTION;

        public static final double DRIVE_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METRES * Math.PI) / DRIVE_MOTOR_REDUCTION; // Metres
        public static final double DRIVE_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METRES * Math.PI) / DRIVE_MOTOR_REDUCTION) / 60.0; // Metres per second

        public static final double TURN_ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians
        public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // Radians per second

        public static final double TURN_PID_MINIMUM_INPUT = 0; // Radians
        public static final double TURN_PID_MAXIMUM_INPUT = TURN_ENCODER_POSITION_FACTOR; // Radians

        public static final double DRIVE_P = 0.04;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;

        public static final double DRIVE_MINIMUM_OUTPUT = -1;
        public static final double DRIVE_MAXIMUM_OUTPUT = 1;

        public static final double TURN_P = 0.75;
        public static final double TURN_I = 0;
        public static final double TURN_D = 0;
        public static final double TURN_FF = 0;
        public static final double TURN_MINIMUM_OUTPUT = -1;
        public static final double TURN_MAXIMUM_OUTPUT = 1;

        public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode TURN_IDLE_MODE = IdleMode.kBrake;

        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50; // Amps
        public static final int TURN_MOTOR_CURRENT_LIMIT = 20; // Amps
    }

    public static final class OperatorConstants {
        public static final int CONTROLLER_PORT = 0;
        public static final double DEADBAND = 0.15;
        public static final double OFFSET = 0.1;
    }

    public static final class AutonomousConstants {
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

    public static final class NeoConstants {
        public static final double FREE_SPEED_RPM = 5676;
    }
}
