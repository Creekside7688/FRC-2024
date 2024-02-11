package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;

public class ModuleConstants {
    public static final double NEO_FREE_SPEED_RPM = 5676;
    // Modules can have 12T, 13T, or 14T pinions. More teeth means it will drive faster.
    public static final int DRIVE_PINION_TEETH = 12;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of the steering motor in the MAXSwerve Module.
    public static final boolean TURN_ENCODER_INVERTED = true;

    public static final double DRIVE_MOTOR_FREE_SPEED_RPS = NEO_FREE_SPEED_RPM / 60; // Revolutions per second
    public static final double WHEEL_DIAMETER_METRES = Units.inchesToMeters(3);
    public static final double WHEEL_CIRCUMFERENCE_METRES = WHEEL_DIAMETER_METRES * Math.PI;

    // Bevel 45T, First Stage 22T, Bevel Pinion 15T
    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED = (DRIVE_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METRES) / DRIVE_MOTOR_REDUCTION;

    public static final double DRIVE_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METRES * Math.PI) / DRIVE_MOTOR_REDUCTION; // Metres
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = DRIVE_ENCODER_POSITION_FACTOR / 60.0; // Metres per second

    public static final double TURN_ENCODER_POSITION_FACTOR = (2 * Math.PI); // Radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // Radians per second

    public static final double TURN_PID_MINIMUM_INPUT = 0; // Radians
    public static final double TURN_PID_MAXIMUM_INPUT = TURN_ENCODER_POSITION_FACTOR; // Radians

    public static final double DRIVE_P = 0.04;
    public static final double DRIVE_I = 0;
    public static final double DRIVE_D = 0;
    public static final double DRIVE_FF = 1 / DRIVE_WHEEL_FREE_SPEED;

    public static final double DRIVE_MINIMUM_OUTPUT = -1;
    public static final double DRIVE_MAXIMUM_OUTPUT = 1;

    public static final double TURN_P = 1;
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