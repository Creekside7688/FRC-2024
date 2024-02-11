package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

public class ElevatorConstants {

    public static final int MOTOR_ID = 11;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final int CURRENT_LIMIT = 50;

    public static final double GEAR_RATIO = 1.0;

    public static final boolean ENCODER_INVERTED = false;
    public static final double ENCODER_POSITION_FACTOR = 1.0;
    public static final double ENCODER_VELOCITY_FACTOR = 1.0 / 60.0;

    public static final double P = 0.04;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double PID_MINIMUM_OUTPUT = -1.0;
    public static final double PID_MAXIMUM_OUTPUT = 1.0;

    public static final double S = 0.0;
    public static final double G = 0.0;
    public static final double V = 0.0;
    public static final double A = 0.0;

    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 1.0;

    public static final double MAX_HEIGHT = 28.0;
    public static final double TARGET_HEIGHT = 28.0;
    public static final double TOLERANCE = 0.1;
}
