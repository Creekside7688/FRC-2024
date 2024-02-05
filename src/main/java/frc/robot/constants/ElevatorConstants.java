package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

public class ElevatorConstants {

    public static final int MOTOR_ID = 9;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final int CURRENT_LIMIT = 50;

    public static final boolean ENCODER_INVERTED = false;
    public static final double ENCODER_POSITION_FACTOR = 5.0;
    public static final double ENCODER_VELOCITY_FACTOR = 5.0 / 60.0;

    public static final double P = 0.04;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double PID_MINIMUM_OUTPUT = -1.0;
    public static final double PID_MAXIMUM_OUTPUT = 1.0;


    // Gains taken from Recalc, we will use SysID later to find more accurate ones.

    public static final double S = 0.5; // Volts
    public static final double G = 0.89; // Volts
    public static final double V = 0.14; // Volts times seconds per inch
    public static final double A = 0.004; // Volts times seconds squared per inch

    public static final double MAX_VELOCITY = 75; // Inches per second
    public static final double MAX_ACCELERATION = 50; // Inches per second squared

    public static final double MIN_HEIGHT = 0.0;
    public static final double MAX_HEIGHT = 28.0;

    public static final double SHOOTER_HEIGHT = 0.0;
    public static final double AMP_HEIGHT = 28.0;
    public static final double TOLERANCE = 0.1;
}
