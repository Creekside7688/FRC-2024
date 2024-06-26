package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

public class ElevatorConstants {

    public static final int MOTOR_ID = 11;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final int CURRENT_LIMIT = 50;

    public static final double GEAR_RATIO = 5.0;
    public static final double SPOOL_DIAMETER = 1.440; // Inches

    public static final boolean ENCODER_INVERTED = false;
    public static final double ENCODER_POSITION_FACTOR = (SPOOL_DIAMETER * Math.PI) / GEAR_RATIO;
    public static final double ENCODER_VELOCITY_FACTOR = ENCODER_POSITION_FACTOR / 60.0;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double PID_MINIMUM_OUTPUT = -1.0;
    public static final double PID_MAXIMUM_OUTPUT = 1.0;

    public static final double S = 0.15944; // Volts
    public static final double G = 0.68283; // Volts
    public static final double V = 0.076034; // Volts times seconds per inch
    public static final double A = 0.022109; // Volts times seconds squared per inch

    public static final double MAX_VELOCITY = 10; // Inches per second
    public static final double MAX_ACCELERATION = 5; // Inches per second squared. Assuming we can accelerate to max velocity in 0.1 seconds.

    // Inches

    public static final double MAX_HEIGHT = 28.0;
    public static final double TARGET_HEIGHT = 28.0;
    public static final double TOLERANCE = 0.1;


    public static final double TEMP_MAX_HEIGHT = 5.0;

    public static final double MOTOR_SLOWRAISE_SPEED = 0.75;
    public static final double MOTOR_SLOWRAISE_STALLSPEED = 0.02;

    public static final double MOTOR_SLOWFALL_SPEED = -0.35;
    public static final double MOTOR_CLIMB_SPEED = -1;

    public static final double MOTOR_MAX_STEPS = 0.3781;

    public static final double MOTOR_MIN_STEPS = 0;

    public static final double MOTOR_TEMP_STEPS = 6;
    public static final double MOTOR_TEMPDROP_SPEED = -0.1;
    public static final double MOTOR_TEMPDROP_DELAY = 0.5;
}

