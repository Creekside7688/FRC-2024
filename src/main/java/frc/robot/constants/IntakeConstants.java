package frc.robot.constants;

import com.revrobotics.CANSparkBase.IdleMode;

public class IntakeConstants {
    public static final int MOTOR_ID = 9;
    public static final int CURRENT_LIMIT = 30;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int SENSOR_CHANNEL = 0;

    public static final double INTAKE_GEAR_RATIO = 10;

    public static final double AMP_SPEED = -1.0;
    public static final double AMP_DURATION = 2.5;

    public static final double PICKUP_SPEED = 0.8;

    public static final double SHOOTER_FEED_SPEED = 1;
    public static final double SHOOTER_FEED_DURATION = 0.75;
    public static final double SHOOTER_FEED_DELAY = 2;

    public static final double P = 0.1;
}