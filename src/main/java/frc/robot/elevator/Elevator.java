package frc.robot.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    private final Constraints constraints;
    private TrapezoidProfile.State setpoint;
    private TrapezoidProfile.State goal;

    private final SparkPIDController feedbackController;
    private final ElevatorFeedforward feedforwardController;

    private final Timer timer = new Timer();
    private double lastVelocity;

    public Elevator() {
        motor = new CANSparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ElevatorConstants.IDLE_MODE);
        motor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        motor.burnFlash();

        encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(ElevatorConstants.ENCODER_POSITION_FACTOR);
        encoder.setVelocityConversionFactor(ElevatorConstants.ENCODER_VELOCITY_FACTOR);
        encoder.setInverted(ElevatorConstants.ENCODER_INVERTED);
        encoder.setPosition(0);

        feedbackController = motor.getPIDController();
        feedbackController.setFeedbackDevice(encoder);
        feedbackController.setP(ElevatorConstants.P);
        feedbackController.setI(ElevatorConstants.I);
        feedbackController.setD(ElevatorConstants.D);
        feedbackController.setFF(0);
        feedbackController.setOutputRange(ElevatorConstants.PID_MINIMUM_OUTPUT, ElevatorConstants.PID_MAXIMUM_OUTPUT);

        feedforwardController = new ElevatorFeedforward(
            ElevatorConstants.S,
            ElevatorConstants.G,
            ElevatorConstants.V,
            ElevatorConstants.A);

        constraints = new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
        setpoint = new TrapezoidProfile.State(0, 0);
        goal = new TrapezoidProfile.State(0, 0);
        lastVelocity = 0;
    }

    @Override
    public void periodic() {
        double deltaTime = timer.get();
        timer.reset();

        TrapezoidProfile profile = new TrapezoidProfile(constraints);
        setpoint = profile.calculate(deltaTime, setpoint, goal);
        
        double feedforward = feedforwardController.calculate(setpoint.velocity, (setpoint.velocity - lastVelocity) / deltaTime);
        
        feedbackController.setReference(
            setpoint.position,
            CANSparkBase.ControlType.kPosition,
            0,
            feedforward);

        lastVelocity = setpoint.velocity;
    }

    public TrapezoidProfile.State getGoal() {
        return this.goal;
    }

    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    public void setHeight(double height) {
        this.goal = new State(height, 0);
    }
}
