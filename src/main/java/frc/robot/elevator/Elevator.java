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
import frc.robot.constants.IntakeConstants;

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

    /**
     * Creates an elevator subsystem. To change settings, use the ElevatorConstants class.
     */
    public Elevator() {
        motor = new CANSparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ElevatorConstants.IDLE_MODE);
        motor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        motor.burnFlash();

        encoder = motor.getEncoder();
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
            ElevatorConstants.A
        );

        constraints = new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION);
        setpoint = new TrapezoidProfile.State(0, 0);
        goal = new TrapezoidProfile.State(0, 0);
        lastVelocity = 0;
    }


    /**
     * Gets the current <STRONG>goal</STRONG> of the elevator.
     * 
     * @return The current goal of the elevator in metres and metres per second.
     */
    public TrapezoidProfile.State getGoal() {
        return this.goal;
    }

    public void elevatorMotorSpeed(double motorspeed) {
        motor.set(motorspeed);
    }
    /**
     * Sets the <STRONG>goal</STRONG> of the elevator.
     * 
     * @param goal The goal position and velocity of the elevator in metres and metres per second.
     */
    
    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }

    /**
     * Sets the goal height of the elevator.
     * 
     * @param height The goal position of the elevator in metres.
     */
    public void setHeight(double height) {
        this.goal = new State(height, 0);
    }

    /**
     * Gets whether the elevator is at its goal within the specified tolerance in the constants file. Use this to check when to move on to the next command in a sequential command
     * group.
     * 
     * @return Whether the elevator is at its goal within the specified tolerance.
     */
    public boolean atGoal() {
        return Math.abs(goal.position - encoder.getPosition()) < ElevatorConstants.TOLERANCE;
    }

    /**
     * Gets whether the elevator is at its goal within the passed tolerance. Use this to check when to move on to the next command in a sequential command group.
     * 
     * @param tolerance The tolerance in metres.
     * @return Whether the elevator is at its goal within the specified tolerance.
     */
    public boolean atGoal(double tolerance) {
        return Math.abs(goal.position - encoder.getPosition()) < tolerance;
    }

    public double encoderGearPos() {
        return encoder.getPosition() / 210;
    }
    
}
