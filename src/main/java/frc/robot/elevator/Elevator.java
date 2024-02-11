package frc.robot.elevator;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Distance> distance = mutable(Inches.of(0));
    private final MutableMeasure<Velocity<Distance>> velocity = mutable(InchesPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

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

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(2),
                Seconds.of(10)
            ),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> motorVoltage) -> {
                    motor.setVoltage(motorVoltage.in(Units.Volts));
                },

                log -> {
                    log.motor("Elevator Motor")
                        .voltage(
                            appliedVoltage.mut_replace(
                                motor.get() * RobotController.getBatteryVoltage(), Volts))
                        .linearPosition(distance.mut_replace(encoder.getPosition(), Inches))
                        .linearVelocity(
                            velocity.mut_replace(encoder.getVelocity(), InchesPerSecond));
                },

                this));
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

    /**
     * Gets the current <STRONG>goal</STRONG> of the elevator.
     * 
     * @return The current goal of the elevator in metres and metres per second.
     */
    public TrapezoidProfile.State getGoal() {
        return this.goal;
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

    public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
