package frc.robot.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
    private final Timer timer = new Timer();

    private final Constraints constraints = new Constraints(1.0, 1.0);
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0.0, 0.0);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State(0.0, 0.0);

    private double lastVelocity = 0.0;

    private final PIDController feedbackController = new PIDController(0.0, 0.0, 0.0);
    private final ElevatorFeedforward feedforwardController = new ElevatorFeedforward(1, 0.1, 0.1, 0.1);

    public Elevator() {
    }

    @Override
    public void periodic() {
        double deltaTime = timer.get();
        timer.reset();

        TrapezoidProfile profile = new TrapezoidProfile(constraints);
        setpoint = profile.calculate(deltaTime, setpoint, goal);

        double feedback = feedbackController.calculate(0, 0);
        double feedforward = feedforwardController.calculate(setpoint.velocity, (setpoint.velocity - lastVelocity) / deltaTime);

        lastVelocity = setpoint.velocity;

        motor.setVoltage(feedback + feedforward);
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
