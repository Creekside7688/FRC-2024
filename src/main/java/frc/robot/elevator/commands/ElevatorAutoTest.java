package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorAutoTest extends Command {
    private final Elevator elevator;

    public ElevatorAutoTest(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.run(ElevatorConstants.MOTOR_SLOWRAISE_SPEED);
        SmartDashboard.putBoolean("Ele. Up finished", false);
    }

    @Override
    public void execute() {
        double maxSteps = elevator.encoderPosition();
        SmartDashboard.putNumber("EncoderSteps", maxSteps);

    }

    @Override
    public void end(boolean interrupted) {
        elevator.run(ElevatorConstants.MOTOR_SLOWRAISE_STALLSPEED);
        SmartDashboard.putBoolean("Ele. Up finished", true);
    }

    @Override
    public boolean isFinished() {
        return elevator.encoderPosition() > ElevatorConstants.MOTOR_MAX_STEPS;
    }
}
