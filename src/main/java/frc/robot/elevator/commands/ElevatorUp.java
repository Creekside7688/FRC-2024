package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorUp extends Command {
    private final Elevator elevator;

    public ElevatorUp(Elevator elevator) {
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
        double maxSteps = elevator.getEncoderPosition();
        SmartDashboard.putNumber("EncoderSteps", maxSteps);

    }

    @Override
    public void end(boolean interrupted) {
        elevator.run(ElevatorConstants.MOTOR_SLOWRAISE_STALLSPEED);
        SmartDashboard.putBoolean("Ele. Up finished", true);
    }

    @Override
    public boolean isFinished() {
        return elevator.getEncoderPosition() > ElevatorConstants.MOTOR_MAX_STEPS;
    }
}
