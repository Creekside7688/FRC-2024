package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;
import frc.robot.constants.ElevatorConstants;

public class ElevatorUp extends Command {
    
    private final Elevator elevatorSubsystem;

    public ElevatorUp(Elevator elevator) {
        elevatorSubsystem = elevator;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setHeight(ElevatorConstants.TARGET_HEIGHT);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atGoal();
    }
}