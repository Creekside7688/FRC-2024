package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;

public class ElevatorDown extends Command {
    
    private final Elevator elevatorSubsystem;

    public ElevatorDown(Elevator elevator) {
        elevatorSubsystem = elevator;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        //shoot command
        elevatorSubsystem.setHeight(0);
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