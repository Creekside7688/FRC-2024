package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.elevator.Elevator;
import frc.robot.constants.ElevatorConstants;

public class ElevatorUp extends Command {

    private final Elevator elevator;

    public ElevatorUp(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setHeight(ElevatorConstants.TARGET_HEIGHT);
        SmartDashboard.putBoolean("upcmd", true);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("upcmd", false);
    }

    @Override
    public boolean isFinished() {
        return elevator.atGoal();
    }
}