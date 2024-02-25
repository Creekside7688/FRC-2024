package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorSlowRaise extends Command {
    private final Elevator elevator;

    public ElevatorSlowRaise(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.run(ElevatorConstants.MOTOR_SLOWRAISE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.run(ElevatorConstants.MOTOR_SLOWRAISE_STALLSPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
