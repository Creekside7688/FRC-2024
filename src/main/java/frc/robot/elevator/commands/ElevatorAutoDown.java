package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorAutoDown extends Command {
    private final Elevator elevator;

    public ElevatorAutoDown(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.run(ElevatorConstants.MOTOR_SLOWFALL_SPEED);
        SmartDashboard.putBoolean("Ele. Down finished", false);
    }

    @Override
    public void execute() {
        double maxSteps = elevator.encoderPosition();
        SmartDashboard.putNumber("EncoderSteps", maxSteps);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.run(0);
        SmartDashboard.putBoolean("Ele. Down finished", true);

    }

    @Override
    public boolean isFinished() {
        return elevator.encoderPosition() < ElevatorConstants.MOTOR_MIN_STEPS;
    }
}
