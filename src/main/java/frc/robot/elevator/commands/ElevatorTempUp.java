package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorTempUp extends Command {
    private final Elevator elevator;

    public ElevatorTempUp(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.run(ElevatorConstants.MOTOR_SLOWRAISE_SPEED);
        SmartDashboard.putBoolean("Ele. Temp finished", false);
    }

    @Override
    public void execute() {
        double maxSteps = elevator.encoderPosition();
        SmartDashboard.putNumber("EncoderSteps", maxSteps);

    }

    @Override
    public void end(boolean interrupted) {
        elevator.run(ElevatorConstants.MOTOR_TEMPDROP_SPEED);
        SmartDashboard.putBoolean("Ele. Temp finished", true);
        Timer.delay(ElevatorConstants.MOTOR_TEMPDROP_DELAY);
        elevator.run(0);
    }

    @Override
    public boolean isFinished() {
        return elevator.encoderPosition() > ElevatorConstants.MOTOR_TEMP_STEPS;
    }
}
