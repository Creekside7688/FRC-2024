package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.intake.Intake;

public class IntakePickup extends Command {
    private final Intake intake;

    public IntakePickup(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.run(IntakeConstants.PICKUP_SPEED);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("sensorSub", intake.hasNote());
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }
}
