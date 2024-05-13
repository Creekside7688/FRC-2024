package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;

import frc.robot.intake.Intake;

public class IntakeShooterFeed extends Command {
    private final Intake intake;

    public IntakeShooterFeed(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.run(IntakeConstants.SHOOTER_FEED_SPEED);
    }

    @Override
    public void execute() {
        this.intake.updateDashboard();
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
    }

    @Override
    public boolean isFinished() {
        //return Timer.getFPGATimestamp() - startTime > IntakeConstants.SHOOTER_FEED_DURATION;
        return false;
    }
}
