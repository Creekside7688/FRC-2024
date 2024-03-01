package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class IntakeShooterFeedPreload extends Command {
    private final Intake intake;
    private final Shooter shooter;
    private double startTime;

    public IntakeShooterFeedPreload(Intake intake,Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
        addRequirements(intake,shooter);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
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
        return Timer.getFPGATimestamp() - startTime > IntakeConstants.SHOOTER_FEED_DURATION;
    }
}
