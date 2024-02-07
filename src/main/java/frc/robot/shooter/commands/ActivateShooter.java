package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.shooter.Shooter;

public class ActivateShooter extends Command {
    private final Shooter shooter;
    private double startTime;

    public ActivateShooter(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    } 

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        shooter.shooterRun();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        shooter.shooterOff();
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > ShooterConstants.DELAY;
    }
}