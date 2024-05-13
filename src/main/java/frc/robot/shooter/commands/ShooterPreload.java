package frc.robot.shooter.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.Shooter;

public class ShooterPreload extends Command {
    private final Shooter shooter;
    private final SlewRateLimiter filter = new SlewRateLimiter(0.5);

    public ShooterPreload(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        //startTime = Timer.getFPGATimestamp();
        
    }

    @Override
    public void execute() {
        shooter.updateDashboard();
        shooter.run(filter.calculate(-1));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return (shooter.getVelocity() >= 2800);
        //return Timer.getFPGATimestamp() - startTime > ShooterConstants.SHOOTER_SPINUP_DELAY;
    }
}