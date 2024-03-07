package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.shooter.Shooter;

public class ShooterPreload extends Command {
    private final Shooter shooter;

    public ShooterPreload(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        //startTime = Timer.getFPGATimestamp();
        shooter.run(-1);
    }

    @Override
    public void execute() {
        shooter.updateDashboard();
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