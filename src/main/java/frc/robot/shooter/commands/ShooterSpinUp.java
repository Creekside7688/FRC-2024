package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.intake.Intake;
import frc.robot.shooter.Shooter;

public class ShooterSpinUp extends Command {
    private final Shooter shooter;
    private double startTime;

    public ShooterSpinUp(Shooter shooter) {
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
        shooter.run(0);
    }

    @Override
    public boolean isFinished() {
        return false;
        //return Timer.getFPGATimestamp() - startTime > ShooterConstants.SHOOTER_SPINUP_DELAY;
        
    }
}