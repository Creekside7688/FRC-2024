package frc.robot.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.shooter.Shooter;

public class ShooterCommand extends Command { //similar to making your own scratch block

    private final Shooter shooterSubsystem; //declares and makes the "shooterSubsystem" variable with the class (items) of "Shooter"

    public ShooterCommand(Shooter shooter) { //makes it so we can shoot multiple motors
        shooterSubsystem = shooter; //sets id for this certain command 
        addRequirements(shooterSubsystem); //makes it so that you cant have two commands using the same shooters
    } 

    @Override
    public void initialize() { // runs once
        shooterSubsystem.shooterMotorOn(); //turn motor on
    }

    @Override
    public void execute() { //runs forever until isFinished is true
    }

    @Override
    public void end(boolean interrupted) { //runs after isFinished is true
        Timer.delay(ShooterConstants.DELAY);
        shooterSubsystem.shooterMotorOff();
    }

    @Override
    public boolean isFinished() { //controls when execute runs
        return true; 
    }
}