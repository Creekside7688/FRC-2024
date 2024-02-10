package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.intake.Intake;

import frc.robot.intake.commands.IntakeShooterFeed;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.ActivateShooter;

public class ShootNote extends ParallelCommandGroup {
    public ShootNote(Shooter shooter, Intake intake) {
        addCommands(
            new ActivateShooter(shooter),

            new IntakeShooterFeed(intake)

        );
    }
}
