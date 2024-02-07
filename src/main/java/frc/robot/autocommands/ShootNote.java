// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.intake.Intake;

import frc.robot.intake.commands.IntakeShooterFeed;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.ActivateShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends ParallelCommandGroup {
  /** Creates a new ShootNoteFeed. */
  public ShootNote(Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ActivateShooter(shooter),

        new IntakeShooterFeed(intake)

    );
  }
}
