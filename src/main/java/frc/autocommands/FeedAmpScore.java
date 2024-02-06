// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorDown;
import frc.robot.elevator.commands.ElevatorUp;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeAmpScore;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FeedAmpScore extends SequentialCommandGroup {
  /** Creates a new FeedAmpScore. */
    public FeedAmpScore(Elevator elevator, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
        addCommands(

            new ElevatorUp(elevator),

            new IntakeAmpScore(intake),

            new ElevatorDown(elevator)



       );
  }
}
