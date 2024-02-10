package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorDown;
import frc.robot.elevator.commands.ElevatorUp;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeAmpScore;

public class AmpScore extends SequentialCommandGroup {
    public AmpScore(Elevator elevator, Intake intake) {
        addCommands(
            new ElevatorUp(elevator),
            new IntakeAmpScore(intake),
            new ElevatorDown(elevator)
        );
    }
}
