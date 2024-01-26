package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;


public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.useIntake(1);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        boolean sensorBool = intake.getIntakeSensor();

        if (sensorBool == true) {
            return true;
        } else {
            return false;
        }
    }
}

