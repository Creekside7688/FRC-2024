package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;


public class IntakePickup extends Command {
    private final Intake intake;

    public IntakePickup(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(1);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        boolean sensorBool = intake.getSensor();

        return sensorBool;
    }
}

