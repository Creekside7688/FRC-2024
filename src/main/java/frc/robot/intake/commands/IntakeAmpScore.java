package frc.robot.intake.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;

public class IntakeAmpScore extends Command {
    private final Intake intake;

    public IntakeAmpScore(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        intake.setSpeed(-0.25);
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
        Timer.delay(1);
        return true;
    }
}