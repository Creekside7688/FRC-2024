package frc.robot.intake.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.intake.Intake;

public class IntakeAmpScore extends Command {
    private final Intake intake;

    public IntakeAmpScore(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        intake.setSpeed(IntakeConstants.AMPSCORE_SPEED);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(IntakeConstants.INTAKE_MOTORS_OFF);
    }

    @Override
    public boolean isFinished() {
        Timer.delay(IntakeConstants.APMSCORE_DELAY);
        return true;
    }
}