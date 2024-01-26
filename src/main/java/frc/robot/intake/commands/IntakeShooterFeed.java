package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.intake.Intake;

public class IntakeShooterFeed extends Command {
    private final Intake intake;

    public IntakeShooterFeed(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setSpeed(-0.5);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        intake.setSpeed(IntakeConstants.INTAKE_MOTORS_OFF);
    }

    @Override
    public boolean isFinished() {
        Timer.delay(IntakeConstants.SHOOTERFEED_DELAY);
        return true;
    }
}
