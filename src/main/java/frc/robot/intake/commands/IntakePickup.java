package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        //double speed = Preferences.getDouble(intake.pickUpSpeedKey, intake.pickUpSpeedDefault);
        intake.run(0.8);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("sensorSub", intake.hasNote());
        Intake.getMotorAU();
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
        Timer.delay(0.5);
        intake.run(-0.3);
        Timer.delay(0.2);
        intake.run(0);
    }

    @Override
    public boolean isFinished() {
        return intake.hasNote();
    }
}
