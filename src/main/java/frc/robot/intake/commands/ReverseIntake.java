// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;



public class ReverseIntake extends Command {
    /** Creates a new ReverseIntake. */
    private final Intake intake;

    public ReverseIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.useIntake(-0.5);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        Timer.delay(1);
        
        return true;
        
    

    }
    
}
