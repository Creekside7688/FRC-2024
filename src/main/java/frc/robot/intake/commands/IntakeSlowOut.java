// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake;

public class IntakeSlowOut extends Command {
    private final Intake intake;
    

    public IntakeSlowOut(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {

        intake.useIntake(-0.25);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        Timer.delay(1);
        return true;


    }
}