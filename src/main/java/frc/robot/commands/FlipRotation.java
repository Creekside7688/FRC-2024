// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;

public class FlipRotation extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final PIDController rotationPIDController;
    private double targetAngle;

    public FlipRotation(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        rotationPIDController = new PIDController(DriveConstants.SNAP_P, DriveConstants.SNAP_I, DriveConstants.SNAP_D);
        rotationPIDController.setTolerance(2);
    }


    @Override
    public void initialize() {
        targetAngle = this.getTargetAngle();
    }


    @Override
    public void execute() {
        rotationPIDController.setSetpoint(this.targetAngle);

        double output = rotationPIDController.calculate(swerveDrive.getRotation2d().getDegrees());

        swerveDrive.setPIDInput(output);
    }


    @Override
    public void end(boolean interrupted) {
        swerveDrive.setPIDInput(null);
    }


    @Override
    public boolean isFinished() {
        return rotationPIDController.atSetpoint();
    }

    private double getTargetAngle() {
        double currentRotation = swerveDrive.getRotation2d().getDegrees();

        double y = currentRotation / 90;

        y = Math.floor(y);

        double lowerLimit = 90 * y;
        double upperLimit = 90 * (y + 1);

        double targetAngle;

        if(Math.abs(currentRotation - upperLimit) > Math.abs(currentRotation - lowerLimit)) {
            targetAngle = lowerLimit;
        } else {
            targetAngle = upperLimit;
        }

        return targetAngle - 180;
    }
}
