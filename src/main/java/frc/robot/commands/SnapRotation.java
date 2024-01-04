// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class SnapRotation extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final PIDController rotationPIDController;
    private final int angleSnap;
    private double targetAngle;

    public SnapRotation(int angleSnap, SwerveDrive swerveDrive) {
        this.angleSnap = angleSnap;
        this.swerveDrive = swerveDrive;

        rotationPIDController = new PIDController(DriveConstants.SNAP_P, DriveConstants.SNAP_I, DriveConstants.SNAP_D);
        rotationPIDController.setTolerance(2);
    }

    @Override
    public void initialize() {
        this.targetAngle = this.getNearestAngle();
    }

    @Override
    public void execute() {
        rotationPIDController.setSetpoint(this.targetAngle);

        double output = MathUtil.clamp(rotationPIDController.calculate(swerveDrive.getRotation2d().getDegrees()), -1, 1);

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

    private double getNearestAngle() {
        double currentRotation = swerveDrive.getRotation2d().getDegrees();

        double y = currentRotation / angleSnap;

        y = Math.floor(y);

        double lowerLimit = angleSnap * y;
        double upperLimit = angleSnap * (y + 1);

        double nearestAngle;

        if(Math.abs(currentRotation - upperLimit) > Math.abs(currentRotation - lowerLimit)) {
            nearestAngle = lowerLimit;
        } else {
            nearestAngle = upperLimit;
        }

        return nearestAngle;
    }

}
