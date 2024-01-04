// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Snap extends CommandBase {
    /** Creates a new snap90. */
    private final SwerveDrive swerveDrive;
    private final PIDController PIDController;
    private final int angleSnap;

    public Snap(int angleSnap, SwerveDrive swerveDrive) {
        this.angleSnap = angleSnap;
        this.swerveDrive = swerveDrive;

        PIDController = new PIDController(DriveConstants.SNAP_P, DriveConstants.SNAP_I, DriveConstants.SNAP_D);
        PIDController.setTolerance(2);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // pid.setSetpoint(90);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double targetAngle = getNearestAngle();
        PIDController.setSetpoint(targetAngle);
        double output = MathUtil.clamp(PIDController.calculate(swerveDrive.getRotation2d().getDegrees()), -1, 1);
        swerveDrive.setPIDInput(output);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveDrive.setPIDInput(null);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return PIDController.atSetpoint();
    }

    private double getNearestAngle() {
        double currentPos = swerveDrive.getRotation2d().getDegrees();

        double y = currentPos / angleSnap;

        y = Math.floor(y);

        double lowerLimit = angleSnap * y;
        double upperLimit = angleSnap * (y + 1);

        double nearestAngle;

        if(Math.abs(currentPos - upperLimit) > Math.abs(currentPos - lowerLimit)) {
            nearestAngle = lowerLimit;
        } else {
            nearestAngle = upperLimit;
        }

        return nearestAngle;
    }

}
