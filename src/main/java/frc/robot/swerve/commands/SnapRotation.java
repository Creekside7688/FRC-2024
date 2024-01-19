package frc.robot.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class SnapRotation extends Command {
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
