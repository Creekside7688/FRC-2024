package frc.robot.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.swerve.SwerveDrive;

public class FlipRotation extends Command {
    private final SwerveDrive swerveDrive;
    private final PIDController rotationPIDController;
    private double targetAngle;

    public FlipRotation(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        rotationPIDController = new PIDController(DriveConstants.FLIP_P, DriveConstants.FLIP_I, DriveConstants.FLIP_D);
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
        output = MathUtil.clamp(output, -0.2, 0.2);

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
