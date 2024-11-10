// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.limelight.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Limelight;
import frc.robot.swerve.SwerveDrive;

public class AprilTagCheck extends Command {
  private final Limelight limelight;
  private final SwerveDrive swerveDrive;
  public AprilTagCheck(Limelight limelight, SwerveDrive swerveDrive) {
    this.limelight = limelight;
    this.swerveDrive = swerveDrive;
    addRequirements( limelight, swerveDrive);



  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.CameraHasTargets()) {
      limelight.TargetPitch();
      limelight.TargetYaw();
      limelight.getAprilTag();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
