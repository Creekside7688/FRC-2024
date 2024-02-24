// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorAutoDown extends Command {
    private final Elevator elevator;

    public ElevatorAutoDown(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.elevatorMotorSpeed(ElevatorConstants.MOTOR_SLOWFALL_SPEED);
        SmartDashboard.putBoolean("Ele. Down finished", false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      double maxSteps = elevator.encoderGearPos();
        SmartDashboard.putNumber("EncoderSteps" ,maxSteps);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.elevatorMotorSpeed(0);
        SmartDashboard.putBoolean("Ele. Down finished", true);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double steps = elevator.encoderGearPos();
        if (steps < ElevatorConstants.MOTOR_MIN_STEPS) {
            return true;
        }   else {
            return false;
        }
    }
  
}
