// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.elevator.Elevator;

public class ElevatorTempUp extends Command {
    private final Elevator elevator;

    public ElevatorTempUp(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    
    }


     @Override
    public void initialize() {
        elevator.elevatorMotorSpeed(ElevatorConstants.MOTOR_SLOWRAISE_SPEED);
        SmartDashboard.putBoolean("Ele. Temp finished", false);
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
        elevator.elevatorMotorSpeed(ElevatorConstants.MOTOR_TEMPDROP_SPEED);
        SmartDashboard.putBoolean("Ele. Temp finished", true);
        Timer.delay(ElevatorConstants.MOTOR_TEMPDROP_DELAY);
        elevator.elevatorMotorSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double maxSteps = elevator.encoderGearPos();
        if(maxSteps >   ElevatorConstants.MOTOR_TEMP_STEPS) {
            
            return true;    
        } else {
            return false;
        }

    }
}
