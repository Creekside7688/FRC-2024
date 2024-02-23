// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator.commands;

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
        elevator.setHeight(ElevatorConstants.TEMP_MAX_HEIGHT);
    }


    @Override
    public void execute() {}


    @Override
    public void end(boolean interrupted) {
        elevator.setHeight(0);
    }


    @Override
    public boolean isFinished() {
        return false;
    }
}
