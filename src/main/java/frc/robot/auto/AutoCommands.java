// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.AmpAlign;
import frc.robot.auto.commands.SpeakerAlign;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.commands.ElevatorDown;
import frc.robot.elevator.commands.ElevatorUp;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeAmpScore;
import frc.robot.intake.commands.IntakeShooterFeed;
import frc.robot.shooter.Shooter;
import frc.robot.shooter.commands.ShooterSpinDown;
import frc.robot.shooter.commands.ShooterSpinUp;
import frc.robot.swerve.SwerveDrive;

/** Add your docs here. */
public class AutoCommands {

    public SequentialCommandGroup ampSuperAlign(SwerveDrive swerveDrive, Elevator elevator) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new AmpAlign(swerveDrive, swerveDrive::getPose),
                new ElevatorUp(elevator)
            )
        );
    }
    public SequentialCommandGroup ampSuperShoot(Elevator elevator, Intake intake) {
        return new SequentialCommandGroup(
            new IntakeAmpScore(intake),
            new ElevatorDown(elevator).withTimeout(0.25)
    );
    }

    public SequentialCommandGroup shooterSuperAlign(SwerveDrive swerveDrive, Shooter shooter) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SpeakerAlign(swerveDrive, swerveDrive::getPose),
                new ShooterSpinUp(shooter)
            )
        );
    }

    public SequentialCommandGroup shooterSuperShoot(Shooter shooter, Intake intake) {
        return new SequentialCommandGroup(
            new IntakeShooterFeed(intake),
            new ShooterSpinDown(shooter)
    );
    }
    

    /*public SequentialCommandGroup ampSuperCommand(SwerveDrive swerveDrive, Elevator elevator, Intake intake) {
        return new SequentialCommandGroup(
    
                new ampSuperAlign(swerveDrive, elevator),
                new ampSuperShoot(elevator, intake)
            
        );
    }
    public SequentialCommandGroup shooterSuperCommand(SwerveDrive swerveDrive, Elevator elevator, Intake intake, Shooter shooter) {
        return new SequentialCommandGroup(
    
                new shooterSuperAlign(swerveDrive, shooter),
                new shooterSuperShoot(elevator, intake)
            
        );
    } */
}

