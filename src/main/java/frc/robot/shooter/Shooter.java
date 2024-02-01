package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase { //similar to making a scratch sprite
    private final CANSparkMax shooterMotor;

    public Shooter() {
        shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
    }

    public void shooterMotorOn() {
        shooterMotor.set(ShooterConstants.SHOOTER_ON);
    }
    
    public void shooterMotorOff() {
        shooterMotor.set(ShooterConstants.SHOOTER_OFF);
    }

    public void inputShooterMotor(double inputShooterMotorValue) {
        shooterMotor.set(inputShooterMotorValue);
    }
}
