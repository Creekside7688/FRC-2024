package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax motor;

    public Shooter() {
        motor = new CANSparkMax(ShooterConstants.SHOOTER_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
    }

    public void shooterRun() {
        motor.set(ShooterConstants.SHOOTER_ON);
    }
    
    public void shooterOff() {
        motor.set(ShooterConstants.SHOOTER_OFF);
    }

    public void inputShooterMotor(double inputShooterMotorValue) {
        motor.set(inputShooterMotorValue);
    }
}
