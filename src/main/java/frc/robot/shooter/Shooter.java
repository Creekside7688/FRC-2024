package frc.robot.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(ShooterConstants.MOTOR_ID, MotorType.kBrushless);

    public Shooter() {
        motor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
        motor.setIdleMode(ShooterConstants.IDLE_MODE);
    }

    @Override
    public void periodic() {
    }

    public void run(double speed) {
        motor.set(speed);
    }
}
