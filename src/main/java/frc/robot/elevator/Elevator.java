package frc.robot.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public Elevator() {
        motor = new CANSparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ElevatorConstants.IDLE_MODE);
        motor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        motor.burnFlash();

        encoder = motor.getEncoder();
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Height (Inches)", this.getEncoderPosition() * 210);
    }

    public void run(double speed) {
        motor.set(speed);
    }

    public double getEncoderPosition() {
        return encoder.getPosition() / 210;
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }
}
