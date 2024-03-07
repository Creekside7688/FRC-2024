package frc.robot.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.constants.ElevatorConstants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;

    public Elevator() {
        motor = new CANSparkMax(ElevatorConstants.MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(ElevatorConstants.IDLE_MODE);
        motor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
        

        encoder = motor.getEncoder();
        encoder.setPosition(0);
        encoder.setPositionConversionFactor(1);
        motor.burnFlash();
    }

    public void getMotorRPM() {
        SmartDashboard.putNumber("Evt. Motor RPM", encoder.getVelocity());
    }

    @Override

    public void periodic() {
        SmartDashboard.putNumber("Elevator Height (Inches)", this.getEncoderPosition() );
    }

    public void run(double speed) {
        motor.set(speed);
    }

    public double getEncoderPosition() {
        return encoder.getPosition()/210;
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }
}
