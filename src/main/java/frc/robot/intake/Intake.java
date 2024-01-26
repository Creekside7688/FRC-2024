package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class Intake extends SubsystemBase {
    //ids have not been set yet
    private final CANSparkMax motor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput sensor = new DigitalInput(IntakeConstants.INTAKE_SENSOR_CHANNEL);
    private final SparkPIDController rotationController = motor.getPIDController();
    private final RelativeEncoder encoder = motor.getEncoder(); 
    

    public Intake() {
        rotationController.setP(0.1);
    }

    @Override
    public void periodic() {
    }

    public boolean getSensor() {
        return sensor.get();

    }

    public double getRPM() {
        double velocity = encoder.getVelocity();
        return velocity / IntakeConstants.INTAKE_GEAR_RATIO;
    }

    public void setRPM(double rpm) {
        rotationController.setReference(rpm, ControlType.kVelocity);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
