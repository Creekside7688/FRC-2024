package frc.robot.intake;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
    
    private final CANSparkMax IntakeMotor = new CANSparkMax(6969, MotorType.kBrushless);

    private final DigitalInput IntakeSensor = new DigitalInput(420);

    private final RelativeEncoder IntakeRE = IntakeMotor.getEncoder(); 

    private final SparkPIDController RPMController = IntakeMotor.getPIDController();
    

    public Intake() {
        RPMController.setP(0.1);

    }

    @Override
    public void periodic() {
    }

    public void useIntake(double x) {
        IntakeMotor.set(x);
    }


    public boolean getIntakeSensor() {
        return IntakeSensor.get();

    }

    public double getRPM() {
        double RPMVar = IntakeRE.getVelocity();
        return RPMVar / 4;
    }

    public void setRPM(double rpm) {
        RPMController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }
    
}
