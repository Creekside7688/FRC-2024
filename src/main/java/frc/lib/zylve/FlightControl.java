package frc.lib.zylve;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Joystick wrapper because ~~it was pissing me off~~ we need a cleaner RobotContainer.
 * <p>
 * The methods in this class return triggers. To get boolean values, use {@link Trigger#getAsBoolean()}
 * </p>
 */
public class FlightControl {
    private final Joystick joystick;
    private final Trigger button1;

    public  FlightControl(int port) {
        joystick = new Joystick(port);

        button1 = new JoystickButton(joystick, Joystick.ButtonType.kTrigger.value);

    }
    

    public Joystick getJoystick() {
        return joystick;
    }
    
    public double getJoyX() {
        return joystick.getRawAxis(Joystick.AxisType.kX.value);
    }

    public double getJoyY() {
        return joystick.getRawAxis(Joystick.AxisType.kY.value);
    }

    public double getTwist() {
        return joystick.getRawAxis(Joystick.AxisType.kTwist.value);
    }

    public Trigger getButton1() {
        return button1;
    }


   
   
}