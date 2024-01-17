package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import frc.lib.zylve.Controller;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.commands.FlipRotation;
import frc.robot.swerve.commands.SnapRotation;

public class RobotContainer {
    private final SwerveDrive swerveDrive = new SwerveDrive();

    private final SnapRotation snap90 = new SnapRotation(90, swerveDrive);
    private final FlipRotation flip180 = new FlipRotation(swerveDrive);

    Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    public RobotContainer() {
        configureButtonBindings();
        swerveDrive.setDefaultCommand(
                new RunCommand(
                        () -> swerveDrive.joystickDrive(
                                -MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DEADBAND),
                                -MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.DEADBAND),
                                true, true),
                        swerveDrive));

        controller.getLeftTrigger().onTrue(snap90);
        controller.getRightTrigger().whileTrue(flip180);
    }

    private void configureButtonBindings() {
        controller.getLeftStick()
                .whileTrue(new RunCommand(
                        () -> swerveDrive.lockPosition(),
                        swerveDrive));

        controller.getRightStick()
                .whileTrue(new RunCommand(
                        () -> swerveDrive.zeroHeading(),
                        swerveDrive));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("New Auto");
    }
}
