package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import frc.lib.zylve.Controller;
import frc.robot.constants.OperatorConstants;
import frc.robot.elevator.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private final SwerveDrive swerveDrive = new SwerveDrive();
    private final Elevator elevator = new Elevator();

    private final Controller controller = new Controller(OperatorConstants.CONTROLLER_PORT);

    public RobotContainer() {

        configureButtonBindings();

        swerveDrive.setDefaultCommand(
            new RunCommand(
                () -> swerveDrive.drive(
                    -MathUtil.applyDeadband(controller.getLeftY(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getLeftX(), OperatorConstants.DEADBAND),
                    -MathUtil.applyDeadband(controller.getRightX(), OperatorConstants.DEADBAND),
                    true, true),
                swerveDrive));
    }

    private void configureButtonBindings() {
        controller.getA().whileTrue(elevator.getSysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.getB().whileTrue(elevator.getSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controller.getX().whileTrue(elevator.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        controller.getY().whileTrue(elevator.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("New Auto");
    }
}
