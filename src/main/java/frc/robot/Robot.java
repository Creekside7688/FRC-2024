package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    private Alliance alliance = Alliance.Red;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        checkDriverStationUpdate();
    }

    @Override
    public void robotPeriodic() {
        checkDriverStationUpdate();
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Time Remaining", DriverStation.getMatchTime());

        SmartDashboard.putData(CommandScheduler.getInstance());
        // Replace with values from PDH
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Battery Amperage", 60);

        SmartDashboard.putNumber("Team", 7688);
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        checkDriverStationUpdate();
        autonomousCommand = robotContainer.getAutonomousCommand();

        if(autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        checkDriverStationUpdate();
        if(autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    private void checkDriverStationUpdate() {
        if(!DriverStation.getAlliance().isPresent()) {
            return;
        }

        Alliance currentAlliance = DriverStation.getAlliance().get();

        // If we have data, and have a new alliance from last time
        if(DriverStation.isDSAttached() && currentAlliance != alliance) {
            robotContainer.onAllianceChanged(currentAlliance);
            alliance = currentAlliance;
        }
    }
}
