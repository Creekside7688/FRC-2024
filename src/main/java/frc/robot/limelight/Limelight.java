package frc.robot.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final NetworkTable table;

    public Limelight() {
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
    }

    public double getXLimelight() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0);
    }

    public double getValidTarget() {
        NetworkTableEntry tv = table.getEntry("tv");
        return tv.getDouble(0);
    }

    public double getTargetArea() {
        NetworkTableEntry ta = table.getEntry("ta");
        return ta.getDouble(0);
    }
}

