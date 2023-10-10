package com.team254.lib.util;

import org.littletonrobotics.junction.Logger;
import com.team254.frc2023.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import edu.wpi.first.networktables.*;

public class AdvantageKit {
    private static Logger mLogger = Logger.getInstance();

    public static void startLog() {
        if (Robot.isReal()) {
            File[] mediaDevices = new File("/media/").listFiles();
            String logPath = "/home/lvuser/";
            for (File mediaDevice : mediaDevices) {
                if (mediaDevice.listFiles().length > 0) {
                    logPath = mediaDevice.getAbsolutePath().concat("/Latest.wpilog");
                    break;
                }
            }
            mLogger.addDataReceiver(new NT4Publisher());
            mLogger.addDataReceiver(new WPILOGWriter(logPath));
            SmartDashboard.putString("Log Path", logPath);
        }
        mLogger.start();
    }

    public static void endLog() {
        mLogger.end();
    }

    public static void logAdvantageKit(NetworkTableEntry entry, String trackedTable, String key) {
        switch (entry.getType()) {
            case kDouble:
                mLogger.recordOutput("Values/" + trackedTable + "/" + key, entry.getDouble(0.0));
                break;
            case kDoubleArray:
                mLogger.recordOutput("Values/" + trackedTable + "/" + key.toString(),
                        entry.getDoubleArray(new double[] { 0.0, 0.0 }));
                break;
            case kString:
                mLogger.recordOutput("Values/" + trackedTable + "/" + key.toString(), entry.getString(""));
                break;
            case kBoolean:
                mLogger.recordOutput("Values/" + trackedTable + "/" + key.toString(), entry.getBoolean(false));
                break;
            default:
                mLogger.recordOutput("Values/" + trackedTable + "/" + key.toString(),
                        "(AdvantageKit) Not tracking entry type " + entry.getType().toString());
                break;
        }
    }

    public static void recordMetadata() {
        mLogger.recordMetadata("ProjectName", "FRC-SWERVE-BASE");
    }
}
