package com.team254.frc2023.controlboard;

public interface IDriveControlBoard {
    double getThrottle();

    double getStrafe();

    double getRotation();

    boolean resetGyro();

    boolean getSwitchDriveMode();

    boolean getScore();

    boolean getAutoAlign();
    
}