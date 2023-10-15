package com.team254.frc2023.controlboard;


public interface IButtonControlBoard {
    boolean getScoreHigh();

    boolean getScoreMedium();

    boolean getScoreLow();

    boolean primeLowFling();

    boolean primeHighFling();

    boolean getScoreMid();

    boolean getPickupFeederCone();

    boolean getPickupGroundCone();

    boolean getPickupFeederCube();

    boolean getPickupGroundCube();

    boolean getOpenClaw();

    boolean getCloseClaw();

    boolean getStow();

    boolean getTestButton();

    boolean getToggleClimbMode();

    double getManualForksJog();

    boolean getForksUp();

    boolean getFling();

    boolean getForksDown();
    
    boolean getGroundIntakePush();
}
