package com.team254.frc2023.auto.actions;

import java.util.function.BooleanSupplier;

public class InterruptableAction implements Action {

    private final Action mAction;
    private final BooleanSupplier mShouldInterrupt;

    private boolean isDone = false;

    public InterruptableAction(Action action, BooleanSupplier shouldInterrupt) {
        mAction = action;
        mShouldInterrupt = shouldInterrupt;
    }
    @Override
    public void start() {
        mAction.start();
    }

    @Override
    public void update() {
        if (mShouldInterrupt.getAsBoolean()) {
            isDone = true;
        }
        mAction.update();
    }

    @Override
    public boolean isFinished() {
        return isDone || mAction.isFinished();
    }

    @Override
    public void done() {
        mAction.done();
    }
}