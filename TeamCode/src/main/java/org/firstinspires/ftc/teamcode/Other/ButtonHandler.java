package org.firstinspires.ftc.teamcode.Other;

public class ButtonHandler {
    private boolean lastStateA_1 = false;
    private boolean lastStateB_1 = false;
    private boolean lastStateY_1 = false;
    private boolean lastStateX_1 = false;
    private boolean lastStateLB_1 = false;
    private boolean lastStateRB_1 = false;


    public boolean isPressedOnceA_1(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateA_1;
        lastStateA_1 = currentState;
        return wasPressedOnce;
    }

    public boolean isPressedOnceB_1(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateB_1;
        lastStateB_1 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceY_1(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateY_1;
        lastStateY_1 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceX_1(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateX_1;
        lastStateX_1 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceLB_1(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateLB_1;
        lastStateLB_1 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceRB_1(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateRB_1;
        lastStateRB_1 = currentState;
        return wasPressedOnce;
    }

    // 2
    private boolean lastStateA_2 = false;
    private boolean lastStateB_2 = false;
    private boolean lastStateY_2 = false;
    private boolean lastStateX_2 = false;
    private boolean lastStateLB_2 = false;
    private boolean lastStateRB_2 = false;


    public boolean isPressedOnceA_2(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateA_2;
        lastStateA_2 = currentState;
        return wasPressedOnce;
    }

    public boolean isPressedOnceB_2(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateB_2;
        lastStateB_2 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceY_2(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateY_2;
        lastStateY_2 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceX_2(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateX_2;
        lastStateX_2 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceLB_2(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateLB_2;
        lastStateLB_2 = currentState;
        return wasPressedOnce;
    }
    public boolean isPressedOnceRB_2(boolean currentState) {
        boolean wasPressedOnce = currentState && !lastStateRB_2;
        lastStateRB_2 = currentState;
        return wasPressedOnce;
    }
}
