package org.firstinspires.ftc.teamcode.Utils;

public class Subsystem {
    public boolean UPDATABLE = false;
    public void update(){};

    public void stop(){};
    public void enable(){};

    public void yesUpdate() {
        UPDATABLE =true;
    }

    public void noUpdate() {
        UPDATABLE = false;
    }
}
