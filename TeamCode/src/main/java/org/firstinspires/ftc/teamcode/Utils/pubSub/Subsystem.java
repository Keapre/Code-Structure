package org.firstinspires.ftc.teamcode.Utils.pubSub;

import java.util.List;

public class Subsystem {
    public boolean UPDATABLE = false;
    public void update(){
        noUpdate();
    };

    public void stop(){};
    public void enable(){};

    public void yesUpdate() {
        UPDATABLE =true;
    }

    public void noUpdate() {
        UPDATABLE = false;
    }


}
