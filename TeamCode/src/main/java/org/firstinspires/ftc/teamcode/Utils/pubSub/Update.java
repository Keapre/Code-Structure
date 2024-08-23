package org.firstinspires.ftc.teamcode.Utils.pubSub;

import java.util.List;

public class Update {
    Runnable action;
    double basePriority;
    List<Subsystem> subsystemList;

    //String description;

    public Update(List<Subsystem> actuated,Runnable action) {
        this.subsystemList = actuated;
        this.action = action;
    }

    public Runnable getAction() {
        return this.action;
    }

    public double getPriority() {
        return this.basePriority;
    }

    public List<Subsystem> getSubsystems() {
        return this.subsystemList;
    }
}
