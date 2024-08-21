package org.firstinspires.ftc.teamcode.Utils.pubSub;

public class Update {
    Runnable action;
    double basePriority;

    //String description;

    public Update(Runnable action,double priority) {
        this.action = action;
        this.basePriority = priority;
    }

    public Runnable getAction() {
        return this.action;
    }

    public double getPriority() {
        return this.basePriority;
    }

}
