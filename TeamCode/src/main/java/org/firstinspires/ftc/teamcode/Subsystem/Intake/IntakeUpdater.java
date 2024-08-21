package org.firstinspires.ftc.teamcode.Subsystem.Intake;

import org.firstinspires.ftc.teamcode.Utils.SubSytemUpdater;
import org.firstinspires.ftc.teamcode.Utils.Update;
import org.firstinspires.ftc.teamcode.Utils.UpdateQueue;

import java.util.PriorityQueue;

public class IntakeUpdater implements SubSytemUpdater {
    public UpdateQueue queue = new UpdateQueue();
    public boolean ENABLED = true;

    public boolean UPDATEABLE = false;

    @Override
    public void update() {

        if(queue.getSize() > 0 && ENABLED) {
            UPDATEABLE = true;
            queue.run();
        }else {
            UPDATEABLE = false;
        }
    }

    public void add(Update update) {

        queue.addUpdate(update);
    }
    @Override
    public void stop() {
        ENABLED = false;
    }

    @Override
    public void enable() {
        ENABLED = true;
    }
}
