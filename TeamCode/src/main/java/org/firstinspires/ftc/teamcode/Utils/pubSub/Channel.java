package org.firstinspires.ftc.teamcode.Utils.pubSub;

import java.io.IOException;

public interface Channel {
    public void receive(String TAG,Update update,String description) throws IOException;
    public void stop();
    public void enable();
    public void update();
}
