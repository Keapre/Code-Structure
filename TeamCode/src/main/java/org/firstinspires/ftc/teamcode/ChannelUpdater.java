package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystem.Intake.IntakeUpdater;
import org.firstinspires.ftc.teamcode.Utils.Channel;
import org.firstinspires.ftc.teamcode.Utils.Update;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class ChannelUpdater implements Channel {

    public boolean ENABLED = true;
    IntakeUpdater intakeUpdater;
    FileWriter writer = null;
    public ChannelUpdater(Telemetry telemetry) {
        File file = new File("/sdcard/FIRST/log.txt");

        // Check if the file exists and delete it
        if (file.exists()) {
            boolean deleted = file.delete();
            if (deleted) {
                telemetry.addData("File Status", "File deleted successfully");
            } else {
                telemetry.addData("File Status", "Failed to delete file");
            }
        } else {
            telemetry.addData("File Status", "File does not exist");
        }

        try {
            // Create the file
            if (file.createNewFile()) {
                telemetry.addData("File Status", "File created successfully");
            } else {
                telemetry.addData("File Status", "File already exists");
            }

            // Write some initial content to the file (optional)
            writer = new FileWriter(file, true);
            writer.write("Log Start\n");
            writer.close();

        } catch (IOException e) {
            telemetry.addData("File Status", "An error occurred: " + e.getMessage());
        }
    }
    @Override
    public void receive(String TAG, Update update,String description){

        try {
            writer.write( System.currentTimeMillis() / 1000 + ": " +  TAG + " " +  description);
            writer.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        switch (TAG){
            case "INTAKE":
                intakeUpdater.add(update);
                break;
        }
    }

    @Override
    public void stop() {
        ENABLED = false;
    }

    @Override
    public void enable() {
        ENABLED = true;
    }

    @Override
    public void update() {

    }
}
