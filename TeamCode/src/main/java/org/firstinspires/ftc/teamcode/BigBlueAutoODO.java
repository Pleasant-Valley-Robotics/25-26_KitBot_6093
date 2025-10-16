package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Big Auto - kitbot", group="Robot")
public abstract class BigBlueAutoODO extends AutoTemplateODO {
    public void init() {
        setupAuto();
    }


    public void loop() {

        if (gotoWithOdo(.25, 10, 0)) {
            stop(); // End the program when target position is reached
        }
    }



}
