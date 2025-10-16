package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Big Auto - kitbot", group="Robot")
public class BigBlueAuto extends AutoTemplate {
    @Override
    public void runOpMode() {
        setupAuto();

        while (opModeIsActive()) {
            holdHeading(.30, 0);
        }
    }
}
