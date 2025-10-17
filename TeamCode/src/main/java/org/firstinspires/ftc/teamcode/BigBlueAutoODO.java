package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Big Auto", group="Kitbot")
public class BigBlueAutoODO extends AutoTemplateODO {
    @Override
    public void runOpMode() {
        setupAuto();

        while (opModeIsActive()) {
            turnToHeading(.30, 0);
        }
    }
}
