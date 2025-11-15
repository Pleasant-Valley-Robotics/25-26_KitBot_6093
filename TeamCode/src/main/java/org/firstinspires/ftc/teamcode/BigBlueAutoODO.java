package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="Blue Big Auto", group="Kitbot")
@Disabled
public class BigBlueAutoODO extends AutoTemplateODO {
    @Override
    public void runOpMode() {
        setupAuto();

        while (opModeIsActive()) {
            telemetry.addData("Current Heading", getHeading());
        }
    }
}
