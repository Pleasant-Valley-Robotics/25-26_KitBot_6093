package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
@TeleOp(name = "Test Auto", group = "wasd")
public class TestAuto extends OpMode {
    @Override
    public void init() {


    }
    public void start() {
        Shooter shooter = new Shooter(hardwareMap);
        Actions.runBlocking(new SequentialAction(shooter.spinUp(300),
                shooter.rumble(gamepad1)));
    }


    @Override
    public void loop() {
        return;
    }


}
