package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

public class OdometryTeleop extends OpMode {
    final double LAUNCHER_FAR_VELOCITY = 1575;
    final double LAUNCHER_CLOSE_VELOCITY = 1100;
    final double LAUNCHER_CYCLE_VELOCITY = 450;

    double CURRENT_TARGET_VELOCITY = 1575;
    boolean manualControl;


    final double FEED_TIME = .35;
    final double BACK_TIME = .5;
    ElapsedTime feederTimer = new ElapsedTime(20);

    int targetAprilTag = 0;




    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    MecanumDrive drive = null;
    Shooter shooter = null;


    @Override
    public void init() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Shooter shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        drive.updatePoseEstimate();

        // Retrieve your pose
        Pose2d myPose = drive.localizer.getPose();

        if (gamepad1.xWasPressed()) {
            manualControl = !manualControl;
        }

        if (gamepad2.xWasPressed()) {
            CURRENT_TARGET_VELOCITY = LAUNCHER_CLOSE_VELOCITY;
        }




        SequentialAction shoot = new SequentialAction(
        shooter.spinUp(CURRENT_TARGET_VELOCITY),
                shooter.fireBall()
        );

        SequentialAction index = new SequentialAction(
                shooter.spinUp(450),
                shooter.fireBall()
        );









        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;


        arcadeDrive(forward, rotate);

        dash.sendTelemetryPacket(packet);
    }

    void arcadeDrive(double forward, double rotate) {
        double y = forward;
        double x = gamepad1.left_stick_x * 1.1; // Strafe
        double r = rotate;

        double frontLeftPower  = y + x + r;
        double frontRightPower = y - x - r;
        double backLeftPower   = y - x + r;
        double backRightPower  = y + x - r;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        drive.leftFront.setPower(frontLeftPower);
        drive.rightFront.setPower(frontRightPower);
        drive.leftBack.setPower(backLeftPower);
        drive.rightBack.setPower(backRightPower);
    }

}