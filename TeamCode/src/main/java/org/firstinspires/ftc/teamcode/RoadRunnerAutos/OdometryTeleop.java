package org.firstinspires.ftc.teamcode.RoadRunnerAutos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "Odometry Teleop", group = "StarterBot")
public class OdometryTeleop extends OpMode {
    final double LAUNCHER_FAR_VELOCITY = 1610;
    final double LAUNCHER_CLOSE_VELOCITY = 1400;
    final double LAUNCHER_CYCLE_VELOCITY = 480;

    double CURRENT_TARGET_VELOCITY = 1575;
    boolean manualRotate = true;
    boolean manualDrive = true;


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
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        shooter = new Shooter(hardwareMap);
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        TelemetryPacket packet = new TelemetryPacket();

        double rotate;

        SequentialAction index = new SequentialAction(
                shooter.spinUp(450),
                shooter.fireBall()
        );





        if (gamepad1.xWasPressed()) manualRotate = !manualRotate;
        if (gamepad2.xWasPressed()) runningActions.add(shooter.spinUp(LAUNCHER_CLOSE_VELOCITY));
        if (gamepad2.bWasPressed()) runningActions.add(shooter.spinUp(LAUNCHER_CYCLE_VELOCITY));
        if (gamepad2.yWasPressed()) runningActions.add(shooter.spinUp(LAUNCHER_FAR_VELOCITY));
        if (gamepad2.aWasPressed()) runningActions.add(shooter.stopSpin());
        if (gamepad1.bWasPressed()) drive.localizer.setPose(new Pose2d(0, 0, 0));
        if (gamepad2.right_bumper) runningActions.add(shooter.fireBall());
        if (gamepad1.dpad_down) manualDrive = true;

        if (gamepad1.aWasPressed()) {
            drive.localizer.setPose(new Pose2d(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, 0));
        }

        if (gamepad1.yWasPressed()) {
            manualDrive = false;
            TrajectoryActionBuilder park = drive.actionBuilder(drive.localizer.getPose()).strafeToLinearHeading(new Vector2d(43.4175, 18.4409 * PoseStorage.isRed), Math.toRadians(0.0));
            runningActions.add(park.build());

        }

        if (!manualRotate) {
            rotate = autoLockAngle();
        } else {
            rotate = gamepad1.right_stick_x;
        }

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }


        if (manualDrive) {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, rotate);
        }

        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();

        runningActions = newActions;
        dash.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        shooter.stopSpin();
        drive.leftFront.setPower(0);
        drive.rightFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
    }


    // Copied from FieldCentric.java
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                drive.localizer.getPose().heading.toDouble());

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    public double autoLockAngle() {
        double xDif = (-72) - drive.localizer.getPose().position.x;
        double yDif =  (72 * PoseStorage.isRed) - drive.localizer.getPose().position.y;
        double tolerance = 0.03; // Tolerance in inches
        double targetHeading = Math.atan(xDif/yDif);

        double deviation = targetHeading+drive.localizer.getPose().heading.toDouble();


        if (Math.abs(deviation) > tolerance) {
            double kP = 1.0;
            double turnPower = kP * deviation;


            return Math.max(-0.6, Math.min(0.6, turnPower));
        } else {
            // We are aligned, so command no turn.
            return 0.0;
        }
    }



    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        drive.leftFront.setPower(maxSpeed * (frontLeftPower / maxPower));
        drive.rightFront.setPower(maxSpeed * (frontRightPower / maxPower));
        drive.leftBack.setPower(maxSpeed * (backLeftPower / maxPower));
        drive.rightBack.setPower(maxSpeed * (backRightPower / maxPower));
    }
}