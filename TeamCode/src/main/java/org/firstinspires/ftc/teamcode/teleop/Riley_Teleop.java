package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp

public class Riley_Teleop extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;

    public double y;
    public double x;
    public double rx;

    public static double latchOpen = 0.38;
    public static double latchClosed = 0;

    public static float velocityOne = -9000;
    public static float velocityTwo = -6800;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(250,0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {
        //initAprilTag();

        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor indexer = hardwareMap.dcMotor.get("indexer");
//        DcMotorEx shootOne = hardwareMap.dcMotorEx.get("shootOne");
//        DcMotorEx shootTwo = hardwareMap.dcMotorEx.get("shootTwo");
        DcMotorEx shootOne = hardwareMap.get(DcMotorEx.class, "shootOne");
        DcMotorEx shootTwo = hardwareMap.get(DcMotorEx.class, "shootTwo");

        shootOne.setDirection(DcMotorSimple.Direction.REVERSE);
        shootTwo.setDirection(DcMotorSimple.Direction.FORWARD);
//Used for traditional setVelocity code
//        shootOne.setTargetPosition(0);
//        shootTwo.setTargetPosition(0);
//        shootOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        shootTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Servo latch = hardwareMap.servo.get("latch");

//         Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Shoot One", "%.2f", shootOne.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Shoot Two", "%.2f", shootTwo.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("Velocity One", "%.2f", velocityOne);
            telemetry.addData("Velocity Two", "%.2f", velocityTwo);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            if (gamepad1.left_bumper) {
                y = -gamepad1.left_stick_y * 0.3;
                x = gamepad1.left_stick_x * 0.3;
                rx = gamepad1.right_stick_x * 0.3 + (-0.2 * gamepad2.left_stick_x);
            } else {
                y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x * 0.7 + (-0.2 * gamepad2.left_stick_x);
            }
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if (gamepad2.right_bumper) {
                intake.setPower(1);
            } else if (gamepad2.dpad_up) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad2.a) {
                indexer.setPower(1);
            } else if (gamepad2.b) {
                indexer.setPower(-1);
            } else {
                indexer.setPower(0);
            }
//Traditional run to velocity shooter code
//            if (gamepad1.dpad_down) {
//                shootOne.setVelocity(-6900);
//                shootTwo.setVelocity(6900);
//            } else {
//                shootOne.setVelocity(0);
//                shootTwo.setVelocity(0);
//            }

            if(gamepad2.dpad_down){
                shootOne.setVelocity(velocityOne, AngleUnit.DEGREES);
                shootTwo.setVelocity(velocityTwo, AngleUnit.DEGREES);
            } else{
                shootOne.setVelocity(0);
                shootTwo.setVelocity(0);
            }
            shootOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
            shootTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

            if (gamepad2.x) {
                latch.setPosition(latchOpen);
            } else if (gamepad2.y) {
                latch.setPosition(latchClosed);
            }
            //telemetryAprilTag();
        }
    }
//    private void initAprilTag() {
//
//        // Create the AprilTag processor the easy way.
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
//
//        // Create the vision portal the easy way.
//        if (USE_WEBCAM) {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
//        } else {
//            visionPortal = VisionPortal.easyCreateWithDefaults(
//                    BuiltinCameraDirection.BACK, aprilTag);
//        }
//
//    }   // end method initAprilTag()
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//    }   // end method telemetryAprilTag()
}
