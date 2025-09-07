package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp

public class Adam_Teleop extends LinearOpMode {
    public double y;
    public double x;
    public double rx;

    @Override
    public void runOpMode() throws InterruptedException {
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

        shootOne.setTargetPosition(0);
        shootTwo.setTargetPosition(0);
        shootOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shootTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            if (gamepad1.right_bumper) {
                intake.setPower(1);
            } else if (gamepad1.dpad_up) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.a) {
                indexer.setPower(1);
            } else if (gamepad1.b) {
                indexer.setPower(-1);
            } else {
                indexer.setPower(0);
            }

            if (gamepad2.a) {
                shootOne.setVelocity(6900);
                shootTwo.setVelocity(-6900);
            }
//            shootOne.setPower(gamepad2.left_stick_y);
//            shootTwo.setPower(-gamepad2.right_stick_y);
        }
    }
}
