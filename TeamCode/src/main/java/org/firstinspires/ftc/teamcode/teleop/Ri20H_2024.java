//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.arcrobotics.ftclib.controller.PIDFController;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//@TeleOp
//@Config
//public class Ri30H_2024 extends LinearOpMode {
//
//    public static int armAngleIntake = 100;
//    public static int armAngleScore = 300;
//
//    public static int armExtensionIntake = 20;
//    public static int armExtensionScoreLow = 300;
//    public static int armExtensionScoreMid = 400;
//    public static int armExtensionScoreHigh = 550;
//
//    //    public static int climbStowed = 0;
////    public static int climbClimb = 0;
////    public static int climbHalfUp = 0;
//    public PIDFController angleController;
//    public static double aP = 0.0005;
//    public static double aI = 0;
//    public static double aD = 0.01;
//    public static double aF = 0;
//    public int angleTarget;
//    public PIDFController extensionController;
//    public static double eP = 0.007;
//    public static double eI = 0;
//    public static double eD = 0.01;
//    public static double eF = 0;
//    public int extensionTarget;
////    public PIDFController climbController;
////    public static double cP = 0;
////    public static double cI = 0;
////    public static double cD = 0;
////    public static double cF = 0;
////    public int climbTarget;
//
//    public int anglePos;
//    public int extensionPos;
//    //    public int climbPos;
//    public double anglePIDF;
//    public double extensionPIDF;
////    public double climbPIDF;
//
//    public double y;
//    public double x;
//    public double rx;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Declare our motors
//        // Make sure your ID's match your configuration
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("backRightMotor");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//
//        DcMotor armAngle = hardwareMap.dcMotor.get("armAngle");
//        DcMotor armExtension = hardwareMap.dcMotor.get("armExtension");
//        DcMotor climb = hardwareMap.dcMotor.get("climb");
//
//        CRServo intake = hardwareMap.crservo.get("intake");
//
//        // Reverse the right side motors. This may be wrong for your setup.
//        // If your robot moves backwards when commanded to go forwards,
//        // reverse the left side instead.
//        // See the note about this earlier on this page.
////        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
////        armAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        climb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//
////            angleController = new PIDFController(aP, aI, aD, aF);
//            extensionController = new PIDFController(eP, eI, eD, eF);
////            climbController = new PIDFController(cP, cI, cD, cF);
//
////            anglePos = armAngle.getCurrentPosition();
//            extensionPos = armExtension.getCurrentPosition();
////            climbPos = climb.getCurrentPosition();
//
////            anglePIDF = angleController.calculate(anglePos, angleTarget);
//            extensionPIDF = extensionController.calculate(extensionPos, extensionTarget);
////            climbPIDF = climbController.calculate(climbPos, climbTarget);
//
////            armAngle.setPower(anglePIDF);
//            armExtension.setPower(extensionPIDF);
////            climb.setPower(climbPIDF);
//
////            armAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            climb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//            if (gamepad1.left_bumper) {
//                y = -gamepad1.left_stick_y * 0.3;
//                x = gamepad1.left_stick_x * 0.3;
//                rx = -gamepad1.right_stick_x * 0.3 + (-0.2 * gamepad2.left_stick_x);
//            } else {
//                y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//                x = gamepad1.left_stick_x;
//                rx = -gamepad1.right_stick_x * 0.7 + (-0.2 * gamepad2.left_stick_x);
//            }
//            // This button choice was made so that it is hard to hit on accident,
//            // it can be freely changed based on preference.
//            // The equivalent button is start on Xbox-style controllers.
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            // Denominator is the largest motor power (absolute value) or 1
//            // This ensures all the powers maintain the same ratio,
//            // but only if at least one is out of the range [-1, 1]
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            frontLeftMotor.setPower(frontLeftPower);
//            backLeftMotor.setPower(backLeftPower);
//            frontRightMotor.setPower(frontRightPower);
//            backRightMotor.setPower(backRightPower);
//
//
//            if (gamepad2.right_bumper) {
//                intake.setPower(1);
//            } else if (gamepad2.left_bumper) {
//                intake.setPower(-1);
//            } else {
//                intake.setPower(0);
//            }
//
//
//            if (gamepad2.a) {
////                angleTarget = armAngleIntake;
//                extensionTarget = armExtensionIntake;
//            } else if (gamepad2.b) {
////                angleTarget = armAngleScore;
//                extensionTarget = armExtensionScoreLow;
//            } else if (gamepad2.y) {
////                angleTarget = armAngleScore;
//                extensionTarget = armExtensionScoreMid;
//            } else if (gamepad2.x) {
//                extensionTarget = armExtensionScoreHigh;
//            }
//
//
////            if (gamepad2.dpad_up) {
////                climbTarget = climbClimb;
////            } else if (gamepad2.dpad_right) {
////                climbTarget = climbHalfUp;
////            } else if (gamepad2.dpad_down) {
////                climbTarget = climbStowed;
////            }
//
//
//            if (gamepad2.left_trigger > 0) {
//                double armAngleControl = (0.4 * -gamepad2.left_stick_y);
//                armAngle.setPower(armAngleControl);
//            } else {
//                double armAngleControl = 0.1 + (0.5 * -gamepad2.left_stick_y);
//                armAngle.setPower(armAngleControl);
//            }
//
////            double armExtensionControl = gamepad2.left_stick_x;
////
//
////            armExtension.setPower(armExtensionControl);
////
////
////            double climbControl = gamepad2.right_stick_y;
////
////            climb.setPower(climbControl);
////
////
////            intake.setPower(gamepad2.right_stick_x);
//
//
//            telemetry.addData("armAnglePos:", anglePos);
//            telemetry.addData("armAngleTarget:", angleTarget);
//            telemetry.addData("armExtensionPos:", extensionPos);
//            telemetry.addData("armExtensionTarget:", extensionTarget);
//            telemetry.update();
//        }
//    }
//}