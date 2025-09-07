//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
////import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
////import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
///*
//
//────────────────────────────────────────────────────────────────────────────
//       ___
//     /     \
//    |  LT  |                            -> Slow Turn
//     \ ___ /
//                             ___
//                           /     \
//                           |  RT  |     -> Slow Turn
//                           \ ___ /
//
//       [LB]                             -> (Pickup, Deposit)
//                            [RB]        -> Slow Driving
//
//         ↑                              -> (Set Upper Box)
//           →                            -> asdf
//       ←                                -> asdf
//         ↓                              -> (Set Lower Box)
//
//                           (Y)          -> asdf
//                              (B)       -> (Starting Position)
//                        (X)             -> (Deposit Position (Saved Height))
//                           (A)          -> (Pickup Position)
//
//       [LS]                             -> Translation Driving
//      ( O )                             -> asdf
//
//                          [RS]          -> Rotation Driving
//                          ( O )         -> asdf
//
//        [SELECT]                        -> asdf
//                         [START]        -> Reset Yaw
//
//────────────────────────────────────────────────────────────────────────────
//       ___                    ___
//     /     \                /     \
//    | L2   |                | R2   |          ← Triggers
//     \ ___ /                \ ___ /
//
//      [L1]                    [R1]            ← Shoulder buttons
//
//        ↑                     (Y)             ← Top button
//      ←   →                (X)   (B)          ← Left / Right face buttons
//        ↓                     (A)             ← Bottom button
//
//       [LS]                  [RS]             ← Analog sticks
//      ( O )                  ( O )            ← Center buttons
//
//     [SELECT]              [START]            ← Menu / Pause buttons
//      [  ]                   [▶]
//
//────────────────────────────────────────────────────────────────────────────
//
// */
//@TeleOp
//@Config
//public class Scrimbo_Testing extends LinearOpMode {
//    public enum ArmState {
//        STOW,
//        INTAKE,
//        SCORE_READY,
//        SCORE_LOW,
//        SCORE_HIGH,
//        SCORE_COMPLETE,
//        HOVER
//    }
//
//    public static int armAngleStow = 0;
//    public static int armAngleIntake = 100;
//    public static int armAngleIntakePrep = 300; //was 300
//    public static int armAngleSpec = 800;
//    public static int armAngleBasket = 1250; //was 1250
//
//    public static int armExtensionStowed = 10;
//    public static int armExtensionSubmersible = 500;
//    public static int armExtensionLowBasket = 1100;
//    public static int armExtensionHighBasket = 1900;
//
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
//
//    public int anglePos;
//    public int extensionPos;
//    public double anglePIDF;
//    public double extensionPIDF;
//
//    public double y;
//    public double x;
//    public double rx;
//
//    public float coDriveTurn;
//
//    public boolean wristStart;
//    public boolean clawStart;
//    public boolean scoring;
//
//    public static double clawOpen = 0.7;
//    public static double clawClosed = 0;
//    public static double wristForwards = 1;
//    public static double wristBackwards = 0.2;
//
//    boolean prevRbumper;
//    boolean prevLbumper;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        DcMotorEx encoder = hardwareMap.get(DcMotorEx.class, "encoder");
//
//        DcMotor pivot = hardwareMap.dcMotor.get("pivot_driver");
//        DcMotor armExtension = hardwareMap.dcMotor.get("extend");
//
//        Servo claw = hardwareMap.servo.get("claw");
//        Servo wrist = hardwareMap.servo.get("wrist");
//
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
////        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
////
////        encoder.setTargetPosition(0);
////
////        encoder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//
//        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
//        imu.initialize(parameters);
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetry.addData(">", "Robot Ready.  Press Play.");
//        telemetry.update();
//
//
//
//        // Setup
//        wristStart = true;
//        clawStart = false;
//        scoring = false;
//
//        claw.setPosition(clawOpen);
//        wrist.setPosition(wristForwards);
//
//        pivot.setTargetPosition(armAngleStow);
//        armExtension.setTargetPosition(armExtensionStowed);
//
//        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
////        pivot.setPower(0.5);
////        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        pivot.setPower(0.6);
//        armExtension.setPower(1);
////        pivot.setTargetPosition(armAngleSpec);
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            ArmState armState = ArmState.HOVER;
//
////            switch (armState){
////                case STOW:
////                    wrist.setPosition(wristForwards);
////                    pivot.setTargetPosition(armAngleSpec);
////                    armExtension.setTargetPosition(armExtensionStowed);
////                case HOVER:
////                    claw.setPosition(clawClosed);
////                    wrist.setPosition(wristForwards);
////                    pivot.setTargetPosition(armAngleIntakePrep);
////                    if(gamepad2.right_trigger>0.1){
////                        armState = ArmState.INTAKE;
////                    }
////                case INTAKE:
////                    wrist.setPosition(wristForwards);
////                    pivot.setTargetPosition(armAngleIntake);
////                    if(gamepad2.right_trigger<0.1){
////                        armState = ArmState.HOVER;
////                    }
////                    if(gamepad2.y){
////                        armState = ArmState.SCORE_READY;
////                    }
////                case SCORE_READY:
////                    pivot.setTargetPosition(armAngleBasket);
////                    wrist.setPosition(wristBackwards);
////                    if(gamepad2.a){
////                        armState = ArmState.SCORE_LOW;
////                    }
////                case SCORE_HIGH:
////                    armExtension.setTargetPosition(armExtensionHighBasket);
////                case SCORE_LOW:
////                    armExtension.setTargetPosition(armExtensionLowBasket);
////                    if (gamepad2.b){
////                        armState = ArmState.SCORE_COMPLETE;
////                    }
////                case SCORE_COMPLETE:
////                    armExtension.setTargetPosition(armExtensionStowed);
////                    if(gamepad2.x){
////                        armState = ArmState.HOVER;
////                    }
////
////
////            }
//
////            angleController = new PIDFController(aP, aI, aD, aF);
////            extensionController = new PIDFController(eP, eI, eD, eF);
//
////            anglePos = pivot.getCurrentPosition();
////            extensionPos = armExtension.getCurrentPosition();
//
////            anglePIDF = angleController.calculate(anglePos, angleTarget);
////            extensionPIDF = extensionController.calculate(extensionPos, extensionTarget);
//
////            pivot.setPower(anglePIDF);
////            armExtension.setPower(extensionPIDF);
//
////            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////            armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            // Operator Turning Adjustment
////            if (gamepad2.left_trigger > 0.1) {
////                coDriveTurn = gamepad2.left_trigger;
////            } else if (gamepad2.right_trigger > 0.1) {
////                coDriveTurn = -gamepad2.right_trigger;
////            } else {
////                coDriveTurn = 0;
////            }
//
//
//            if (gamepad1.right_bumper) {
//                y = -gamepad1.left_stick_y * 0.3;
//                x = gamepad1.left_stick_x * 0.3;
//                rx = gamepad1.right_stick_x * 0.3 + (-0.2 * coDriveTurn);
//            } else {
//                y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
//                x = gamepad1.left_stick_x;
//                rx = gamepad1.right_stick_x * 0.7 + (-0.2 * coDriveTurn);
//            }
//
//            if (gamepad1.options) {
//                imu.resetYaw();
//            }
//
//            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//            double botHeading = 0;
//            // Rotate the movement direction counter to the bot's rotation
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            frontLeftMotor.setPower(backRightPower);
//            backLeftMotor.setPower(frontLeftPower);
//            frontRightMotor.setPower(backLeftPower);
//            backRightMotor.setPower(frontRightPower);
//
//
//            if (gamepad2.left_bumper & !prevLbumper & wristStart) {
//                wrist.setPosition(wristBackwards);
//                wristStart = false;
//            } else if (gamepad2.left_bumper & !prevLbumper & !wristStart) {
//                wrist.setPosition(wristForwards);
//                wristStart = true;
//            }
//
//            if (gamepad2.right_trigger>0.1 & !prevRbumper & clawStart) {
//                pivot.setTargetPosition(armAngleIntake);
//                sleep(300);
//                claw.setPosition(clawOpen);
//                sleep(500);
//                pivot.setTargetPosition(armAngleIntakePrep);
////                clawStart = false;
////                claw.setPosition(clawClosed);
////                pivot.setTargetPosition(armAngleIntake);
//            } else if (gamepad2.right_trigger>0.1 & !prevRbumper & !clawStart){
////                claw.setPosition(clawClosed);
//                clawStart = true;
//                claw.setPosition(clawOpen);
////                pivot.setTargetPosition(armAngleIntakePrep);
//            }
//
//            if (gamepad2.right_bumper) {
//                claw.setPosition(clawClosed);
//            }
//            if (gamepad2.a) {
//                armExtension.setTargetPosition(armExtensionLowBasket);
//            } else if (gamepad2.b) {
//                scoring = false;
//                wrist.setPosition(wristForwards);
//                wristStart = true;
//                pivot.setTargetPosition(armAngleIntakePrep);
//                armExtension.setTargetPosition(armExtensionStowed);
//            } else if (gamepad2.y) {
//                scoring = true;
//                wrist.setPosition(wristBackwards);
//                wristStart = false;
//                pivot.setTargetPosition(armAngleBasket);
//                armExtension.setTargetPosition(armExtensionHighBasket);
//            } else if (gamepad2.x) {
//                armExtension.setTargetPosition(armExtensionSubmersible);
//            }
//
//            if(gamepad2.right_bumper && !prevRbumper && clawStart){
//                claw.setPosition(clawOpen);
//                clawStart = false;
//            }
//            if(gamepad2.right_bumper && !prevRbumper && !clawStart){
//                claw.setPosition(clawClosed);
//                clawStart = true;
//            }
//
//
//            prevRbumper = gamepad2.right_bumper;
//            prevLbumper = gamepad2.left_bumper;
//
//            //pivot.setPower(-gamepad2.left_stick_y);
//
////            if (gamepad2.left_stick_y > 0.3) {
////                int target = pivot.getTargetPosition();
////                target += 1;
////                pivot.setTargetPosition(target);
////            } else if (gamepad2.left_stick_y < 0.3) {
////                int target = pivot.getTargetPosition();
////                target -= 1;
////                pivot.setTargetPosition(target);
////            }
//
////            armExtension.setPower(gamepad2.right_stick_y);
////
////            if (gamepad2.right_stick_y > 0.3) {
////                int target = armExtension.getCurrentPosition();
////                target += 1;
////                armExtension.setTargetPosition(target);
////            } else if (gamepad2.right_stick_y < 0.3) {
////                int target = armExtension.getTargetPosition();
////                target -= 1;
////                armExtension.setTargetPosition(target);
////            }
//
//
////            if (gamepad2.a) {
////                armExtension.setTargetPosition(armExtensionLowBasket);
////            } else if (gamepad2.b) {
////                wrist.setPosition(wristForwards);
////                wristStart = true;
////                pivot.setTargetPosition(armAngleIntakePrep);
////                armExtension.setTargetPosition(armExtensionStowed);
////            } else if (gamepad2.y) {
////                wrist.setPosition(wristBackwards);
////                wristStart = false;
////                pivot.setTargetPosition(armAngleBasket);
////                armExtension.setTargetPosition(armExtensionHighBasket);
////            } else if (gamepad2.x) {
////                armExtension.setTargetPosition(armExtensionSubmersible);
////            }
//
//            if (gamepad2.dpad_left) {
//                pivot.setTargetPosition(armAngleSpec);
//            } else if (gamepad2.dpad_down) {
//                pivot.setTargetPosition(armAngleIntake);
//            } else if (gamepad2.dpad_right) {
//                pivot.setTargetPosition(armAngleIntakePrep);
//            } else if (gamepad2.dpad_up) {
//                armExtension.setTargetPosition(armExtensionHighBasket);
////                pivot.setTargetPosition(armAngleBasket);
//            }
//
//
//            telemetry.addData("armAnglePos:", pivot.getCurrentPosition());
//            telemetry.addData("armAngleTarget:", pivot.getTargetPosition());
//            telemetry.addData("armExtensionPos:", armExtension.getCurrentPosition());
//            telemetry.addData("armExtensionTarget:", armExtension.getTargetPosition());
//            telemetry.addData("Arm Encoder Angle:", encoder.getCurrentPosition());
//            telemetry.update();
//        }
//    }
//}