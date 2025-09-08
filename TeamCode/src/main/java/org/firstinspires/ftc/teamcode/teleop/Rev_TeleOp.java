package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class Rev_TeleOp extends LinearOpMode {

    public static double bucketStow = 1;
    public static double bucketDeposit = 0.81;
    @Override
    public void runOpMode() throws InterruptedException {

        double left;
        double right;
        double drive;
        double turn;
        double max;

        DcMotor leftDrive = hardwareMap.dcMotor.get("left");
        DcMotor rightDrive = hardwareMap.dcMotor.get("right");
        DcMotor shootLeft = hardwareMap.dcMotor.get("shootLeft");
        DcMotor shootRight = hardwareMap.dcMotor.get("shootRight");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        shootLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo leftBucket = hardwareMap.servo.get("leftBucket");
//        Servo rightBucket = hardwareMap.servo.get("rightBucket");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

//            right.setPower(gamepad1.left_stick_y);
//            left.setPower(gamepad1.right_stick_y);

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            drive = gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x * 0.7;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            leftDrive.setPower(left);
            rightDrive.setPower(right);

            if (gamepad1.right_bumper) {
                shootLeft.setPower(1);
                shootRight.setPower(1);
            } else {
                shootLeft.setPower(0);
                shootRight.setPower(0);
            }

//            if (gamepad1.x) {
//                leftBucket.setPosition(0.5);
//                sleep(300);
//                leftBucket.setPosition(0);
//            } else if (gamepad1.b) {
//                rightBucket.setPosition(0.5);
//                sleep(300);
//                rightBucket.setPosition(0);
//            }

            if(gamepad1.a){
                leftBucket.setPosition(bucketDeposit);
            } else {leftBucket.setPosition(bucketStow);}
        }
    }

}
