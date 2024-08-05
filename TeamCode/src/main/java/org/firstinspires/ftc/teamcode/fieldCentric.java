//ftc package yay!
package org.firstinspires.ftc.teamcode;

//importing needed things
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
//@Disabled
//beginning of class
public class fieldCentric extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
//        //motors
        double dampSpeedRatio = 0.58;
        double dampTurnRatio = 0.4;

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft"); //0
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FrontRight"); //1
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BackLeft"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BackRight"); //3
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu         = null;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        // stop the program if button click

        if (isStopRequested()) return;

        boolean slowMode = false;

        while (opModeIsActive()) {


            double y = Range.clip(gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);

            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //magnitude of joysticky
            double mag = Range.clip(Math.abs(Math.sqrt(x*x+y*y)), 0, 1);

            //angle of joysticky
            double ang = Math.toDegrees(Math.atan2(x, y));

            //combined the joystick and robo angle

            double goob = ang-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            double maxPower = Math.max(-Math.sin(goob-45), Math.sin(goob+45));

            double flPower = mag*(-Math.sin(goob-45)) + 0.5*rx;
            double frPower = mag*(Math.sin(goob+45))- 0.5*rx;
            double blPower = mag*(Math.sin(goob+45))+ 0.5*rx;
            double brPower = mag*(-Math.sin(goob-45)) -0.5*rx;

            /*
            flPower/= maxPower;
            frPower/= maxPower;
            blPower/= maxPower;
            brPower/= maxPower;
    */

            motorFrontLeft.setPower(flPower);
            motorBackLeft.setPower(blPower);
            motorFrontRight.setPower(frPower);
            motorBackRight.setPower(brPower);


            telemetry.addData("yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) );


            telemetry.update();

        }
    }
}
