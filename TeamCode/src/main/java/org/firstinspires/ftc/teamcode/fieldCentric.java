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
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();
        // stop the program if button click

        if (isStopRequested()) return;

        boolean slowMode = false;

        while (opModeIsActive()) {
//
            telemetry.addData("stickdrift-y", Double.toString(gamepad2.left_stick_y));


            double y = Range.clip(gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);

            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //magnitude of joysticky
            double mag = Math.sqrt(x*x+y*y);

            //angle of joysticky
            double ang = Math.toDegrees(Math.atan2(x, y));

            //combined the joystick and robo angle

            double goob = ang-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            //only goes forward when you press a on

            if(gamepad1.a){
                motorFrontLeft.setPower(-Math.sin(goob-45));
                motorBackRight.setPower(-Math.sin(goob-45));

                motorFrontRight.setPower(Math.sin(goob+45));
                motorBackLeft.setPower(Math.sin(goob+45));

            }

            telemetry.update();

        }
    }
}
