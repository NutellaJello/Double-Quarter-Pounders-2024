//ftc package yay!
package org.firstinspires.ftc.teamcode;

//importing needed things
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
//@Disabled
//beginning of class
public class TeleOp2425 extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
//        //motors
        double dampSpeedRatio = 0.58;
        double dampTurnRatio = 0.4;
//
//        double flopSpeed = 0.000003;
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FrontLeft"); //0
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FrontRight"); //1
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BackLeft"); //2
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BackRight"); //3
//
//        Servo flopper = hardwareMap.servo.get("flopper"); //0
//        Servo claw = hardwareMap.servo.get("claw"); //1
//        Servo airplane = hardwareMap.servo.get("airplane");//expansion 2
        //flopper.setDirection(Servo.Direction.REVERSE);
//        //expansion
//        Servo leftarm = hardwareMap.servo.get("leftarm"); //port 0 lswing
//        Servo rightarm = hardwareMap.servo.get("rightarm"); //port 1 rswing
//        //imma make a double that updates the servo posi
//        rightarm.setDirection(Servo.Direction.REVERSE);
//        double sPosiL = 0.8;
//
//        DcMotor slides = hardwareMap.dcMotor.get("slides"); //0
//        DcMotorEx pullup = hardwareMap.get(DcMotorEx.class, "lifter"); //1
//        pullup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        double floposi = 0.305;

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        pullup.setDirection(DcMotorSimple.Direction.REVERSE);

        //slides.setDirection(DcMotorSimple.Direction.REVERSE);
//        pullup.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        pullup.setTargetPosition(0);
//        pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        /*slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/


//        airplane.setPosition (0.7);
        //wait for button click
        waitForStart();
        // stop the program if button click
        if (isStopRequested()) return;
        boolean slowMode = false;

        while (opModeIsActive()) {
//            telemetry.addData("leftarm", Double.toString(leftarm.getPosition()));
//            telemetry.addData("rightarm", Double.toString(rightarm.getPosition()));
//            telemetry.addData("claw", Double.toString(claw.getPosition()));
//            telemetry.addData("flopper", Double.toString(flopper.getPosition()));
//            telemetry.addData("slides", Double.toString(slides.getCurrentPosition()));
//            telemetry.addData("airplane", Double.toString(airplane.getPosition()));
//            telemetry.addData("pullup", Double.toString(pullup.getCurrentPosition()));
//            telemetry.addData("pullup-v", Double.toString(pullup.getVelocity()));
            telemetry.addData("drift", Double.toString(gamepad2.left_stick_y));


            ////////////////arm
//            leftarm.setPosition(sPosiL);
//            rightarm.setPosition(sPosiL);
//
//
//            double upPosi = 0.94;
//
//            if(sPosiL > upPosi){
//                sPosiL = upPosi;//setting minimmum so it dont go below the ground
//            }
//
//            if (gamepad2.dpad_up){
//                sPosiL = upPosi;//up
//
//            }
//            if (gamepad2.dpad_down){
//                sPosiL = 0.05;//down
//
//            }
//
//            if (gamepad2.dpad_right){
//                sPosiL = 0.12;//tweaker
//            }
//            if (gamepad2.dpad_left){
//                sPosiL = 0.6;//tweaker
//            }
//            if(gamepad2.left_bumper){
//                sPosiL = 0.08;
//            }
//
//            ////////////////claww
//            if(gamepad2.right_bumper){
//                claw.setPosition( 0.83);//open
//            }else {
//                claw.setPosition(0.62);//close
//            }
//
//
//
//            ///////////////////////flopper
//            if (gamepad2.right_trigger > 0 && sPosiL < upPosi -0.05){
//                //floposi = 0.72;//dump
//                floposi+= 0.0022 ;
//            }
//            else if (gamepad2.x) {
//                floposi = 0.305;//rest
//            }
//            else if (gamepad2.b  && sPosiL < upPosi -0.05){
//                floposi = 0.5;
//            }else if(gamepad2.a){
//                floposi = 0.275;
//            }
//            flopper.setPosition(floposi);
//            if (floposi > 0.70){
//                floposi = 0.70;
//            }
//
//
//            /// airplane
//            if (gamepad1.dpad_left){
//                airplane.setPosition (1);
//            }
//            else {
//                airplane.setPosition(0.8);
//            }
//
//            if(gamepad1.left_trigger>0){
//
//                pullup.setTargetPosition(3950);//up
//                pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pullup.setVelocity(1600);
//            }else if(gamepad1.right_trigger>0){
//
//                pullup.setTargetPosition(1500);//down
//                pullup.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                pullup.setVelocity(600);
//            }

//            if (gamepad2.left_stick_y > 0 ){
//                if (slides.getCurrentPosition() != -1640) {
//                    slides.setTargetPosition(slides.getCurrentPosition() + 50);
//                    slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slides.setPower(0.5);
//                }
//            }
//            else if (gamepad2.left_stick_y < 0){
//                if (!(slides.getCurrentPo
//                sition() < 50)) {
//                    slides.setTargetPosition(slides.getCurrentPosition() - 50);
//                    slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    slides.setPower(0.5);
//                }
//            }
//            else if (gamepad2.left_stick_y == 0){
//                slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }
            /*
            if(gamepad2.left_stick_y > 0.4 || gamepad2.left_stick_y < -0.4){
                slides.setPower(gamepad2.left_stick_y *0.5);
            }else{
                slides.setPower(0);
            }*/
//            slides.setPower(gamepad2.left_stick_y * 0.4);

            //9.0
            //-1750
//            int spee
//            if (gamepad1.left_trigger >0){
//                motorBackLeft.setTargetPosition(BackLeft.getCurrentPosition() + move);
//                motorFrontLeft.setTargetPosition(FrontLeft.getCurrentPosition() + move);
//                motorBackRight.setTargetPosition(BackRight.getCurrentPosition() + move);
//                motorFrontRight.setTargetPosition(FrontRight.getCurrentPosition() + move);
//                //
//                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //
//                motorFrontLeft.setPower(speed);
//                motorBackLeft.setPower(speed);
//                motorFrontRight.setVelocity(speed);
//                motorBackRight.setVelocity(speed);
//
//            }


            double y = Range.clip(gamepad1.left_stick_y, -1, 1);
            //left stick x value
            double x = Range.clip(gamepad1.left_stick_x, -1, 1);
            //right stick x value
            double rx = Range.clip(-gamepad1.right_stick_x, -1, 1);

            //    double arct = 0;

            double flPower = (y - x) * dampSpeedRatio + dampTurnRatio * rx;
            double frPower = (y + x) * dampSpeedRatio - dampTurnRatio * rx;
            double blPower = (y + x) * dampSpeedRatio + dampTurnRatio * rx;
            double brPower = (y - x) * dampSpeedRatio - dampTurnRatio * rx;

            double maxFront = Math.max(flPower, frPower);
            double maxBack = Math.max(blPower, brPower);
            double maxPower = Math.max(maxFront, maxBack);

            if (maxPower > 1.0) {
                flPower /= maxPower;
                frPower /= maxPower;
                blPower /= maxPower;
                brPower /= maxPower;
            }
            //finally moving the motors
            motorFrontLeft.setPower(flPower);
            motorBackLeft.setPower(blPower);
            motorFrontRight.setPower(frPower);
            motorBackRight.setPower(brPower);

            //sprint
//

            if(gamepad1.right_bumper || gamepad1.right_trigger > 0){
                dampSpeedRatio = 0.18;
                dampTurnRatio = 0.12;
            }else if(gamepad1.left_bumper || gamepad1.left_trigger > 0){
                dampSpeedRatio = 1.0;
            }else{
                dampSpeedRatio = 0.6;
                dampTurnRatio = 0.4;
            }

            telemetry.update();

        }
    }
}