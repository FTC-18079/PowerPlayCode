package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "GooseTeleOp", group = "FTC 18079")
public class Drive23 extends LinearOpMode {
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private DcMotorEx arm;

    private Servo claw;
    private Servo leftFlick;
    private Servo rightFlick;

    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;
    boolean headless = false;
    double lastAngle = 0;

    BNO055IMU imu;
    Orientation angles;

    boolean limitEnabled = true;
    boolean limitReached = true;
    double deadZone = 0.05;
    double driveSens = 1.0;
    double armSens = 0.7;

    public DcMotorEx initMotor(String name, Boolean brake, Boolean reverse, Boolean withoutEncoder){
        DcMotorEx motor;
        motor = hardwareMap.get(DcMotorEx.class, name);
        if (brake) motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        if (reverse) motor.setDirection(DcMotorEx.Direction.REVERSE);
        else motor.setDirection(DcMotorEx.Direction.FORWARD);

        if (withoutEncoder) motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        else motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        return motor;
    }

    @Override
    public void runOpMode() {
        //name, brake, reverse, withoutEncoder, toPosition, defaultRunMode
        leftFront = initMotor("leftFront", true, false, true);
        rightFront = initMotor("rightFront", true, false, true);
        leftBack = initMotor("leftBack", true, false, true);
        rightBack = initMotor("rightBack", true, true, true);
        arm = initMotor("arm", true, false, true);

        claw = hardwareMap.get(Servo.class, "claw");
        leftFlick = hardwareMap.get(Servo.class, "leftFlick");
        rightFlick = hardwareMap.get(Servo.class, "rightFlick");

        //Initialize IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");
        waitForStart();
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            //Gamepad 1
            //Drive
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            double heading = angles.firstAngle - lastAngle;

            double x = -gamepad1.left_stick_x * 1.1;
            double y = gamepad1.left_stick_y;
            double spinny_winny = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            if(headless){
                double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
                double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(spinny_winny), 1);
                frontLeftPower = (rotY + rotX + spinny_winny) / denominator;
                backLeftPower = (rotY - rotX + spinny_winny) / denominator;
                frontRightPower = (rotY - rotX - spinny_winny) / denominator;
                backRightPower = (rotY + rotX - spinny_winny) / denominator;
            }else{
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(spinny_winny), 1);
                frontLeftPower = (y + x + spinny_winny) / denominator;
                backLeftPower = (y - x + spinny_winny) / denominator;
                frontRightPower = (y - x - spinny_winny) / denominator;
                backRightPower = (y + x - spinny_winny) / denominator;
            }

            leftFront.setPower(frontLeftPower * driveSens);
            leftBack.setPower(backLeftPower * driveSens);
            rightFront.setPower(frontRightPower * driveSens);
            rightBack.setPower(backRightPower * driveSens);

            //Speed toggle
            if(gamepad1.right_trigger > 0.15f){
                driveSens = 0.25;
            }else if(gamepad1.left_trigger > 0.15f){
                driveSens = 0.5;
            }else driveSens = 1.0;

            //Headless toggle
            if(gamepad1.a) headless = true;
            else if(gamepad1.b) headless = false;

            //Headless reset
            if(gamepad1.y){
                lastAngle = angles.firstAngle;
            }

            telemetry.addData("X", x);
            telemetry.addData("Y", y);
            telemetry.addData("RX", spinny_winny);
            telemetry.addData("Headless", headless);
            telemetry.addData("LF", frontLeftPower);
            telemetry.addData("RF", frontRightPower);
            telemetry.addData("LB", backLeftPower);
            telemetry.addData("RB", backRightPower);

            //Gamepad 2
            //Speed Toggle
            if(gamepad2.right_trigger > 0.25f){
                armSens = 0.2;
            }else armSens = 0.7;

            //Lift limit
            if(!limitEnabled){
                limitReached = false;
            }else{
                limitReached = arm.getCurrentPosition() >= 0;
                if(gamepad2.x){
                    limitEnabled = false;
                }
            }

            if(gamepad2.y){
                limitEnabled = true;
                arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            }

            //Arm
            if(gamepad2.left_stick_y > deadZone && !limitReached){
                arm.setPower(gamepad2.left_stick_y * armSens);
            }else if(gamepad2.left_stick_y < deadZone){
                arm.setPower(gamepad2.left_stick_y * armSens);
            }else arm.setPower(0);

            telemetry.addData("","");
            telemetry.addData("ArmPos", arm.getCurrentPosition());
            telemetry.addData("Arm", gamepad2.left_stick_y);

            //Claw
            if(gamepad2.a){
                claw.setPosition(0.55);
            }else if(gamepad2.b){
                claw.setPosition(0.215);
            }
            if(claw.getPosition() == 0.55) telemetry.addData("Claw","Open");
            else if(claw.getPosition() == 0.215) telemetry.addData("Claw","Claw");
            else telemetry.addData("Claw","Moving");

            //Left Flicker
            if(gamepad2.left_bumper){
                leftFlick.setPosition(1);
            }else{
                leftFlick.setPosition(0);
            }
            if(leftFlick.getPosition() == 1) telemetry.addData("Left Wing","Down");
            else if(leftFlick.getPosition() == 0) telemetry.addData("Left Wing","Up");
            else telemetry.addData("Left Wing","Moving");

            //Right Flicker
            if(gamepad2.right_bumper){
                rightFlick.setPosition(0);
            }else{
                rightFlick.setPosition(1);
            }
            if(rightFlick.getPosition() == 0) telemetry.addData("Right Wing","Down");
            else if(rightFlick.getPosition() == 1) telemetry.addData("Right Wing","Up");
            else telemetry.addData("Right Wing","Moving");

            telemetry.update();
        }
    }
}