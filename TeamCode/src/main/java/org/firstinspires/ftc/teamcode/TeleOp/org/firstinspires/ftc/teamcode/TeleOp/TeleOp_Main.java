package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class TeleOp_Main extends OpMode{
    public DcMotor mTE, mTD, mFE, mFD;
    public DcMotor coreHex;
    public Servo servoMotor;

    @Override
    public void init(){
        //Movement motors
        mTE = hardwareMap.get(DcMotor.class, "mTE");
        mTD = hardwareMap.get(DcMotor.class, "mTD");
        mFE = hardwareMap.get(DcMotor.class, "mFE");
        mFD = hardwareMap.get(DcMotor.class, "mFD");

        //Movement motors setDirection
        //sets FORWARD or REVERSE
        mTE.setDirection(DcMotor.Direction.FORWARD);
        mTD.setDirection(DcMotor.Direction.REVERSE);
        mFE.setDirection(DcMotor.Direction.FORWARD);
        mFD.setDirection(DcMotor.Direction.REVERSE);

        //BNO55IMU for precision rotation
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        //parameters for BNO55IMU
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);


        //coreHex motors
        coreHex = hardwareMap.get(DcMotor.class, "cHex");

        //coreHex direction
        //sets FORWARD or REVERSE
        coreHex.setDirection(DcMotor.Direction.FORWARD);


        //Servo motor
        servoMotor = hardwareMap.get(Servo.class, "servo");
    }

    @Override 
    public void loop(){
       move();
       servo();
       cHex();
    }

    public void move(){
        while(opModeIsActive()) {
            double y = -gamepad1.right_trigger; //REVERSED
            double x = gamepad1.left_stick_x * 1.1; //correct the imperfection
            double yMinor = gamepad1.left_trigger;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.cos(botHeading) + y * Math.sin(botHeading);


            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            double frontLeftPower = (y - yMinor + x + rx) / denominator;
            double backLeftPower = (y - yMinor - x + rx) / denominator;
            double frontRightPower = (y - yMinor - x - rx) / denominator;
            double backRightPower = (y - yMinor + x - rx) / denominator;

            double denominatorRot = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPowerRot = (rotY + rotX + rx) / denominatorRot;
            double backLeftPowerRot = (rotY - rotX + rx) / denominatorRot;
            double frontRightPowerRot = (rotY - rotX - rx) / denominatorRot;
            double backRightPowerRot = (rotY + rotX - rx) / denominatorRot;

            mFD.setPower(frontRightPower + frontRightPowerRot);
            mFE.setPower(frontLeftPower + frontLeftPowerRot);
            mTD.setPower(backRightPower + backRightPowerRot);
            mTE.setPower(backLeftPower + backLeftPowerRot);
        }
    }

    public void cHex(){
        while(opModeIsActive()){
            double moveCore = gamepad2.right_trigger;
            double downCore = gamepad2.left_trigger; 

            coreHex.setPower(moveCore);
            coreHex.setPower(downCore);
        }
    }

    public void servo(){
        while(opModeIsActive()){
            boolean open;
            boolean close;

            open = gamepad2.y;
            close = gamepad2.b;

            if(open){
                servo.setPosition(1);
            }
            if(close){
                servo.setDirection(0);
            }
        }
    }
    
}