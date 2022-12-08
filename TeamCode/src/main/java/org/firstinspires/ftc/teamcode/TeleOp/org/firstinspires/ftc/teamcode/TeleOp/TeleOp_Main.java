package org.firstinspires.ftc.teamcode.TeleOp.org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

public class TeleOp_Main extends OpMode{
    public DcMotor mTE, mTD, mFE, mFD;
    public DcMotor coreHex;
    public Servo servoMotor;
    public BNO055IMU imu;

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

//looping function
    @Override 
    public void loop(){
       move(); //move function
       servo(); //servo function
       cHex(); //coreHex function
    }

    //move function
    public void move(){
        //Movement of robot
            double y = -gamepad1.right_trigger; //REVERSED
            double x = gamepad1.left_stick_x * 1.1; //correct the imperfection
            double yMinor = gamepad1.left_trigger; //backwards
            double rx = gamepad1.right_stick_x; //roundX

        //imu variable for rotation in angles
            double botHeading = -imu.getAngularOrientation().firstAngle;

            //rotation in X and Y
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.cos(botHeading) + y * Math.sin(botHeading);


            //Movementation with math
            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            double frontLeftPower = (y - yMinor + x + rx) / denominator;
            double backLeftPower = (y - yMinor - x + rx) / denominator;
            double frontRightPower = (y - yMinor - x - rx) / denominator;
            double backRightPower = (y - yMinor + x - rx) / denominator;

            //Movement rot with math
            double denominatorRot = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPowerRot = (rotY + rotX + rx) / denominatorRot;
            double backLeftPowerRot = (rotY - rotX + rx) / denominatorRot;
            double frontRightPowerRot = (rotY - rotX - rx) / denominatorRot;
            double backRightPowerRot = (rotY + rotX - rx) / denominatorRot;

            //set power
            mFD.setPower(frontRightPower + frontRightPowerRot);
            mFE.setPower(frontLeftPower + frontLeftPowerRot);
            mTD.setPower(backRightPower + backRightPowerRot);
            mTE.setPower(backLeftPower + backLeftPowerRot);

    }

    //coreHex movement
    public void cHex(){
        //high definition of power with coreHex
            double moveCore = gamepad2.right_trigger;
            double downCore = gamepad2.left_trigger;

            //set power cHex
            coreHex.setPower(moveCore);
            coreHex.setPower(downCore);

            //setting power definition for adjust
        double powerCore1 = 1;
        double powerCore2 = 0.5;

        //up and down adjust. If nothing is pressed still still stop
        if (gamepad2.dpad_up){
            coreHex.setPower(powerCore1); //1
        } else if(gamepad2.dpad_down){
            coreHex.setPower(powerCore2); //0.5
        } else {
            coreHex.setPower(0); //no movement
        }

    }

    //Servo motor movement
    public void servo(){
        //close and up function for servoMotor
            boolean open;
            boolean close;
            //close and open definition
            open = gamepad2.y;
            close = gamepad2.b;

            //position for servoMotor
            if(open){
                servoMotor.setPosition(1); //servo motor is open
            }
            if(close){
                servoMotor.setPosition(0); //servo motor is closed
            }

    }
}