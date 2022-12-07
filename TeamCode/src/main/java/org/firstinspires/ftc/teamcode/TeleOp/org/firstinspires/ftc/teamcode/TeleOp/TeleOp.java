package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp_Master", group = "TeleOp")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * mFD = MOTOR FRENTE DIREITA
         * mFE = MOTOR FRENTE ESQUERDA
         * mTD = MOTOR TRÁS DIREITA
         * mTE = MOTOR TRÁS ESQUERDA
         */

        DcMotor mFD = hardwareMap.dcMotor.get("mFD");
        DcMotor mFE = hardwareMap.dcMotor.get("mFE");
        DcMotor mTD = hardwareMap.dcMotor.get("mTD");
        DcMotor mTE = hardwareMap.dcMotor.get("mTE");

        mFD.setDirection(DcMotorSimple.Direction.REVERSE);
        mTD.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            double y = -gamepad1.right_trigger; //REVERSED
            double x = gamepad1.left_stick_x * 1.1; //correct the imperfection
            double yMinor = gamepad1.left_trigger;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.cos(botHeading) + y * Math.sin(botHeading);


            double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);
            double frontLeftPower = (y + yMinor + x + rx) / denominator;
            double backLeftPower = (y - yMinor - x + rx) / denominator;
            double frontRightPower = (y + yMinor - x - rx) / denominator;
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
}
