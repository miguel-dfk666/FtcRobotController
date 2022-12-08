package org.firstinspires.ftc.teamcode.TeleOp.org.firstinspires.ftc.teamcode.TeleOp;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name = "TeleOp_20858_Main", group = "TeleOp")
public class TeleOp_Optmized extends OpMode{
    motors_TeleOp motor = new motors_TeleOp();
    coreHex_TeleOp cHex = new coreHex_TeleOp();

    @Override
    public void init(){
      motor.runOpMode();
      cHex.runOpMode();

    }

    @Override
    public void loop(){
        move();
        coreHex();
    }

    public void move(){ motor.climbOpMode(); }

    public void coreHex(){ cHex.climbOpMode(); }
}
