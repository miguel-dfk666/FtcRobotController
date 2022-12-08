
package org.firstinspires.ftc.teamcode.TeleOp.org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "hex", group = "LinearOpMode")
@Disabled 
public class coreHex_TeleOp extends LinearOpMode{
    public DcMotor coreHex = null;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode(){
        coreHex = hardwareMap.get(DcMotor.class, "cHex");

        coreHex.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
    }

    public void climbOpMode(){
        double coreHexPower;

        double up = gamepad.right_trigger;
        double down = gamepad2.left_trigger;

        coreHexPower = Range.clip(up + down, -1.0, 1.0);

        coreHex.setPower(coreHexPower);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "coreHex(%.2f)", coreHexPower);
        telemetry.update();
    }
}
