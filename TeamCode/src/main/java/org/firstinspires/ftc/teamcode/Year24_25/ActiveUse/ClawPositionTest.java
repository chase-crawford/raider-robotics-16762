package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// manually input, do not delete pls - CC


@TeleOp(name="Claw Position Testing")

public class ClawPositionTest extends OpMode{
    //Create object for InitVars and UniversalSubmethods classes
    InitVars IV = new InitVars();

    // Create object for AutoMethods class
    ManualMethods MM;

    // Create claw position var
    double clawPos = 0;
    double clawDiff = .05;

    // var for storing last gamepad status
    private Gamepad prevPad = new Gamepad();


    @Override
    public void init(){
        MM = new ManualMethods(hardwareMap, telemetry);
        MM.init();

        // FUN FACT! You can change telemetry font :)
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        // make sure gamepad has something to show so there are
        // no nullRefErrors on startup.
        prevPad.copy(gamepad1);
        MM.raiseElbow();

    }

    @Override
    public void loop()
    {
        //Get input for sElbow0 and sClaw1 -CS
        if (gamepad1.right_bumper && !prevPad.right_bumper){
            clawPos = Math.min(clawPos+clawDiff, 1);
            MM.setClawPos(clawPos);
        }

        if (gamepad1.left_bumper && !prevPad.left_bumper){
            clawPos = Math.max(clawPos-clawDiff, 0);
            MM.setClawPos(clawPos);
        }
        telemetry.addData("Claw Position", MM.getClawPos());

        // set current gamepad as prev state for next iteration
        prevPad.copy(gamepad1);
    }

}


