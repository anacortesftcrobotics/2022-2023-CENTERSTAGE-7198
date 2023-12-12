package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name="AlfalfaAutoBackBluePark", group="Robot")
public class AlfalfaAutoBackBluePark extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    private ElapsedTime runtime = new ElapsedTime();
    boolean toggle;


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    public void mecanumX(double forwards,double sideways, double rotate) {
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        telemetry.addData("Forward", forwards);

        //does math for mechanim chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        frontRight.setPower((forwards - sideways - rotate) / denominator);
        backLeft.setPower((forwards - sideways + rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);
    }

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//previous: (0.75,0,0.25)
        while (opModeIsActive() && !toggle) {
            mecanumX(0.65,0,-0.15);

            new Timer().schedule(new TimerTask()
            {
                @Override
                public void run()
                {
                    toggle = true;
                    mecanumX(0,0,0);                }
            }, 1750 );

        }
        sleep(1000);

    }


        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds


        // Step 2:  Spin right for 1.3 seconds
        /**leftDrive.setPower(TURN_SPEED);
        rightDrive.setPower(-TURN_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backward for 1 Second
        leftDrive.setPower(-FORWARD_SPEED);
        rightDrive.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/

        // Step 4:  Stop

}


