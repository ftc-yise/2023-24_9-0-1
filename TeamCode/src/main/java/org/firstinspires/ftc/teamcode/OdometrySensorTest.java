package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class OdometrySensorTest extends LinearOpMode
{

    public double pi = Math.PI;

    public double piOverTwo = Math.PI/2;

    private double AngleA = pi;
    //private double DeadzoneA = 0.12; original value of dead zone
    private double DeadzoneA = .06;

    public static double SIDE_LENGTH = 16;

    DcMotorEx motorFL;
    DcMotorEx motorFR;
    DcMotorEx motorBL;
    DcMotorEx motorBR;

    SparkFunOTOS myOdometrySensor;

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle)
    {
        for (int i = 0; i < xPoints.length; i++)
        {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motorFL = hardwareMap.get(DcMotorEx.class, "LeftFrontDrive");
        motorFR = hardwareMap.get(DcMotorEx.class, "RightFrontDrive");
        motorBL = hardwareMap.get(DcMotorEx.class, "LeftBackDrive");
        motorBR = hardwareMap.get(DcMotorEx.class, "RightBackDrive");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

//        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myOdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sfe_otos");
        SparkFunOTOS.otos_pose2d_t offsetPose = myOdometrySensor.new otos_pose2d_t(3.7, -.55, 0);
        myOdometrySensor.setOffset(offsetPose);
        myOdometrySensor.setAngularScalar(0.994);
        myOdometrySensor.setLinearScalar(0.990);

        waitForStart();

        while (opModeIsActive())
        {
            SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();
            double theta = pose.h + Math.PI;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value

            double horizontal =  gamepad1.left_stick_x;
            double vertical     =  gamepad1.left_stick_y * -1;
            double turnR = 0;
            double turnL = 0;


            /*if (gamepad1.b && theta % (Math.PI/2) > 0.01) {
                turn = (theta % (Math.PI/2)) * 2;
            } else if (!gamepad1.b) {
                turn = gamepad1.right_stick_x;
            }*/

            if (gamepad1.dpad_up && theta > AngleA + DeadzoneA) {
                turnR = (((theta - AngleA) - DeadzoneA) / -piOverTwo);
                //needs to end in positive value
                //turnR = 1;
            }else if (gamepad1.dpad_up && theta < AngleA - DeadzoneA) {
                //CURRENT VERSION of the auto lock code
                turnL = (((theta - AngleA) - DeadzoneA) / piOverTwo);
                //needs to end in negative value
                //turn = .65;
            } else if (!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.dpad_left) {
                turnR = 0;
                turnL = gamepad1.right_stick_x;
            }


            double horizontalOut = (horizontal * Math.cos(theta)) - (vertical * Math.sin(theta));
            double verticalOut = (vertical * Math.cos(theta)) + (horizontal * Math.sin(theta));


            double powerFL  = (verticalOut - horizontalOut + turnR - turnL) * 0.5;
            double powerFR = (verticalOut + horizontalOut - turnR + turnL) * 0.5;
            double powerBL   = (verticalOut + horizontalOut + turnR - turnL) * 0.5;
            double powerBR  = (verticalOut - horizontalOut - turnR + turnL) * 0.5;



            if (gamepad1.left_bumper)
            {
                powerFL /= 4;
                powerFR /= 4;
                powerBL /= 4;
                powerBR /= 4;
            }

            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);

            telemetry.addData("X (inch)", pose.x);
            telemetry.addData("Y (inch)", pose.y);
            telemetry.addData("H (Deg)", pose.h);

            telemetry.addData("Heading", "%.2f Radians", theta);

            telemetry.addData("Horizontal input", gamepad1.left_stick_x * -1);
            telemetry.addData("Vertical input: ", gamepad1.left_stick_y * -1);
            telemetry.addData("Turn input: ", gamepad1.right_stick_x);
            telemetry.addData("Hertical out: ", verticalOut);
            telemetry.addData("Vorizontal out: ", horizontalOut);
            telemetry.update();

            double bx = pose.x;
            double by = pose.y;
            double l = SIDE_LENGTH / 2;

            double[] bxPoints = {l, -l, -l, l};
            double[] byPoints = {l, l, -l, -l};
            double[] axPoints = {0, 0, 0, l / 2, 0, -l / 2};
            double[] ayPoints = {-l, l, l, l / 2, l, l / 2};
            rotatePoints(bxPoints, byPoints, pose.h * Math.PI / 180);
            rotatePoints(axPoints, ayPoints, pose.h * Math.PI / 180);
            for (int i = 0; i < 4; i++)
            {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }
            for (int i = 0; i < 6; i++)
            {
                axPoints[i] += bx;
                ayPoints[i] += by;
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    .setStrokeWidth(1)
                    .setStroke("goldenrod")
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints)
                    .strokeLine(axPoints[0], ayPoints[0], axPoints[1], ayPoints[1])
                    .strokeLine(axPoints[2], ayPoints[2], axPoints[3], ayPoints[3])
                    .strokeLine(axPoints[4], ayPoints[4], axPoints[5], ayPoints[5]);
            dashboard.sendTelemetryPacket(packet);

            if (gamepad1.x)
            {
                myOdometrySensor.calibrateImu();
            }

            if (gamepad1.y)
            {
                myOdometrySensor.resetTracking();
//                SparkFunOTOS.otos_pose2d_t newPose = myOdometrySensor.new otos_pose2d_t(5, 10, 45);
//                myOdometrySensor.setPosition(newPose);
            }

            sleep(20);
        }
    }
}
