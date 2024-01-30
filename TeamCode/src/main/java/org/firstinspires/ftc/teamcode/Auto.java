package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.yise.LiftArm;
import org.firstinspires.ftc.teamcode.yise.PoseStorage;
import org.firstinspires.ftc.teamcode.yise.TensorflowVision;

import org.firstinspires.ftc.teamcode.yise.Parameters;


@Autonomous(name="Autonomous", group="Linear Opmode")
public class Auto extends LinearOpMode {
    //Initialize timer
    private ElapsedTime runtime = new ElapsedTime();
    int prop;

    double startX = 0;
    double startZ = 0;
    double startY = 0;

    public enum Prop {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static Prop location;

    public Prop propDetection(TensorflowVision vision) {
        prop = vision.getPropPosition();
        if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            if (prop == 2) {
                location = Prop.RIGHT;
            } else if (prop == 0) {
                location = Prop.MIDDLE;
            } else if (prop == 1) {
                location = Prop.LEFT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            if (prop == 0) {
                location = Prop.RIGHT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 2) {
                location = Prop.LEFT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            if (prop == 2) {
                location = Prop.LEFT;
            } else if (prop == 1) {
                location = Prop.MIDDLE;
            } else if (prop == 0) {
                location = Prop.RIGHT;
            }
        } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            if (prop == 1) {
                location = Prop.LEFT;
            } else if (prop == 0) {
                location = Prop.MIDDLE;
            } else if (prop == 2) {
                location = Prop.RIGHT;
            }
        }

        return location;
    }

    public TrajectorySequence propPlacment(SampleMecanumDrive drive, Pose2d startPose, LiftArm arm) {
        double z_coordinate = 0.0;
        double x_coordinate = 0.0;
        double y_coordinate = 0.0;

                if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR){
                    z_coordinate = -90;
                    if (location == Prop.RIGHT) {
                        y_coordinate = -36.0;
                        x_coordinate = 19.5;
                    } else if (location == Prop.MIDDLE) {
                        y_coordinate = -32.5;
                        x_coordinate = 12.0;
                    } else if (location == Prop.LEFT) {
                        y_coordinate = -35.0;
                        z_coordinate = 0;
                        x_coordinate = 7.5;
                    }
                } else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR){
                    z_coordinate = -90;
                    if (location == Prop.RIGHT) {
                        y_coordinate = -30.0;
                        z_coordinate = 180;
                        x_coordinate = -33.5;
                    } else if (location == Prop.MIDDLE) {
                        y_coordinate = -32.5;
                        x_coordinate = -36.0;
                    } else if (location == Prop.LEFT) {
                        y_coordinate = -36.0;
                        x_coordinate = -49.5;
                    }
                }   else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR){
                    z_coordinate = 90;
                    if (location == Prop.LEFT) {
                        y_coordinate = 36.0;
                        x_coordinate = 25.5;
                    } else if (location == Prop.MIDDLE) {
                        y_coordinate = 32.5;
                        x_coordinate = 12.0;
                    } else if (location == Prop.RIGHT) {
                        y_coordinate = 32.0;
                        z_coordinate = 0;
                        x_coordinate = 9;
                    }
                } else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR){
                    z_coordinate = 90;
                    if (location == Prop.LEFT) {
                        y_coordinate = 34;
                        z_coordinate = 180;
                        x_coordinate = -32.5;
                    } else if (location == Prop.MIDDLE) {
                        y_coordinate = 32.5;
                        x_coordinate = -36.0;
                    } else if (location == Prop.RIGHT) {
                        y_coordinate = 38.0;
                        x_coordinate = -43.5;
                    }
                }

        TrajectorySequence mySequence = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(Parameters.wait)
                .lineToLinearHeading(new Pose2d(x_coordinate, y_coordinate, Math.toRadians(z_coordinate)))
                //drop purple pixel here
                .waitSeconds(1)
                .addDisplacementMarker(20, arm::DropPurplePixel)
                .waitSeconds(1)
                .build();

        return mySequence;
    }

    public TrajectorySequence yellowPixle(SampleMecanumDrive drive, Pose2d startPose, LiftArm arm) {
        double x_coordinate = 47.0;
        double y_coordinate = 0.0;
        double z_coordinate = 180.0;

        double white_x = 0.0;
        double white_y = 0.0;
        double white_z = 0.0;

        double in_x = 0.0;
        double in_y = 0.0;
        double in_z = 180.0;

        double strafe = 0;

        if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            if (location == Prop.RIGHT) {
                y_coordinate = -42.0;
            } else if (location == Prop.MIDDLE) {
                y_coordinate = -35.0;
            } else if (location == Prop.LEFT) {
                y_coordinate = -29.0;
            }


            white_x = -55.0;
            white_y = -36.0;
            white_z = 180.0;

            in_x = 18;
            in_y = -8;

            strafe = -24;
        }
            else if (Parameters.allianceColor == Parameters.Color.RED && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
                if (location == Prop.LEFT) {
                    y_coordinate = -25.5;
                } else if (location == Prop.MIDDLE) {
                    y_coordinate = -31;
                } else if (location == Prop.RIGHT) {
                    y_coordinate = -39;
                }


            white_x = -55.0;
            white_y = -36.0;
            white_z = 180.0;

            in_x = 18;
            in_y = -8;

            strafe = -24;
            }
            else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
                    if (location == Prop.LEFT) {
                        y_coordinate = 42.0;
                    } else if (location == Prop.MIDDLE) {
                        y_coordinate = 35.0;
                    } else if (location == Prop.RIGHT) {
                        y_coordinate = 29.0;
                    }

            white_x = -55.0;
            white_y = 36.0;
            white_z = 180.0;

            in_x = 18;
            in_y = 8;

            strafe = 24;

            }
            else if (Parameters.allianceColor == Parameters.Color.BLUE && Parameters.autoConfig == Parameters.AutonomousConfig.EXTERIOR) {
            if (location == Prop.RIGHT) {
                y_coordinate = 20;
            } else if (location == Prop.MIDDLE) {
                y_coordinate = 27;
            } else if (location == Prop.LEFT) {
                y_coordinate = 35;
            }

            white_x = -55.0;
            white_y = 36.0;
            white_z = 180.0;

            in_x = 18;
            in_y = 8;

            strafe = 24;

            }
        TrajectorySequence backdrop = drive.trajectorySequenceBuilder(startPose)
                .back(-12)
                .lineToLinearHeading(new Pose2d(x_coordinate, y_coordinate, Math.toRadians(z_coordinate)))
                .addDisplacementMarker(20, () -> {
                    arm.extend(LiftArm.Distance.AUTO);
                })
                .forward(-8)
                .build();

        TrajectorySequence whitestack = drive.trajectorySequenceBuilder(startPose)
                .back(-16)
                .lineToLinearHeading(new Pose2d(white_x, white_y, Math.toRadians(white_z)))
                .forward(5)
                .waitSeconds(2) // insert intake here
                .back(3)
                .strafeLeft(strafe)
                .lineToLinearHeading(new Pose2d(in_x, in_y, Math.toRadians(in_z)))
                .lineToLinearHeading(new Pose2d(x_coordinate, y_coordinate, Math.toRadians(z_coordinate)))
                .addDisplacementMarker(100, () -> {
                    arm.extend(LiftArm.Distance.AUTO);
                })
                .forward(-4)
                .build();
        if (Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR) {
            return backdrop;
        } else {
            return whitestack;
        }
    }

    public TrajectorySequence parking(SampleMecanumDrive drive, Pose2d startPose, LiftArm arm) {
        double x_coordinate = 48.0;
        double y_coordinate = 0.0;
        double z_coordinate = 180.0;

        if (Parameters.allianceColor == Parameters.Color.RED) {
            if (Parameters.endingPosition == Parameters.EndingPosition.LEFT) {
                y_coordinate = -11;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.CENTER) {
                y_coordinate = -35.0;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.RIGHT) {
                y_coordinate = -50;
            }
        } else {
            if (Parameters.endingPosition == Parameters.EndingPosition.LEFT) {
                y_coordinate = 50;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.CENTER) {
                y_coordinate = 35.0;
            } else if (Parameters.endingPosition == Parameters.EndingPosition.RIGHT) {
                y_coordinate = 11;
            }
        }


        TrajectorySequence park = drive.trajectorySequenceBuilder(startPose)
                .back(-5)
                .lineToLinearHeading(new Pose2d(x_coordinate, y_coordinate, Math.toRadians(z_coordinate)))
                .build();
        return park;
    }


    @Override
    public void runOpMode() {

        //Initialize RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        LiftArm arm = new LiftArm(hardwareMap);

        TensorflowVision vision = new TensorflowVision(hardwareMap);

        Parameters parameters = new Parameters();

        //Sense cones
        while (!isStarted()) {
            propDetection(vision);

            telemetry.addData("Prop: ", location);

            telemetry.addData("side", Parameters.autoConfig);
            telemetry.addData("color", Parameters.allianceColor);
            telemetry.addData("park", Parameters.endingPosition);
            telemetry.addData("wait", Parameters.wait);

            telemetry.update();
        }

        if (isStopRequested()) return;


        //Bot starting position
        if (Parameters.allianceColor == Parameters.Color.BLUE){
            startY = 62;
            startZ = 90;
        } else {
            startY = -62;
            startZ = -90;
        }

        if (Parameters.autoConfig == Parameters.AutonomousConfig.INTERIOR){
            startX = 15.4375;
        } else {
            startX = -39.4375;
        }

        telemetry.addData("color:", Parameters.allianceColor);
        telemetry.addData("confiq:", Parameters.autoConfig);
        telemetry.addData("park:", Parameters.endingPosition);

        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startZ));
        drive.setPoseEstimate(startPose);

        //Trajectory sequences contain driving instructions

        //TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(startPose)
        //        .lineToLinearHeading(new Pose2d(-33, 44, Math.toRadians(130)))
        //        .build();
        TrajectorySequence sequence1 = propPlacment(drive, startPose, arm);

        TrajectorySequence sequence2 = yellowPixle(drive, sequence1.end(), arm);

        TrajectorySequence park = parking(drive, sequence2.end(), arm);

        //Follow trajectories in order
        drive.followTrajectorySequence(sequence1);
        drive.followTrajectorySequence(sequence2);
        sleep(1000);
        arm.openTrapdoor();
        sleep(1500);
        arm.retract();
        arm.closeTrapdoor();
        sleep(2000);
        drive.followTrajectorySequence(park);
        PoseStorage.currentPose = drive.getPoseEstimate();
    }
}
