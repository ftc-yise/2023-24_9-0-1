package org.firstinspires.ftc.teamcode.LaserRoadrunner;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

//unused but may potentially have a use


/**
 * This class provides an abstraction layer for the SparkFun OTOS odometry sensor.
 * It simplifies interaction with the sensor by handling initialization,
 * calibration, and data retrieval.
 */
public class RRAbstarctionLayer {
    // Reference to the SparkFunOTOS sensor object
    SparkFunOTOS myOdometrySensor;

    // Reference to the SampleMecanumDrive object
    SampleMecanumDrive drive;

    /**
     * Constructor for the RRAbstarctionLayer class.
     * Initializes the odometry sensor and performs necessary configuration.
     *
     * @param hardwareMap The hardware map used to access robot hardware
     * @param drive
     */

    //Declare the constructor for the class
    public RRAbstarctionLayer(HardwareMap hardwareMap, SampleMecanumDrive drive) {

        this.drive = drive; // Use the drive object passed as a parameter

        // Get the SparkFunOTOS sensor from the hardware map
        myOdometrySensor = hardwareMap.get(SparkFunOTOS.class, "sfe_otos");

        // Set the offset location of the sensor relative to the robot center
        SparkFunOTOS.otos_pose2d_t offsetPose = myOdometrySensor.new otos_pose2d_t(3.7, -.55, 0);
        myOdometrySensor.setOffset(offsetPose);

        // Set scalar correction factors for translation and rotation
        myOdometrySensor.setLinearScalar(1.0);
        myOdometrySensor.setAngularScalar(1.0);

        // Calibrate IMU. Must be stationary and flat during this time! By default, this will take
        // about 612ms, but can be made faster by taking fewer samples
        myOdometrySensor.calibrateImu();

        // Reset tracking algorithm on OTOS
        myOdometrySensor.resetTracking();

        // Set known location on field
        SparkFunOTOS.otos_pose2d_t newPose = myOdometrySensor.new otos_pose2d_t(5, 10, 45);
        myOdometrySensor.setPosition(newPose);

        /**
         * Updates the internal pose estimate based on the latest sensor data.
         * This method might not be used in all use cases.
         */
    }

    public void PostionUpdate() {
        // Get the current pose data from the sensor
        SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();

        // Convert the pose data to a RoadRunner Pose2d object
        Pose2d currentpose = new Pose2d(pose.x, pose.y, Math.toRadians(pose.h));
        drive.setPoseEstimate(currentpose);
    }

    /**
     * Getter method to retrieve the X,Y,Z coordinate (position) from the sensor data.
     *
     * @return The X,Y,Z coordinate (inches/degrees) from the odometry sensor
     */    public double getX() {
        SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();
        return pose.x;
    }

    public double getY() {
        SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();
        return pose.y;
    }

    public double getZ() { // Assuming z represents heading (angle)
        SparkFunOTOS.otos_pose2d_t pose = myOdometrySensor.getPosition();
        return Math.toRadians(pose.h); // Convert to radians if needed
    }

    // Update pose estimate (if applicable)
    public void updatePoseEstimate() {
        Pose2d currentPose = new Pose2d(getX(), getY(), getZ());
        drive.setPoseEstimate(currentPose);
    }

    public Pose2d getUpdatedPOSE() {
        updatePoseEstimate();

        return drive.getPoseEstimate();
    }

    public void calibrateImu() {
        myOdometrySensor.calibrateImu();
    }

    public void resetTracking() {
        myOdometrySensor.resetTracking();
        // SparkFunOTOS.otos_pose2d_t newPose = myOdometrySensor.new otos_pose2d_t(5, 10, 45);
    }

}
