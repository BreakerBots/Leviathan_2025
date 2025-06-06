// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.drivers;

import java.util.ArrayList;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.networktables.TimestampedObject;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.physics.BreakerVector3;
import frc.robot.BreakerLib.util.TimestampedValue;
import frc.robot.BreakerLib.util.math.BreakerMath;

public class ZED extends SubsystemBase {
  /** Creates a new ZED. */
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private IntegerSubscriber heartbeatSub;
  private IntegerArraySubscriber idSub;
  private StringArraySubscriber labelSub;
  private IntegerSubscriber latencySub;
  private StructArraySubscriber<Translation3d> translationSub;
  private StructArraySubscriber<Translation3d> boxSub;
  private DoubleArraySubscriber confSub;
  private BooleanArraySubscriber isVisSub;
  private BooleanArraySubscriber isMovingSub;
  private DoubleArraySubscriber customValueSub;

  private StructSubscriber<Pose3d> cameraPoseSub;
  private DoubleSubscriber camPoseLatencySub;
  private DoubleSubscriber camPoseConfSub;

  private long lastHeartbeat = -1;
  private Timer timeSinceLastUpdate;

  private RefrenceFrame cameraRefrenceFrameInRobotSpace;
  private TimeInterpolatableBuffer<Pose3d> robotPoseHistory;
  private Supplier<TimestampedValue<Pose3d>> robotPoseAtTimeSupplier;

  private DetectionResults latestResult;
  private LocalizationResults latestLocalization;


  public ZED(String cameraName, Supplier<TimestampedValue<Pose3d>> robotPoseAtTimeSupplier, Transform3d robotToZedLeftEye) {
    cameraRefrenceFrameInRobotSpace = new RefrenceFrame(robotToZedLeftEye);
    this.robotPoseAtTimeSupplier = robotPoseAtTimeSupplier;
    latestResult = new DetectionResults(new TreeMap<>(), Timer.getTimestamp());
    robotPoseHistory = TimeInterpolatableBuffer.createBuffer(10);
    latestLocalization = new LocalizationResults(new Pose3d(), Timer.getTimestamp(), 0.0, cameraRefrenceFrameInRobotSpace, new RefrenceFrame(getPoseAtTime(Timer.getTimestamp())), null);
    timeSinceLastUpdate = new Timer();
    configNT(cameraName);
  }

  private void configNT(String name) {
    NetworkTable mainTable = inst.getTable(name);
    NetworkTable table = mainTable.getSubTable("detections");
    heartbeatSub = table.getIntegerTopic("heartbeat").subscribe(-1);
    idSub = table.getIntegerArrayTopic("id").subscribe(new long[0]);
    labelSub = table.getStringArrayTopic("label").subscribe(new String[0]);
    latencySub = table.getIntegerTopic("pipeline_latency").subscribe(0);
    translationSub = table.getStructArrayTopic("translation", Translation3d.struct).subscribe(new Translation3d[0]);
    boxSub = table.getStructArrayTopic("box", Translation3d.struct).subscribe(new Translation3d[0]);
    confSub = table.getDoubleArrayTopic("conf").subscribe(new double[0]);
    isVisSub = table.getBooleanArrayTopic("is_visible").subscribe(new boolean[0]);
    isMovingSub = table.getBooleanArrayTopic("is_moving").subscribe(new boolean[0]);
    customValueSub = table.getDoubleArrayTopic("rotation").subscribe(new double[0]);
    NetworkTable camPoseTable = mainTable.getSubTable("pose");
    cameraPoseSub = camPoseTable.getStructTopic("cam_pose", Pose3d.struct).subscribe(new Pose3d());
    camPoseLatencySub = camPoseTable.getDoubleTopic("cam_pose_latency").subscribe(0.0);  
    camPoseConfSub = camPoseTable.getDoubleTopic("cam_pose_conf").subscribe(0.0);  
  }

  public Transform3d getRobotToCameraTransform() {
    return cameraRefrenceFrameInRobotSpace.getParentToFrameTransform();
  }

  public boolean isConnected() {
    return (timeSinceLastUpdate.get() < 0.3) && (lastHeartbeat > -1) ;
  }

  public DetectionResults getDetectionResults() {
    updateRobotPoseHistory();
   
    try {
       // TimestampedInteger[] latencyQueue = latencySub.readQueue();
      // if (latencyQueue.length > 0) {
        TimestampedInteger latency = latencySub.getAtomic(); //latencyQueue[latencyQueue.length - 1];
        double captureTimestamp = (((double)(latency.timestamp)) - (((double)(latency.value)) / 1000.0)) / ((double)(1e6));
        updateDetections(captureTimestamp);
      // }
    } catch (Exception e) {
      System.err.print("ZED: Error reading detections: " + e.getMessage());
    }
      
    return latestResult;
  }

  @Override
  public void periodic() {
    long newHb = heartbeatSub.get();
    if (newHb > lastHeartbeat && newHb != -1) {
      timeSinceLastUpdate.reset();
    }
    updateRobotPoseHistory();
  }
  
  private TimestampedValue<Pose3d> updateRobotPoseHistory() {
    TimestampedValue<Pose3d> curPose = robotPoseAtTimeSupplier.get();
  
    if (robotPoseHistory.getInternalBuffer().isEmpty() || !MathUtil.isNear(curPose.getTimestamp().in(Units.Second), robotPoseHistory.getInternalBuffer().lastKey(), 1e-5)) {
      robotPoseHistory.addSample(curPose.getTimestamp().in(Units.Second), curPose.getValue());
    }
    return curPose;
  }

  public LocalizationResults getLocalizationResults() {
    updateRobotPoseHistory();
    TimestampedObject<Pose3d> atomicPose = cameraPoseSub.getAtomic();
    double captureTimestamp = atomicPose.timestamp + camPoseLatencySub.get();
    double conf = camPoseConfSub.get();
    RefrenceFrame robotFrameInWorldSpace = new RefrenceFrame(getPoseAtTime(captureTimestamp));
    latestLocalization = new LocalizationResults(atomicPose.value, captureTimestamp, conf, cameraRefrenceFrameInRobotSpace, robotFrameInWorldSpace, latestLocalization);
    return latestLocalization;
  }

  public Optional<LocalizationResults> getUnreadLocalizationResults() {
    TimestampedObject<Pose3d> atomicPose = cameraPoseSub.getAtomic();
    double captureTimestamp = atomicPose.timestamp + camPoseLatencySub.get();
    if (MathUtil.isNear(captureTimestamp, latestLocalization.getTimestamp(), 1e-5)) {
      return Optional.empty();
    }
    return Optional.of(getLocalizationResults());
  }

  private Pose3d getPoseAtTime(double timestamp) {
    TimestampedValue<Pose3d> latest = updateRobotPoseHistory();
    Optional<Pose3d> opt = robotPoseHistory.getSample(timestamp);
    return opt.orElse(latest.getValue());
  }
  
  private DetectionResults updateDetections(double timestamp) {
    TreeMap<Long, TrackedObject> prevTrackedObjects = latestResult.getTrackedObjectsMap();

    Pose3d robotPose = getPoseAtTime(timestamp);
    RefrenceFrame robotFrameInWorld = new RefrenceFrame(robotPose);

    long[] ids = idSub.get();
    String[] lables = labelSub.get();
    Translation3d[] translations = translationSub.get();
    Translation3d[] boxes = boxSub.get();
    double[] confs = confSub.get();
    boolean[] isVisArr = isVisSub.get();
    boolean[] isMovArr = isMovingSub.get();
    double[] cvArr = customValueSub.get();

    latestResult.results.clear();

    int[] arrLens = new int[]{ids.length, lables.length, translations.length, boxes.length, confs.length, isVisArr.length, isMovArr.length, cvArr.length};

    int minLen = ids.length;
    for (int len : arrLens) {
      if (len < minLen) {
        minLen = len;
      }
    }
    
    for (int i = 0; i < minLen; i++) {
      long id = ids[i];
      Optional<TrackedObject> prevInstance = Optional.ofNullable(prevTrackedObjects.get(id));
      String lable = lables[i];
      boolean isVis = isVisArr[i];
      boolean isMov = isMovArr[i]; 
      double conf = confs[i];
      double cv = cvArr[i];

      Translation3d transCamSpace = translations[i];
      RawObjectPosition positionRaw = RawObjectPosition.fromCamSpaceTranslation(transCamSpace, cameraRefrenceFrameInRobotSpace, robotFrameInWorld, timestamp);
      
      ObjectVelocity vel = new ObjectVelocity();
      if (prevInstance.isPresent()) {
        vel = new ObjectVelocity(prevInstance.get().position().getRawObjectPosition(), positionRaw, isMov);
      }

      ObjectPosition position = new ObjectPosition(positionRaw, vel);
      ObjectDimensions dimensions = new ObjectDimensions(boxes[i]);


      TrackedObject object = new TrackedObject(id, lable, timestamp, vel, position, dimensions, conf, cv, isVis, isMov);
      latestResult.results.put(id, object);
    }
    return latestResult;
  }

  public static final class ObjectVelocity {
    private BreakerVector3 velocityCameraSpace;
    private BreakerVector3 velocityRobotSpace;
    private BreakerVector3 velocityFieldSpace;

    public ObjectVelocity() {
      velocityCameraSpace = new BreakerVector3();
      velocityRobotSpace = new BreakerVector3();
      velocityFieldSpace = new BreakerVector3();
    }

    private ObjectVelocity(RawObjectPosition previousPose, RawObjectPosition currentPose, boolean isMoveing) {
      double dt = currentPose.timestamp() - previousPose.timestamp();
      velocityCameraSpace = new BreakerVector3(currentPose.translationCameraSpace.minus(previousPose.translationCameraSpace)).div(dt);
      velocityRobotSpace = new BreakerVector3(currentPose.translationRobotSpace.minus(previousPose.translationRobotSpace)).div(dt);
      if (isMoveing) {
        velocityFieldSpace = new BreakerVector3(currentPose.translationFieldSpace.minus(previousPose.translationFieldSpace)).div(dt);
      } else {
        velocityFieldSpace = new BreakerVector3();
      }
    }

    public BreakerVector3 getVelocityCameraSpace() {
        return velocityCameraSpace;
    }

    public BreakerVector3 getVelocityFieldSpace() {
        return velocityFieldSpace;
    }

    public BreakerVector3 getVelocityRobotSpace() {
        return velocityRobotSpace;
    }
    
  }

  public static final class ObjectPosition {
    private RawObjectPosition position;
    private ObjectVelocity velocity;
    private ObjectPosition(RawObjectPosition currentPose, ObjectVelocity velocity) {
      position = currentPose;
      this.velocity = velocity;
    }

    public Translation3d getPositionCameraSpace(boolean compensateForLatency) {
      var pos = position.translationCameraSpace();
      if (compensateForLatency) {
        pos = pos.plus(velocity.getVelocityCameraSpace().times(Timer.getTimestamp() - position.timestamp()).getAsTranslation());
      }
      return pos;
    }

    public Translation3d getPositionRobotSpace(boolean compensateForLatency) {
      var pos = position.translationRobotSpace();
      if (compensateForLatency) {
        pos = pos.plus(velocity.getVelocityRobotSpace().times(Timer.getTimestamp() - position.timestamp()).getAsTranslation());
      }
      return pos;
    }

    public Translation3d getPositionFieldSpace(boolean compensateForLatency) {
      var pos = position.translationFieldSpace();
      if (compensateForLatency) {
        pos = pos.plus(velocity.getVelocityFieldSpace().times(Timer.getTimestamp() - position.timestamp()).getAsTranslation());
      }
      return pos;
    }

    RawObjectPosition getRawObjectPosition() {
      return position;
    }
  }

  
  public static final record ObjectDimensions(double width, double height, double length) {
    public ObjectDimensions(Translation3d translationRepresentation) {
      this(translationRepresentation.getX(), translationRepresentation.getZ(), translationRepresentation.getY());
    }
  }
  

  public static final record RawObjectPosition(Translation3d translationCameraSpace, Translation3d translationRobotSpace, Translation3d translationFieldSpace, double timestamp) {
    public RawObjectPosition() {
      this(new Translation3d(), new Translation3d(), new Translation3d(), 0.0);
    }
    public static RawObjectPosition fromCamSpaceTranslation(Translation3d translationCameraSpace, RefrenceFrame cameraRefrenceFrameInRobotSpace, RefrenceFrame robotRefrenceFrameInFieldSpace, double timestamp) {
      Translation3d robotSpaceTrans = cameraRefrenceFrameInRobotSpace.convertToParentFrame(translationCameraSpace);
      Translation3d fieldSpaceTrans = robotRefrenceFrameInFieldSpace.convertToParentFrame(robotSpaceTrans);
      return new RawObjectPosition(translationCameraSpace, robotSpaceTrans, fieldSpaceTrans, timestamp);
    }
  }

  private static class RefrenceFrame {
    private Transform3d parentToFrameTransfrom;
    // private CoordinateSystem coordinateSystem;
    public RefrenceFrame(Transform3d parentToFrameTransfrom) {
      this.parentToFrameTransfrom = parentToFrameTransfrom;
      // coordinateSystem = BreakerMath.getCoordinateSystemFromRotation(parentToFrameTransfrom.getRotation());
    }

    public RefrenceFrame(Pose3d frameOriginInParentSpace) {
      this(frameOriginInParentSpace.minus(Pose3d.kZero));
    }

    public Translation3d convertToParentFrame(Translation3d val) {
      //  val = CoordinateSystem.convert(val, coordinateSystem, CoordinateSystem.NWU());
      val.rotateBy(parentToFrameTransfrom.getRotation().unaryMinus());
      val = val.minus(parentToFrameTransfrom.getTranslation());
      return val;
    }

    public Rotation3d convertToParentFrame(Rotation3d val) {
      val.rotateBy(parentToFrameTransfrom.getRotation().unaryMinus());
      return val;
    }


    public Pose3d convertToParentFrame(Pose3d val) {
      // val = CoordinateSystem.convert(val, coordinateSystem, CoordinateSystem.NWU());
      return new Pose3d(convertToParentFrame(val.getTranslation()), convertToParentFrame(val.getRotation()));
    }


    // public CoordinateSystem getCoordinateSystem() {
    //     return coordinateSystem;
    // }

    public Transform3d getParentToFrameTransform() {
        return parentToFrameTransfrom;
    }
    
  }

  public static record TrackedObject(
      long objectID, 
      String label,
      double timestamp,
      ObjectVelocity velocity,
      ObjectPosition position,
      ObjectDimensions cameraRelitiveDimensions,
      double confidance,
      double customValue,
      boolean isVisible,
      boolean isMoveing) implements Comparable<TrackedObject> {

    @Override
    public int compareTo(TrackedObject otherTracked) {
        return (int) (MathUtil.clamp(otherTracked.objectID - objectID, (long) Integer.MIN_VALUE, (long) Integer.MAX_VALUE));
    }

    @Override
    public boolean equals(Object object) {
      if (object instanceof TrackedObject) {
        var otherTracked = (TrackedObject) object;
        return otherTracked.objectID == objectID;
      }
      return false;
    }
    
  } 

  public class DetectionResults {
    private TreeMap<Long, TrackedObject> results;
    private double timestamp;
    private DetectionResults(TreeMap<Long, TrackedObject> results, double timestamp) {
      this.results = results;
      this.timestamp = timestamp;
    }

    public ArrayList<TrackedObject> getTrackedObjects() {
      return new ArrayList<>(results.values());
    }
  
    public TreeMap<Long, TrackedObject> getTrackedObjectsMap() {
      return new TreeMap<>(results);
    }

    public double getTimestamp() {
      return timestamp;
    }

  }

  public class LocalizationResults {
    private Pose3d cameraPose;
    private double timestamp;
    private double confidance;
    private RefrenceFrame cameraFrameInRobotSpace;
    private RefrenceFrame robotFrameInWorldSpace;
    private LocalizationResults prevResults;
    private LocalizationResults(Pose3d cameraPose, double timestamp, double confidance, RefrenceFrame cameraFrameInRobotSpace, RefrenceFrame robotFrameInWorldSpace, LocalizationResults prevResults) {
      this.cameraPose = cameraPose;
      this.timestamp = timestamp;
      this.confidance = confidance;
      this.cameraFrameInRobotSpace = cameraFrameInRobotSpace;
      this.robotFrameInWorldSpace = robotFrameInWorldSpace;
      this.prevResults = prevResults;
    }

    public Pose3d getPoseRobotSpace() {
      return cameraFrameInRobotSpace.convertToParentFrame(cameraPose);
    }

    public Pose3d getPoseFieldSpace() {
      return robotFrameInWorldSpace.convertToParentFrame(getPoseRobotSpace());
    }

    public OdometryRecord getRobotRelativeDelta() {
      Twist3d delta = prevResults.getPoseRobotSpace().log(getPoseRobotSpace());
      return new OdometryRecord(delta, prevResults.timestamp, timestamp);
    }

    public OdometryRecord getFieldRelativeDelta() {
      Twist3d delta = prevResults.getPoseFieldSpace().log(getPoseFieldSpace());
      return new OdometryRecord(delta, prevResults.timestamp, timestamp);
    }

    public double getConfidance() {
      return confidance;
    }

    public double getTimestamp() {
      return timestamp;
    }

    public record OdometryRecord(
      Twist3d delta,
      double startTime,
      double endTime
    ) {
    }

  }
}