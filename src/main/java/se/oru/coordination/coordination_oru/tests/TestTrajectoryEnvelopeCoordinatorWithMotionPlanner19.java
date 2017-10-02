package se.oru.coordination.coordination_oru.tests;

import java.io.File;
import java.util.Comparator;

import org.metacsp.multi.spatioTemporal.paths.Pose;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.ConstantAccelerationForwardModel;
import se.oru.coordination.coordination_oru.CriticalSection;
import se.oru.coordination.coordination_oru.Mission;
import se.oru.coordination.coordination_oru.RobotAtCriticalSection;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.demo.DemoDescription;
import se.oru.coordination.coordination_oru.motionplanning.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.simulation2D.TrajectoryEnvelopeCoordinatorSimulation;
import se.oru.coordination.coordination_oru.util.Missions;

@DemoDescription(desc = "Example showing hazard of deadlock breaking in combination with yielding-for-parking policy.")
public class TestTrajectoryEnvelopeCoordinatorWithMotionPlanner19 {
	
	public static void main(String[] args) throws InterruptedException {
		
		double MAX_ACCEL = 3.0;
		double MAX_VEL = 14.0;
		//Instantiate a trajectory envelope coordinator.
		//The TrajectoryEnvelopeCoordinatorSimulation implementation provides
		// -- the factory method getNewTracker() which returns a trajectory envelope tracker
		// -- the getCurrentTimeInMillis() method, which is used by the coordinator to keep time
		//You still need to add one or more comparators to determine robot orderings thru critical sections (comparators are evaluated in the order in which they are added)
		final TrajectoryEnvelopeCoordinatorSimulation tec = new TrajectoryEnvelopeCoordinatorSimulation(MAX_VEL,MAX_ACCEL);
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				CriticalSection cs = o1.getCriticalSection();
				RobotReport robotReport1 = o1.getTrajectoryEnvelopeTracker().getRobotReport();
				RobotReport robotReport2 = o2.getTrajectoryEnvelopeTracker().getRobotReport();
				return ((cs.getTe1Start()-robotReport1.getPathIndex())-(cs.getTe2Start()-robotReport2.getPathIndex()));
			}
		});
		tec.addComparator(new Comparator<RobotAtCriticalSection> () {
			@Override
			public int compare(RobotAtCriticalSection o1, RobotAtCriticalSection o2) {
				return (o2.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID()-o1.getTrajectoryEnvelopeTracker().getTrajectoryEnvelope().getRobotID());
			}
		});

		//You probably also want to provide a non-trivial forward model
		//(the default assumes that robots can always stop)
		tec.setForwardModel(1, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		tec.setForwardModel(2, new ConstantAccelerationForwardModel(MAX_ACCEL, MAX_VEL, tec.getControlPeriod(), tec.getTemporalResolution()));
		
		Coordinate footprint1 = new Coordinate(-1.0,0.5);
		Coordinate footprint2 = new Coordinate(1.0,0.5);
		Coordinate footprint3 = new Coordinate(1.0,-0.5);
		Coordinate footprint4 = new Coordinate(-1.0,-0.5);
		tec.setDefaultFootprint(footprint1, footprint2, footprint3, footprint4);

		//Need to setup infrastructure that maintains the representation
		tec.setupSolver(0, 100000000);
		
		//Setup a simple GUI (null means empty map, otherwise provide yaml file)
		String yamlFile = "maps/map-empty.yaml";
		tec.setupGUI(yamlFile);
		//tec.setupGUI(null);
		
		tec.setUseInternalCriticalPoints(false);
		//tec.setBreakDeadlocks(false);
		tec.setYieldIfParking(false);
		
		//MetaCSPLogging.setLevel(tec.getClass().getSuperclass(), Level.FINEST);

		Pose startRobot1 = new Pose(5.0,5.0,0.0);
		Pose finalRobot1 = new Pose(25.0,5.0,0.0);

		Pose startRobot21 = new Pose(5.0,15.0,-Math.PI/2.0);
		Pose finalRobot21 = new Pose(10.0,5.0,0.0);
		
		Pose startRobot22 = new Pose(10.0,5.0,0.0);
		Pose finalRobot22 = new Pose(35.0,5.0,0.0);

		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setMapFilename("maps"+File.separator+Missions.getProperty("image", yamlFile));
		double res = 0.2;// Double.parseDouble(getProperty("resolution", yamlFile));
		rsp.setMapResolution(res);
		rsp.setRobotRadius(1.1);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.3);

		rsp.setStart(startRobot1);
		rsp.setGoals(finalRobot1);
		rsp.plan();
		Missions.putMission(new Mission(1,rsp.getPath()));

		rsp.setStart(startRobot21);
		rsp.setGoals(finalRobot21);
		rsp.plan();
		Missions.putMission(new Mission(2,rsp.getPath()));

		rsp.setStart(startRobot22);
		rsp.setGoals(finalRobot22);
		rsp.plan();
		Missions.putMission(new Mission(2,rsp.getPath()));

		tec.placeRobot(1, startRobot1);
		tec.placeRobot(2, startRobot21);

		tec.addMissions(Missions.getMission(2, 0));
		tec.computeCriticalSections();
		tec.startTrackingAddedMissions();

		Thread.sleep(5000);
		
		tec.addMissions(Missions.getMission(1, 0));
		tec.computeCriticalSections();
		tec.startTrackingAddedMissions();
		
		//Thread.sleep(10000);
		
		while (!tec.isFree(2)) { Thread.sleep(100); }
		
		tec.addMissions(Missions.getMission(2, 1));
		tec.computeCriticalSections();
		tec.startTrackingAddedMissions();
		
				
	}
	
}