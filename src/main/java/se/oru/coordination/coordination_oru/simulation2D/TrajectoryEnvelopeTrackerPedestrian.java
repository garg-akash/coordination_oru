package se.oru.coordination.coordination_oru.simulation2D;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Scanner;
import java.util.TreeMap;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import se.oru.coordination.coordination_oru.AbstractTrajectoryEnvelopeTracker;
import se.oru.coordination.coordination_oru.RobotReport;
import se.oru.coordination.coordination_oru.TrackingCallback;
import se.oru.coordination.coordination_oru.TrajectoryEnvelopeCoordinator;
import se.oru.coordination.coordination_oru.util.ColorPrint;

public abstract class TrajectoryEnvelopeTrackerPedestrian extends AbstractTrajectoryEnvelopeTracker implements Runnable {

	protected static final long WAIT_AMOUNT_AT_END = 3000;
	protected static final double EPSILON = 0.01;
	
	protected double overallDistance = 0.0;
	protected double totalDistance = 0.0;
	
	protected double elapsedTrackingTime = 0.0;
	protected Pose currentPose;
	protected double stoppageTime = 0.0;
	protected double positionToStop = -1.0;
	
	protected ArrayList<Pose> pedestrianPoses = null;
	protected ArrayList<Double> pedestrianSpeeds = null;
	protected ArrayList<Double> pedestrianTimeStamps = null;
	protected ArrayList<Double> pedestrianDTs = null;
	
	private Thread th = null;
	private int maxDelayInMilis = 0;
	private Random rand = new Random();
	
	/**
	 * Read a path from a file.
	 * @param fileName The name of the file containing the pedestrian path
	 * @return The pedestrian path read from the file
	 */
	private void loadPedestrianTrajectoryFromFile(String fileName) {
		this.pedestrianPoses = new ArrayList<Pose>();
		this.pedestrianSpeeds = new ArrayList<Double>();
		this.pedestrianTimeStamps = new ArrayList<Double>();
		this.pedestrianDTs = new ArrayList<Double>();
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			while (in.hasNextLine()) {
				String line = in.nextLine().trim();
				if(line.length() == 5) {
					String[] oneline = line.split(" |\t");
					pedestrianPoses.add(new Pose(
							new Double(oneline[0]).doubleValue(),
							new Double(oneline[1]).doubleValue(),
							new Double(oneline[2]).doubleValue()));
					pedestrianSpeeds.add(new Double(oneline[3]).doubleValue());
					pedestrianTimeStamps.add(new Double(oneline[4]).doubleValue());
				}
				else {
					ColorPrint.error("Error reading a line from " + fileName + ". Line did not contain 5 elements. Skipping line.");
					continue;
				}

			}
			in.close();
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }

		// Compute deltaTs for the Trajectory.
		this.pedestrianDTs = new ArrayList<Double>();
		for(int i = 0; i < this.pedestrianTimeStamps.size() - 1; i++) {
			this.pedestrianDTs.add(new Double(this.pedestrianTimeStamps.get(i+1).doubleValue() - pedestrianTimeStamps.get(i).doubleValue()));
		}
		pedestrianDTs.add(new Double(0.0));

	}
	
	public TrajectoryEnvelopeTrackerPedestrian(TrajectoryEnvelope te, int timeStep, double temporalResolution, TrajectoryEnvelopeCoordinator tec, TrackingCallback cb, String fileName) {
		super(te, temporalResolution, tec, timeStep, cb);
		this.loadPedestrianTrajectoryFromFile(fileName);
		this.currentPose = pedestrianPoses.get(0);
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.overallDistance = totalDistance;
		this.th = new Thread(this, "Pedestrian tracker " + te.getComponent());
		this.th.setPriority(Thread.MAX_PRIORITY);
	}
	
	@Override
	public void onTrajectoryEnvelopeUpdate(TrajectoryEnvelope te) {
		this.totalDistance = this.computeDistance(0, traj.getPose().length-1);
		this.overallDistance = totalDistance;
	}
	
	@Override
	public void startTracking() {
		while (this.th == null) {
			try { Thread.sleep(10); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		this.th.start();
	}

	public static double computeDistance(Trajectory traj, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < endIndex; i++) {
			ret += traj.getPose()[i].distanceTo(traj.getPose()[i+1]);
		}
		return ret;
	}

	private double computeDistance(int startIndex, int endIndex) {
		return computeDistance(this.traj, startIndex, endIndex);
	}
			
	@Override
	public void setCriticalPoint(int criticalPointToSet) {
		ColorPrint.info("Setting critical point for Pedestrian...");
		if (this.criticalPoint != criticalPointToSet) {
			
			//A new intermediate index to stop at has been given
			if (criticalPointToSet != -1 && criticalPointToSet > getRobotReport().getPathIndex()) {			
				//Store backups in case we are too late for critical point
				double totalDistanceBKP = this.totalDistance;
				int criticalPointBKP = this.criticalPoint;
	
				this.criticalPoint = criticalPointToSet;
				this.totalDistance = computeDistance(0, criticalPointToSet);
			}

			//Critical point <= current position, ignore -- WHY??
			else if (criticalPointToSet != -1 && criticalPointToSet <= getRobotReport().getPathIndex()) {
				metaCSPLogger.warning("Ignored critical point (" + te.getComponent() + "): " + criticalPointToSet + " because robot is already at " + getRobotReport().getPathIndex() + " (and current CP is " + this.criticalPoint + ")");
			}
			
			//The critical point has been reset, go to the end
			else if (criticalPointToSet == -1) {
				this.criticalPoint = criticalPointToSet;
				this.totalDistance = computeDistance(0, traj.getPose().length-1);
				metaCSPLogger.finest("Set critical point (" + te.getComponent() + "): " + criticalPointToSet);
			}
		}
		
		//Same critical point was already set
		else {
			metaCSPLogger.warning("Critical point (" + te.getComponent() + ") " + criticalPointToSet + " was already set!");
		}
		
	}
	
	@Override
	public RobotReport getRobotReport() {
		if (this.currentPose == null) return null;
		if (!this.th.isAlive()) return new RobotReport(te.getRobotID(), traj.getPose()[0], -1, 0.0, 0.0, -1);
		synchronized(state) {
			Pose pose = null;
			int currentPathIndex = -1;
			double accumulatedDist = 0.0;
			Pose[] poses = traj.getPose();
			for (int i = 0; i < poses.length-1; i++) {
				double deltaS = poses[i].distanceTo(poses[i+1]);
				accumulatedDist += deltaS;
				if (accumulatedDist > state.getPosition()) {
					double ratio = 1.0-(accumulatedDist-state.getPosition())/deltaS;
					pose = poses[i].interpolate(poses[i+1], ratio);
					currentPathIndex = i;
					break;
				}
			}
			if (currentPathIndex == -1) {
				currentPathIndex = poses.length-1;
				pose = poses[currentPathIndex];
			}
			return new RobotReport(te.getRobotID(), pose, currentPathIndex, state.getVelocity(), state.getPosition(), this.criticalPoint);
		}
	}

	public void delayIntegrationThread(int maxDelayInmillis) {
		this.maxDelayInMilis = maxDelayInmillis;
	}
	
	protected void updatePedestrianState() {
		if(this.elapsedTrackingTime < this.pedestrianTimeStamps.get(0).doubleValue()) { return; }
		
		double timePassed = this.elapsedTrackingTime - this.stoppageTime;
		
	}
	
	@Override
	public void run() {
		this.elapsedTrackingTime = 0.0;
		double deltaTime = 0.0;
		boolean atCP = false;
		int myRobotID = te.getRobotID();
		int myTEID = te.getID();
		
		while (true) {
						
			//End condition: passed the middle AND velocity < 0 AND no criticalPoint 			
			boolean skipIntegration = false;
			
			if (state.getVelocity() < 0.0) {
				if (criticalPoint == -1 && !atCP) {
					//set state to final position, just in case it didn't quite get there (it's certainly close enough)
					state = new State(totalDistance, 0.0);
					onPositionUpdate();
					break;
				}
								
				//Vel < 0 hence we are at CP, thus we need to skip integration
				if (!atCP /*&& getRobotReport().getPathIndex() == criticalPoint*/) {
					metaCSPLogger.info("At critical point (" + te.getComponent() + "): " + criticalPoint + " (" + getRobotReport().getPathIndex() + ")");
					atCP = true;
				}			
				skipIntegration = true;
			}

			//Compute deltaTime
			long timeStart = Calendar.getInstance().getTimeInMillis();
			
			//Update the robot's state via RK4 numerical integration
			if (!skipIntegration) {
				if (atCP) {
					metaCSPLogger.info("Resuming from critical point (" + te.getComponent() + ")");
					atCP = false;
				}
				updatePedestrianState();
			}
			
			//Do some user function on position update
			onPositionUpdate();
						
			//Sleep for tracking period
			int delay = trackingPeriodInMillis;
			if (maxDelayInMilis > 0) delay += rand.nextInt(maxDelayInMilis);
			try { Thread.sleep(delay); }
			catch (InterruptedException e) { e.printStackTrace(); }
			
			//Advance time to reflect how much we have slept (~ trackingPeriod)
			long deltaTimeInMillis = Calendar.getInstance().getTimeInMillis()-timeStart;
			deltaTime = deltaTimeInMillis/this.temporalResolution;
			this.elapsedTrackingTime += deltaTime;
			// If we have skipped integration, add this to stoppage time.
			if(skipIntegration)	this.stoppageTime += deltaTime;
		}
		
		//persevere with last path point in case listeners didn't catch it!
		long timerStart = getCurrentTimeInMillis();
		while (getCurrentTimeInMillis()-timerStart < WAIT_AMOUNT_AT_END) {
			//System.out.println("Waiting " + te.getComponent());
			try { Thread.sleep(trackingPeriodInMillis); }
			catch (InterruptedException e) { e.printStackTrace(); }
		}
		metaCSPLogger.info("RK4 tracking thread terminates (Robot " + myRobotID + ", TrajectoryEnvelope " + myTEID + ")");
	}

}
