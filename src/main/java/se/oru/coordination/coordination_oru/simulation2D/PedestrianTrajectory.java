package se.oru.coordination.coordination_oru.simulation2D;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.lang.Math;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import se.oru.coordination.coordination_oru.util.ColorPrint;

public class PedestrianTrajectory {

	protected ArrayList<Pose> poses = null;
	protected ArrayList<Double> speeds = null;
	protected ArrayList<Double> timeStamps = null;
	protected ArrayList<Double> dts = null;
	protected ArrayList<Double> us = null;
	protected ArrayList<Double> vs = null;
	
	protected int size = 0;
	
	public ArrayList<Pose> getPoses() { return poses; }
	public Pose getPose(int index) { return poses.get(index); } 
	
	public ArrayList<Double> getTimeStamps() { return timeStamps; }
	public double getTimeStamp(int index) { return timeStamps.get(index).doubleValue(); }
	
	public ArrayList<Double> getDTs() { return dts; }
	public double getDT(int index) { return dts.get(index).doubleValue(); }
	
	public ArrayList<Double> getSpeeds() { return speeds; }
	public double getSpeed(int index) { return speeds.get(index).doubleValue(); }
	
	public ArrayList<Double> getUs() { return us; }
	public double getU(int index) { return us.get(index).doubleValue(); }
	
	public ArrayList<Double> getVs() { return vs; }
	public double getV(int index) { return vs.get(index).doubleValue(); }

	public PoseSteering[] getPoseSteeringAsArray() {
		PoseSteering[] poseSteering = new PoseSteering[this.poses.size()];
		for(int i = 0; i < this.poses.size(); i++) {
			poseSteering[i] = new PoseSteering(this.poses.get(i), 0.0);
		}
		return poseSteering;
	}

	public Pose[] getPosesAsArray() {
		return this.poses.toArray(new Pose[this.poses.size()]);
	}

	public double[] getSpeedsAsArray() {
		double[] speeds = new double[this.speeds.size()];
		for(int i = 0; i < this.speeds.size(); i++) {
			speeds[i] = this.speeds.get(i).doubleValue();
		}
		return speeds;
	}

	public double[] getTimeStampsAsArray() {
		double[] timeStamps = new double[this.timeStamps.size()];
		for(int i = 0; i < this.timeStamps.size(); i++) {
			timeStamps[i] = this.timeStamps.get(i).doubleValue();
		}
		return timeStamps;
	}

	public double[] getDTsAsArray() {
		double[] dts = new double[this.dts.size()];
		for(int i = 0; i < this.dts.size(); i++) {
			dts[i] = this.dts.get(i).doubleValue();
		}
		return dts;
	}
	
	public int size() { return size; }
	
	public PedestrianTrajectory(String fileName) {
		loadTrajectoryFromFile(fileName);
	}
	
	private static boolean check(List ... arrays) {
		if (arrays.length < 1) {
			return true;
		}
		int expectedSize = arrays[0].size();
		for (int i = 1; i < arrays.length; i++) {
			int sizeOfThisList = arrays[i].size();
			if (sizeOfThisList != expectedSize) {
				return false;
			}
		}
		return true;
	}

	/**
	 * Read a path from a file.
	 * @param fileName The name of the file containing the pedestrian path
	 * @return The pedestrian path read from the file
	 */
	public void loadTrajectoryFromFile(String fileName) {
		this.poses = new ArrayList<Pose>();
		this.speeds = new ArrayList<Double>();
		this.timeStamps = new ArrayList<Double>();
		this.dts = new ArrayList<Double>();
		this.us = new ArrayList<Double>();
		this.vs = new ArrayList<Double>();
		try {
			Scanner in = new Scanner(new FileReader(fileName));
			while (in.hasNextLine()) {
				String[] oneline = in.nextLine().trim().split(" |\t");
				if(oneline.length == 6) {
					poses.add(new Pose(
							new Double(oneline[0]).doubleValue(),
							new Double(oneline[1]).doubleValue(),
							new Double(oneline[2]).doubleValue()));
					
					double thisU = new Double(oneline[3]).doubleValue();
					double thisV = new Double(oneline[4]).doubleValue();
					us.add(new Double(thisU));
					vs.add(new Double(thisV));
					
					speeds.add(new Double(Math.sqrt(thisU*thisU + thisV*thisV)));
					timeStamps.add(new Double(oneline[5]).doubleValue());
				}
				else {
					ColorPrint.error("Error reading a line from " + fileName + ". Line did not contain 5 elements. Skipping line.");
					continue;
				}

			}
			in.close();
			
			if(check(poses, speeds, timeStamps)) { size = poses.size(); }
			else {
				size = Math.max(timeStamps.size(), Math.max(poses.size(), speeds.size()));
				throw new IllegalArgumentException(ColorPrint.ANSI_RED + "ERROR in PedestrianTrajectory.java. Poses, speeds and time stamps read from the file were not all the same size. Check the file." + ColorPrint.ANSI_RESET); 
			}
		}
		catch (FileNotFoundException e) { e.printStackTrace(); }

		// Compute deltaTs for the Trajectory.
		this.dts = new ArrayList<Double>();
		for(int i = 0; i < this.timeStamps.size() - 1; i++) {
			this.dts.add(new Double(this.timeStamps.get(i+1).doubleValue() - timeStamps.get(i).doubleValue()));
		}
		dts.add(new Double(0.0));

	}
}
