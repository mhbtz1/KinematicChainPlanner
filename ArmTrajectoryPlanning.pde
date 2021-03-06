import java.io.*;
import java.util.*;
import java.nio.*;
/*
Forward Kinematics for a 2D kinematic chain can be figured out using some trigonometry stuff although in order to generalise to a 3D space
we can also use the denavit hartenberg matrices in order to solve the FK problem (although this is decidedly harder)
Inverse Kinematics in 2D, though, is comparably difficult to in 3D space, and for this we will implement an IK algorithm called cyclic coordinate descent
https://www.ryanjuckett.com/cyclic-coordinate-descent-in-2d/
The idea is that you try to optimize each individual arm of your chain by attempting to move it closer to the target position and iteratively
attempt to move closer and closer to the goal state
If you are familiar with machine learning, the forward and inverse kinematics processes can be thought of similar to the feedforward and backpropagation procedures in a 
neural network, since we can theoretically optimize a chain with any amount of arms in it.
The main issues that are prevalent with CCD is that it tends to make the arm move in unreasonable ways, and in some cases can lead to NaN in some values
due to weird numerical errors (although I think this can be resolved easily).
*/

/*
PROJECT STRUCTURE: 
You'll probably notice that we have two important classes here: ForwardKinematics, InverseKinematics, and ArmSegment
The way that we will do this is that we will have the InverseKinematics class extend the ForwardKinematics class, because we need to use FK
to find out where our end effector is after changing some angle.
ArmSegment is a class for storing the information about each arm in our kinematic chain.
Both classes take the parameters:
   1. ArrayList<ArmSegment> kinematicChain: encodes the structure of the chain for us
   2. PVector seed_position: the position at which the arm is rooted
   3. ArrayList<Float> arm_lengths: this stores the length of each arm for us, but it is technically redundant since none of our arms extend, only rotate
   4. ArrayList<Float> angles: this stores the angle information for the chain for us
   
The only caveat is that since InverseKinematics extends ForwardKinematics, we can all APPLY_FK() from InverseKinematics to find out where our end effector is
and we do this after each call of cyclic_coordinate_descent() as well
*/

TrajectoryPlanner trj;
PathPlanner ida;
MatrixOps matPipeline;
RRT myRRT;
PrintWriter trajectoryInformation;

Location cur_loc = new Location(200, 300);
Location goal_loc = new Location(1200, 700);
ArrayList<ArmParameters> armStateMachine = new ArrayList<ArmParameters>();
int IK_PTR = 0;
float TIME_PTR = 0;
float TIME_DELTA = 0.15;
boolean GOAL_STATE_CHANGED = false;
boolean fillPathInfo = true;
boolean TEST_RRT = true;
boolean RRT_HAS_BEEN_GENERATED = false;
boolean draw_obstacle = false;
boolean USE_TRAJECTORY_PLANNING = true;
boolean TRAJECTORY_NOT_GENERATED = true;
boolean START_RECORDING = false;
boolean ARM_RRT_WAYPOINTS_NOT_GENERATED = true;
int CHAIN_SIZE = 14;

PrintWriter controlPoints;
//these will be the parameters for one 2D kinematic chain (we can add more later and generalise this)
PVector cur = new PVector(680,400);
ArrayList<ArmSegment> robotArm = new ArrayList<ArmSegment>();
ArrayList<Float> arm_lengths = new ArrayList<Float>();
ArrayList<Float> angles = new ArrayList<Float>();
ArrayList<Float> angles_copy = new ArrayList<Float>();
ArrayList<PVector> target_points = new ArrayList<PVector>();
ArrayList< ArrayList<Float> > angle_targets = new ArrayList< ArrayList<Float> >();
InverseKinematics IK1, IK2;
//for drawing, maintain list of reached waypoints:
ArrayList<PVector> reached_points = new ArrayList<PVector>();


void runRRT(boolean useVoronoiBias){
  if(useVoronoiBias){
     background(0);
     MotionProfiler M_P = new MotionProfiler();
     if(!draw_obstacle){
       if(TEST_RRT){
        fill(255,0,0);
        circle((float)goal_loc.x, (float)goal_loc.y, 8);
        if(!myRRT.rrtVoronoiBias()){
          myRRT.displayRRT(myRRT.seed);
          myRRT.reset();
          //when we run IDA, we want to check to go to the node which is closest to our goal node (in the case that our goal node isn't in the RRT, which it likely isnt.)
          Location target = null;
          float rmin = 1000000007;
          for(PVector g : myRRT.graph.keySet()){
            rmin = min(rmin, dist(g.x,g.y,(float)goal_loc.x,(float)goal_loc.y) );
            if(rmin == dist(g.x,g.y,(float)goal_loc.x,(float)goal_loc.y) ){
              target = new Location(g.x,g.y); //make a copy of it so that original isnt edited
            }
          }
          ida = new PathPlanner(myRRT.graph, cur_loc, target);
          ArrayList<PVector> myPath = ida.IDA();
          for(int i = 0; i < myPath.size()-1; i++){
            stroke(255,0,255);
            line(myPath.get(i).x, myPath.get(i).y, myPath.get(i+1).x, myPath.get(i+1).y);
          }
          ArrayList<PVector> augmented = ida.augment_waypoints(0.04);
          for(PVector p: augmented){fill(255,0,255); circle(p.x,p.y,4);}
          M_P.iterate_profiles();
          fill(0,255,0);
          for(Location true_w : M_P.true_waypoints){
            circle( (float)true_w.x, (float)true_w.y, 8);
          }
          stroke(0,0,255);
        }
       }
       String[] s = loadStrings("goal.txt");
       String line = s[0]; 
       int one = Integer.parseInt(line.substring(0, line.indexOf(":")));
       int two = Integer.parseInt(line.substring(line.indexOf(":")+1));
       if(one != this.goal_loc.x || two != this.goal_loc.y){
         this.goal_loc = new Location(one,two);
         GOAL_STATE_CHANGED = true;
       }
    }
  } else {
    //use Halton sequence for sampling
     background(0);
     MotionProfiler M_P = new MotionProfiler();
      if(!draw_obstacle){
       if(TEST_RRT){
        fill(255,0,0);
        circle( (float)goal_loc.x, (float)goal_loc.y, 8);
        if(!myRRT.rrtHalton()){
          myRRT.displayRRT(myRRT.seed);
          myRRT.reset();
          //when we run IDA, we want to check to go to the node which is closest to our goal node (in the case that our goal node isn't in the RRT, which it likely isnt.)
          Location target = null;
          float rmin = 1000000007;
          for(PVector g : myRRT.graph.keySet()){
            rmin = min(rmin, dist(g.x,g.y,goal_loc.x,goal_loc.y) );
            if(rmin == dist(g.x,g.y,goal_loc.x,goal_loc.y) ){
              target = new Location(g.x,g.y); //make a copy of it so that original isnt edited
            }
          }
          ida = new PathPlanner(myRRT.graph, cur_loc, target);
          ArrayList<PVector> myPath = ida.IDA();
          for(int i = 0; i < myPath.size()-1; i++){
            stroke(255,0,255);
            line(myPath.get(i).x, myPath.get(i).y, myPath.get(i+1).x, myPath.get(i+1).y);
          }
          ArrayList<PVector> augmented = ida.augment_waypoints(0.04);
          for(PVector p: augmented){fill(255,0,255); circle(p.x,p.y,4);}
          M_P.iterate_profiles();
          fill(0,255,0);
          for(Location true_w : M_P.true_waypoints){
            circle(true_w.x, true_w.y, 8);
          }
          stroke(0,0,255);
        }
       }
       String[] s = loadStrings("goal.txt");
       String line = s[0]; 
       int one = Integer.parseInt(line.substring(0, line.indexOf(":")));
       int two = Integer.parseInt(line.substring(line.indexOf(":")+1));
       if(one != this.goal_loc.x || two != this.goal_loc.y){
         this.goal_loc = new Location(one,two);
         GOAL_STATE_CHANGED = true;
       }
    }
  }
}



//it could be cool to make a way to create multiple chains onscreen and have each of them running their own IK procedures
//This ArmParameters class can be used in order to streamline that process.
//Also, you could change the arm generation method and maybe try to find ways to generate "better" chains(WIP).

public class ArmParameters{
  PVector cur;
  ArrayList<ArmSegment> robotArm;
  ArrayList<Float> arm_lengths;
  ArrayList<Float> angles;
  PVector target_point;
  public ArmParameters(PVector cur, ArrayList<ArmSegment> robotArm, ArrayList<Float> arm_lengths, ArrayList<Float> angles, PVector target_point){
    this.cur = cur;
    this.robotArm = robotArm;
    this.arm_lengths = arm_lengths;
    this.angles = angles;
    this.target_point = target_point;
  }
}




//TODO: this method should generate an arm with "LENGTH" amount of arms in it, and to make the arms reasonable,
//we will keep each arm length a fixed length (indicated by len), and randomize the angle at which we should turn the arm
//and then move "len" units in that direction. This may be a bit of a confusing description, but I will likely clarify.

public void GENERATE_CUSTOM_ARM(int LENGTH){
    
   PVector init = new PVector(cur.x,cur.y);
   float len = 90;
   for(int i = 0; i < LENGTH; i++){
     //WRITE CODE HERE
      float n_ang = random(0,2*PI);
      PVector nxt = new PVector(cur.x + (len * cos(n_ang)), cur.y + (len*sin(n_ang)) );
      println(cur.x + " " + cur.y + " " + nxt.x + " " + nxt.y);
      float dx = nxt.x - cur.x;
      float dy = nxt.y-cur.y;
      robotArm.add(new ArmSegment(init, nxt) );
      arm_lengths.add(sqrt( dx*dx + dy*dy) );
      angles.add(atan( (float)(nxt.y - init.y)/(float)(nxt.x - init.x) ) );
      init = nxt;
    }
    angles_copy = (ArrayList)angles.clone();
}

//TODO: This method takes an InverseKinematics object and draws the kinematic chain stored in it.
public void DRAW_ROBOT_ARM(InverseKinematics IK){
     PVector start = new PVector(IK.seed_position.x, IK.seed_position.y);
     for(int i = 0; i < IK.arm_lengths.size(); i++){
       //WRITE CODE HERE
       noStroke();
       fill(255,255,255);
       ellipse(start.x,start.y,8,8);
       ellipse(start.x + (IK.arm_lengths.get(i))*cos(IK.angles.get(i)), start.y + (IK.arm_lengths.get(i)*sin(IK.angles.get(i))), 8, 8);
       stroke(0,255,0);
       line(start.x, start.y, start.x + (IK.arm_lengths.get(i))*cos(IK.angles.get(i)), start.y + (IK.arm_lengths.get(i)*sin(IK.angles.get(i))) );
       start = new PVector(start.x + (IK.arm_lengths.get(i))*cos(IK.angles.get(i)), start.y + (IK.arm_lengths.get(i)*sin(IK.angles.get(i))));
     }
}

//This is a utility class for encoding information about arms in the chain. This class is provided.
class ArmSegment{
  PVector start;
  PVector end;
  public ArmSegment(PVector one, PVector two){
    this.start = one;
    this.end = two;
  }
}

class ForwardKinematics{
  PVector seed_position;
  ArrayList<ArmSegment> kinematic_chain;
  ArrayList<Float> angles, arm_lengths;
  public ForwardKinematics(ArrayList<ArmSegment> kinematic_chain, PVector seed_position, ArrayList<Float> angles, ArrayList<Float> arm_lengths){
    this.kinematic_chain = kinematic_chain;
    this.seed_position = seed_position;
    this.angles = angles;
    this.arm_lengths = arm_lengths;
  }
  //TODO: This method returns a PVector which is the position of the end of our chain. We have to use the angle information as well as the length of each
  //arm to find out where the end of our chain is in space. There is also some prerequisite understanding of trigonometry that is assumed.
  
  //This method is pretty short, so don't overthink it.
  public PVector APPLY_FK(){
    float angSum = this.angles.get(0);
    PVector ret = new PVector(seed_position.x,seed_position.y);
    ellipse(ret.x,ret.y,8,8);
    for(int i = 0; i < kinematic_chain.size(); i++){
      //WRITE CODE HERE
      ret.x += this.arm_lengths.get(i) * cos(this.angles.get(i));
      ret.y += this.arm_lengths.get(i) * sin(this.angles.get(i));
      angSum += this.angles.get(i);
    }
    return ret;
  }
  //TODO: Given a new ArrayList<Float> of angles for our arm, we need to update the ArrayList<ArmSegment> structure that is actually storing 
  //info about the points in our chain. This method is also pretty short.
  public ArrayList<ArmSegment> UPDATE_KINEMATIC_CHAIN(ArrayList<Float> newAngles){
    PVector start = new PVector(seed_position.x,seed_position.y);
    ArrayList<ArmSegment> updatedArm = new ArrayList<ArmSegment>();
    for(int i = 0; i < newAngles.size(); i++){
      //WRITE CODE HERE
      PVector nxt = new PVector(start.x + this.arm_lengths.get(i) * cos(this.angles.get(i)), start.y  + this.arm_lengths.get(i)*sin(this.angles.get(i)));
      updatedArm.add(new ArmSegment(start,nxt));
      start = nxt;
    }
    return updatedArm;
  }
}


//This is where a lot of the hard stuff begins to rear its head, and I'll probably talk on this the most, but the guide for the
//inverse kinematics stuff is really good, and provides some nice visuals. This also assumes some understanding of vectors and basic vector subtraction
//(at least in order to understand some of the mathematics)

class InverseKinematics extends ForwardKinematics{
  
  //these parameters eps_x and eps_y set an error bound for our CCD function (since we don't necessarily want to be right on the point, just close enough)
  public static final float eps_x = 2;
  public static final float eps_y = 2;
  PVector end_effector_position;
  public InverseKinematics(ArrayList<ArmSegment> kinematic_chain, PVector seed_position, ArrayList<Float> angles, ArrayList<Float> arm_lengths){
    super(kinematic_chain,seed_position,angles,arm_lengths);
    this.end_effector_position = this.APPLY_FK();
  }
  
  public double angleReduction(double angle){
    angle = angle % (2.0*PI);
    if( angle < PI )
        angle += (2.0 * PI);
    else if( angle > PI )
        angle -= (2.0 * PI);
    return angle;
  }
  
  //TODO: this method will run iteratively in draw(), but we will have the method return TRUE if a solution has been found, and FALSE otherwise.
  //It takes the parameter of some position in 2D space (for the moment, we will assume that the arm will always be able to reach desired_position, so we don't get weird cases)
  
  public ArrayList<Float> cyclic_coordinate_descent(PVector desired_position){
    boolean run =  abs(end_effector_position.x-desired_position.x)>eps_x || abs(end_effector_position.y-desired_position.y) > eps_y;
    float epsilon = 0.01;
    if(run){
      for(int i = this.kinematic_chain.size()-1; i >= 0; i--){
        PVector j = new PVector( this.kinematic_chain.get(i).start.x, this.kinematic_chain.get(i).start.y);
        PVector efp = new PVector(end_effector_position.x, end_effector_position.y);
        PVector dp = new PVector(desired_position.x, desired_position.y);
        //WRITE CODE HERE:
        
        
        efp.sub(j); //this is (e-j)
        dp.sub(j); //this is (t-j)
        float mg = efp.mag() * dp.mag();
 
        //indicates amount of rotation
        float p1 = (float)(efp.dot(dp));
        
        float cos_angle = acos(max(-1, min(1,(float)p1/(float)mg)));
        //indicates direction of rotation
        float p2 =  (float)(((efp.x * dp.y) - (efp.y*dp.x)));
        float sin_angle = asin(max(-1, min(1,(float)p2/(float)mg)));
        //i think the weird issues with NaN in our CCD algo may be due to some domain issues with asin() and acos()
        if(dist(desired_position.x, desired_position.y, this.end_effector_position.x, this.end_effector_position.y) <= epsilon){
          cos_angle = 1;
          sin_angle = 0;
        }
        
        float turning_angle = 0;
        turning_angle = cos_angle;
        
        
        if( p2 < 0){
          turning_angle *= -1;
        }

        //given new angle of some joint, we need to determine the new forward kinematics of the system;
        //this will make it so we dont get super big angles.
        float new_angle = (float)(angleReduction(this.angles.get(i) + turning_angle));
        this.angles.set(i, new_angle);
        
        //now, we call methods to update the position of the end effector and mutate our chain.
        this.end_effector_position = this.APPLY_FK();
        this.kinematic_chain = this.UPDATE_KINEMATIC_CHAIN(this.angles);
        
      }
      return new ArrayList<Float>();
    }
    return this.angles;
  }
}


void setup(){
  background(0);
  trajectoryInformation = createWriter("trajInfo.txt");
  size(1400,900);
  matPipeline = new MatrixOps();
  trj = new TrajectoryPlanner();
  myRRT = new RRT(new PVector(cur_loc.x,cur_loc.y), 45, 1000);
  controlPoints = createWriter("pathPlanner.txt");
  float[][] A = { {1, 2, 3, 4}, {5, 6, 7, 8}, {1, 2, 3, 4}, {5, 6, 7, 8} };
  float[][] B = { {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1} };
  int RAD = 300;
  
  /*
  for(float f = 0; f <= 2*PI; f+= 0.01){
    target_points.add(new PVector(cur.x + (2*RAD*cos(f)),cur.y + RAD*sin(f)) );
  }
  */
  

  /*
  for(float f = 0; f <= 2*PI; f+= 0.01){
    target_points.add(new PVector(cur.x + (RAD*cos(f)),cur.y + RAD*sin(f)) );
  }
  for(float f = 0; f <= 2*PI; f+= 0.01){
    target_points.add(new PVector(cur.x + (RAD*cos(f)),cur.y + 0.5*RAD*sin(f)) );
  }
  for(float f = 0; f <= 2*PI; f+= 0.01){
    target_points.add(new PVector(cur.x + (0.25*RAD*cos(f)),cur.y + 0.5*RAD*sin(f)) );
  }
  */
  PVector vertex = new PVector(680,700);
  HaltonSampler hq = new HaltonSampler(1400,900);
  hq.genHalton(2000);
 
 //testing trajectory planning with halton sequence as our waypoints
 if(!TEST_RRT){
   for(int f = 50; f < 100; f++){
    target_points.add(hq.HALTON_POINTS.get(f));
   }
 }
  
  /*
  for(int i = 0; i < hq.HALTON_POINTS.size(); i++){
    target_points.add(hq.HALTON_POINTS.get(i));
  }
  */
  
  GENERATE_CUSTOM_ARM(CHAIN_SIZE);//will modify this to generate "better" kinematic chains
  IK1 = new InverseKinematics(robotArm, cur, angles, arm_lengths);
  IK2 = new InverseKinematics( (ArrayList)robotArm.clone(), new PVector(cur.x,cur.y), (ArrayList)angles.clone(), (ArrayList)arm_lengths.clone());
  /*
  ArrayList<TrajecMatrixSolver> firstPos = trj.quinticTraj.get(0);
  float timer = 0;
  while(timer <= 3){
    DRAW_ROBOT_ARM(IK1);
    ArrayList<Float> new_pose = new ArrayList<Float>();
    for(int i = 0; i < firstPos.size(); i++){
      double[] solution_vector = firstPos.get(i).solution_vector;
      float new_angle = 0;
      for(int j = 0; j < solution_vector.length; j++){
        new_angle += (solution_vector[j]) * (pow(TIME_PTR,j));
      }
      new_pose.add(new_angle);
    }
    for(int i = 0; i < new_pose.size(); i++){
      IK1.angles.set(i,new_pose.get(i));
    }
    timer += TIME_DELTA;
  }
  */
  
  /*
  for(float f = 280; f <= 800; f++){
    target_points.add(new PVector(420,f));
  }
  for(float f = 420; f <= 940; f++){
    target_points.add(new PVector(f, 800));
  }
  for(float f = 800; f >= 280; f--){
    target_points.add(new PVector(940,f));
  }
  for(float f = 940; f >= 420; f--){
    target_points.add(new PVector(f,280));
  }
  */
  //armStateMachine.add(new ArmParameters(cur,new ArrayList<ArmSegment>(),new ArrayList<Float>(), new ArrayList<Float>(), target_points.get(0)));

  /*
  for(int i = 0; i < target_points.size(); i++){
    while(IK.cyclic_coordinate_descent(target_points.get(i)).size() > 0){
      IK_PTR = (IK_PTR + 1)%(target_points.size());
    }
  }
  */
}

void draw(){
  background(0);
  //RRT path planner
  MotionProfiler M_P = new MotionProfiler();
   if(TEST_RRT && !draw_obstacle && START_RECORDING){
      fill(255,0,0);
      circle(goal_loc.x, goal_loc.y, 8);
      if(!myRRT.rrtVoronoiBias()){
        myRRT.displayRRT(myRRT.seed);
        myRRT.reset();
        //when we run IDA, we want to check to go to the node which is closest to our goal node (in the case that our goal node isn't in the RRT, which it likely isnt.)
        Location target = null;
        float rmin = 1000000007;
        for(PVector g : myRRT.graph.keySet()){
          rmin = min(rmin, dist(g.x,g.y,goal_loc.x,goal_loc.y) );
          if(rmin == dist(g.x,g.y,goal_loc.x,goal_loc.y) ){
            target = new Location(g.x,g.y); //make a copy of it so that original isnt edited
          }
        }
        ida = new PathPlanner(myRRT.graph, cur_loc, target);
        ArrayList<PVector> myPath = ida.IDA();
        for(int i = 0; i < myPath.size()-1; i++){
          stroke(255,0,255);
          line(myPath.get(i).x, myPath.get(i).y, myPath.get(i+1).x, myPath.get(i+1).y);
        }
        ArrayList<PVector> augmented = ida.augment_waypoints(0.04);
        for(PVector p: augmented){fill(255,0,255); circle(p.x,p.y,4);}
        M_P.iterate_profiles();
        fill(0,255,0);
        for(Location true_w : M_P.true_waypoints){
          if(ARM_RRT_WAYPOINTS_NOT_GENERATED){
            target_points.add(new PVector(true_w.x, true_w.y));
          }
          circle(true_w.x, true_w.y, 8);
        }
        ARM_RRT_WAYPOINTS_NOT_GENERATED = false;
        stroke(0,0,255);
      }
    
     String[] s = loadStrings("goal.txt");
     String line = s[0]; 
     int one = Integer.parseInt(line.substring(0, line.indexOf(":")));
     int two = Integer.parseInt(line.substring(line.indexOf(":")+1));
     if(one != this.goal_loc.x || two != this.goal_loc.y){
       this.goal_loc = new Location(one,two);
       GOAL_STATE_CHANGED = true;
     }
     
   }
       /*
       if(SET_OF_WAYPOINTS){
         mr.iterate_profiles();
       }
       */
     //}
     //path_planning_two();
   if((TEST_RRT && RRT_HAS_BEEN_GENERATED) && START_RECORDING){
    if(USE_TRAJECTORY_PLANNING){
      stroke(255);
      if(TRAJECTORY_NOT_GENERATED){
        stroke(0);
        text("GENERATING TRAJECTORY...", 600, 20);
        DRAW_ROBOT_ARM(IK1);
        println();
        PVector ret = IK1.APPLY_FK();
        noStroke();
        ellipse(ret.x, ret.y, 10, 10);
        for(PVector p : reached_points){noStroke(); fill(0,0,255); ellipse(p.x, p.y, 8, 8);}
        PVector target_point = target_points.get(IK_PTR);
        fill(125,125,255);
        ellipse(target_point.x, target_point.y, 8,8);
        if(IK_PTR==target_points.size()-1){
          trj.constructParametrizedTraj(); 
          TRAJECTORY_NOT_GENERATED=false; 
          reached_points.clear();
          IK_PTR=0;
          trajectoryInformation.close();
         }
          ArrayList<Float> curpose =  IK1.cyclic_coordinate_descent(target_point);
          if(curpose.size() > 0){
            println("POSE FOUND!");
            if(TRAJECTORY_NOT_GENERATED){
              String s = "";
              //s += Integer.toString(IK_PTR);
              //s += ":";
              for(float f : curpose){s += (Float.toString(f) + ",");}
              println("POSE: " + s);
              trajectoryInformation.println(s);
              println("POSE ADDED!");
              trajectoryInformation.flush();
            }
            IK_PTR = (IK_PTR + 1)%(target_points.size());
            reached_points.add(target_point);
          }
      } else if(!TRAJECTORY_NOT_GENERATED){
        stroke(0);
        text("USING TRAJECTORY PLANNING...", 600, 20);
        DRAW_ROBOT_ARM(IK2);
        PVector ret = IK2.APPLY_FK();
        noStroke();
        ellipse(ret.x, ret.y, 10, 10);
        for(PVector p : reached_points){noStroke(); fill(0,0,255); ellipse(p.x, p.y, 8, 8);}
        String[] IO = loadStrings("trajInfo.txt");
        if(IK_PTR >= trj.quinticTraj.size()){IK_PTR = 0; reached_points.clear();}
        println("CURRENT POSE: ");
        for(float f : IK2.angles){
          print(f + " " );
        }
        println();
        ArrayList<TrajecMatrixSolver> cur = trj.quinticTraj.get(IK_PTR);
        ArrayList<Float> new_pose = new ArrayList<Float>();
        for(int i = 0; i < cur.size(); i++){
          double[] solution_vector = cur.get(i).solution_vector;
          float new_angle = 0;
          /*
          println("QUINTIC COEFFICIENTS: ");
          for(int j = 0; j < solution_vector.length; j++){
            print(solution_vector[j] + " " );
          }
          */
          println();
          for(int j = 0; j < solution_vector.length; j++){
            new_angle += (solution_vector[j]) * (pow(TIME_PTR,j));
          }
          new_pose.add(new_angle);
        }
        for(int i = new_pose.size()-1; i >= 0; i--){
          IK2.angles.set(i,new_pose.get(i));
        }
        TIME_PTR += TIME_DELTA;
        if(TIME_PTR >= 1){
            TIME_PTR = 0; 
            reached_points.add(target_points.get(IK_PTR)); 
            IK_PTR = (IK_PTR + 1)%target_points.size();
            println("-------------------------------------------------------------------------");
        }
        
      }
    } else {
      stroke(255);
      text("WITHOUT TRAJECTORY PLANNING...", 600, 20);
      DRAW_ROBOT_ARM(IK1);
      println();
      PVector ret = IK1.APPLY_FK();
      noStroke();
      ellipse(ret.x, ret.y, 10, 10);
      for(PVector p : reached_points){noStroke(); fill(0,0,255); ellipse(p.x, p.y, 8, 8);}
      println("LENGTH: " + target_points.size());
      PVector target_point = target_points.get(IK_PTR);
      fill(125,125,255);
      ellipse(target_point.x, target_point.y, 8,8);
      if(IK_PTR==target_points.size()-1){TRAJECTORY_NOT_GENERATED=false; reached_points.clear();}
       ArrayList<Float> curpose = new ArrayList<Float>();
       curpose = IK1.cyclic_coordinate_descent(target_point);
       if(curpose.size() > 0){
          IK_PTR = (IK_PTR + 1)%(target_points.size());
          reached_points.add(target_point);
        }
    }
  } else if(!TEST_RRT){
    if(USE_TRAJECTORY_PLANNING){
      stroke(255);
      if(TRAJECTORY_NOT_GENERATED){
        stroke(0);
        text("GENERATING TRAJECTORY...", 600, 20);
        DRAW_ROBOT_ARM(IK1);
        println();
        PVector ret = IK1.APPLY_FK();
        noStroke();
        ellipse(ret.x, ret.y, 10, 10);
        for(PVector p : reached_points){noStroke(); fill(0,0,255); ellipse(p.x, p.y, 8, 8);}
        PVector target_point = target_points.get(IK_PTR);
        fill(125,125,255);
        ellipse(target_point.x, target_point.y, 8,8);
        if(IK_PTR==target_points.size()-1){
          trj.constructParametrizedTraj(); 
          TRAJECTORY_NOT_GENERATED=false; 
          reached_points.clear();
          IK_PTR=0;
          trajectoryInformation.close();
         }
          ArrayList<Float> curpose =  IK1.cyclic_coordinate_descent(target_point);
          if(curpose.size() > 0){
            println("POSE FOUND!");
            if(TRAJECTORY_NOT_GENERATED){
              String s = "";
              //s += Integer.toString(IK_PTR);
              //s += ":";
              for(float f : curpose){s += (Float.toString(f) + ",");}
              println("POSE: " + s);
              trajectoryInformation.println(s);
              println("POSE ADDED!");
              trajectoryInformation.flush();
            }
            IK_PTR = (IK_PTR + 1)%(target_points.size());
            reached_points.add(target_point);
          }
      } else if(!TRAJECTORY_NOT_GENERATED){
        stroke(0);
        text("USING TRAJECTORY PLANNING...", 600, 20);
        DRAW_ROBOT_ARM(IK2);
        PVector ret = IK2.APPLY_FK();
        noStroke();
        ellipse(ret.x, ret.y, 10, 10);
        for(PVector p : reached_points){noStroke(); fill(0,0,255); ellipse(p.x, p.y, 8, 8);}
        String[] IO = loadStrings("trajInfo.txt");
        if(IK_PTR >= trj.quinticTraj.size()){IK_PTR = 0; reached_points.clear();}
        println("CURRENT POSE: ");
        for(float f : IK2.angles){
          print(f + " " );
        }
        println();
        ArrayList<TrajecMatrixSolver> cur = trj.quinticTraj.get(IK_PTR);
        ArrayList<Float> new_pose = new ArrayList<Float>();
        for(int i = 0; i < cur.size(); i++){
          double[] solution_vector = cur.get(i).solution_vector;
          float new_angle = 0;
          /*
          println("QUINTIC COEFFICIENTS: ");
          for(int j = 0; j < solution_vector.length; j++){
            print(solution_vector[j] + " " );
          }
          */
          println();
          for(int j = 0; j < solution_vector.length; j++){
            new_angle += (solution_vector[j]) * (pow(TIME_PTR,j));
          }
          new_pose.add(new_angle);
        }
        for(int i = new_pose.size()-1; i >= 0; i--){
          IK2.angles.set(i,new_pose.get(i));
        }
        TIME_PTR += TIME_DELTA;
        if(TIME_PTR >= 1){
            TIME_PTR = 0; 
            reached_points.add(target_points.get(IK_PTR)); 
            IK_PTR = (IK_PTR + 1)%target_points.size();
            println("-------------------------------------------------------------------------");
        }
        
      }
    } else {
      stroke(255);
      text("WITHOUT TRAJECTORY PLANNING...", 600, 20);
      DRAW_ROBOT_ARM(IK1);
      println();
      PVector ret = IK1.APPLY_FK();
      noStroke();
      ellipse(ret.x, ret.y, 10, 10);
      for(PVector p : reached_points){noStroke(); fill(0,0,255); ellipse(p.x, p.y, 8, 8);}
      println("LENGTH: " + target_points.size());
      PVector target_point = target_points.get(IK_PTR);
      fill(125,125,255);
      ellipse(target_point.x, target_point.y, 8,8);
      if(IK_PTR==target_points.size()-1){TRAJECTORY_NOT_GENERATED=false; reached_points.clear();}
       ArrayList<Float> curpose = new ArrayList<Float>();
       curpose = IK1.cyclic_coordinate_descent(target_point);
       if(curpose.size() > 0){
          IK_PTR = (IK_PTR + 1)%(target_points.size());
          reached_points.add(target_point);
        }
    }
  }
  
  
  
}

void keyPressed(){
  if(key == ENTER){
    START_RECORDING = true;
  }
}
