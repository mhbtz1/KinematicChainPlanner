//cubic bezier curve can be represented as:
//B(x) = (1-x)^3 * (P0) + 3x(1-x)^2 * (P1) + 3x^2(1-x)*(P2) + x^3 * (P3)

public static final int ANGLE_SHIFTING_FACTOR = 0;
public static boolean SHOULD_DRAW = false;
class Point{
  float x, y;
  public Point(float x, float y){
    this.x = x;
    this.y = y;
  }
      public boolean equals(Location candidate){
        return candidate.x==x && candidate.y==y;
    }
    public double distance(Point one, Point two){
        return (Math.ceil(Math.sqrt(Math.pow( (one.x-two.x),2) + Math.pow( (one.y-two.y),2))));
    }
    public Point add(Point addend){
        return new Point(this.x+addend.x,this.y+addend.y);
    }
    public Point subtract(Point subtractend){
        return new Point(this.y-subtractend.y, this.x - subtractend.x);
    }
    public double magnitude(){
        return Math.sqrt( (this.x * this.x) + (this.y * this.y) );
    }
    public boolean is_greater_or_equal(Point one, Point two){
        return ( one.x >= two.x && one.y >= two.y);
    }
}

//objective: create a smooth profile of cubic bezier curves with 4 control points that some point robot can follow a path along.
//let P(n,k) be the nth control point (n <= 3) for the kth bezier curve:

//for n=0 and n=3, it is trivial, these points are given to be one of the pre-existing waypoints.

//for n=2, P(2,i) = 2*K(i+1) - P(1,i+1) for 0 <= i <= n-2
// P(2,n-1) = 0.5*(K(n) + P(1,n-1))



void thomas_algorithm(){
  
}
float sigmoid_spline(float x){
  return (float)( ((20)/(1+pow((float)(Math.E),0.2*x)))* 20) + 300;
}


//cubic bezier profile
class BezierProfile{
  float init_x, init_y;
  float ref1_x, ref1_y;
  float ref2_x, ref2_y;
  float final_x, final_y;
  public BezierProfile(float init_x, float init_y, float ref1_x,float ref1_y, float ref2_x, float ref2_y, float final_x, float final_y){
    this.init_x = init_x;
    this.init_y = init_y;
    this.ref1_x = ref1_x;
    this.ref1_y = ref1_y;
    this.ref2_x = ref2_x;
    this.ref2_y = ref2_y;
    this.final_x = final_x;
    this.final_y = final_y;
  }
  
  //takes some input [0,1]
  float return_x(float x){
  double v = (init_x * Math.pow( (1-x), 3)) +(3*x * (1-x) * (1-x) * ref1_x) + (3*x*x * (1-x) * ref2_x)+ (Math.pow(x,3) * final_x);
  return (float)(v);
}
float return_y(float x){
   double v = (init_y * Math.pow( (1-x), 3)) +(3*x * (1-x)*(1-x) * ref1_y) + (3*x*x * (1-x) * ref2_y)+ (Math.pow(x,3) * final_y);
  return (float)(v);
}
}

class MotionProfiler {
  public ArrayList<BezierProfile> bez;
  public ArrayList<Location> waypoints;
  public ArrayList<Location> true_waypoints;
  public HashMap<Integer, Float> angle_profile;
  public HashMap<Integer, Location> velocity_profile;
  public static final int ITER = 20;
  
  public MotionProfiler(ArrayList<BezierProfile> bez, ArrayList<Location> waypoints){
    this.bez = bez;
    this.waypoints = waypoints;
    this.true_waypoints = new ArrayList<Location>();
  }
  public MotionProfiler(){
    this.bez = new ArrayList<BezierProfile>();
    this.waypoints = new ArrayList<Location>();
    this.true_waypoints = new ArrayList<Location>();
    this.velocity_profile = new HashMap<Integer, Location>();
    this.angle_profile = new HashMap<Integer, Float>();
  }
  
  public ArrayList<Location> parseFile(){
    ArrayList<Location> ans = new ArrayList<Location>();
    String[] r = loadStrings("controlPoints.txt");
    println("SIZE: "  + r.length);
    for(String line : r){
       double one = Double.parseDouble(line.substring(0, line.indexOf(":")));
       double two = Double.parseDouble(line.substring(line.indexOf(":")+1));
       ans.add(new Location( (float)(one), (float)(two) ) );
       println("POINT: " + one + " " + two);
    }
    return ans;
  }

 public void constructVelocityMap(){
        System.out.println("PATH SIZE: " + true_waypoints.size());
        //PREDICATED ON SOME SEED STARTING POSITION
        Location cur_pos= new Location(200,300);
        for(int i = 0; i < true_waypoints.size(); i++){
            Location vel = true_waypoints.get(i).subtract(cur_pos);
            vel.x /= (double)(vel.magnitude()/(double)100);
            vel.y /= (double)(vel.magnitude()/(double)100);
            velocity_profile.put(i,vel);
            System.out.println("WAYPOINT INDEX: " + i  + "VELOCITY: " + vel.x + " " + vel.y);
            cur_pos= true_waypoints.get(i);
        }
    }
    public boolean loc_contains(ArrayList<Location> targ, Location tst){
        for(Location l : targ){
          if(l.equals(tst)){
            return true;
          }
        }
        return false;
      }
    
    /*
    public ArrayList<Location> parseFile(boolean pass){
        ArrayList<Integer> maintain_hashcodes = new ArrayList<Integer>();
        ArrayList<Location> ans = new ArrayList<Location>();
        //String[] r = loadStrings("controlPoints.txt");
        ArrayList<Location> remove_duplicates = new ArrayList<Location>();
        try{
            String s = Filesystem.getDeployDirectory() + "/controlPathBehavior.txt";
            BufferedReader r = new BufferedReader(new FileReader(s) );
            String inp = "";
            while( (inp = r.readLine() ) != null){
            double one = Double.parseDouble(inp.substring(0, inp.indexOf(":")));
            double two = Double.parseDouble(inp.substring(inp.indexOf(":")+1));
            //System.out.println(one + ":" + two);
            int hc = (new Location( (float)(one), (float)(two) ) ).hashCode();
            if(loc_contains(remove_duplicates, new Location( (float)(one), (float)(two) ) ) ){continue;}
                remove_duplicates.add(new Location( (float)(one), (float)(two) ) );
                ans.add(new Location( (float)(one), (float)(two) ) );
            //println("POINT: " + one + " " + two);
            }
            r.close();
        } catch(IOException e){
            e.printStackTrace();
        }
        return new ArrayList<Location>(remove_duplicates);
    }
    */
    
    //check out some weird stalling points, the robot is following the path, yet at waypoint 26 it gets stuck on a straight path
    public void iterate_profiles(){
        //if(waypoints.size() == 0){
          waypoints = parseFile();
          System.out.println("SIZE: " + waypoints.size());
          for(int i = 0; i < waypoints.size()-ITER; i+= ITER){
              //println(waypoints.get(i).x + " " + waypoints.get(i).y);
              BezierProfile b = new BezierProfile(waypoints.get(i).x,waypoints.get(i).y,waypoints.get(i+ITER/4).x,waypoints.get(i+ITER/4).y,waypoints.get(i+ITER/2).x,waypoints.get(i+ITER/2).y,waypoints.get(i+ITER).x,waypoints.get(i+ITER).y);
              bez.add(b);
            }
        int idx = 0;
        for(BezierProfile b : bez){
          for(float j = 0; j <= 1.0; j += 0.04){
            System.out.println("POINT: " + idx  +  " " + b.return_x(j) + " " + b.return_y(j));
            fill(0,255,0);
            ellipse(b.return_x(j), b.return_y(j), 5, 5);
            true_waypoints.add(new Location(b.return_x(j), b.return_y(j)));
            //bezierPoints.println(b.return_x(j) + ":" + b.return_y(j));
            ++idx;
          }
        }

        constructAngleMap();
        constructVelocityMap();
        //ANGLE_FILE_HAS_BEEN_INITIALIZED=true;
    }
    /*
    public void constructAccelerationMap(){
        if(path == null){
            return;
        }
        for(int i = 0; i < path.size()-1; i++){
            Location acc = velocity_profile.get(i+1).subtract(velocity_profile.get(i));
            acceleration_profile.put(i, acc);
        }
    }
    */
    public void constructAngleMap(){
        //try{ 
          //  String s = Filesystem.getDeployDirectory() + "/the_angles.txt";
            //BufferedWriter  r= new BufferedWriter( new FileWriter( s ) );
            for(int i = 0; i < true_waypoints.size() - 1; i++){
                double avl = Math.atan( (double)( true_waypoints.get(i+1).y - true_waypoints.get(i).y)/(double)(true_waypoints.get(i+1).x - true_waypoints.get(i).x) ) * ((double)(180)/(double)(Math.PI));
                double mult = 1.0;
             
                angle_profile.put(i, (float)( avl + (ANGLE_SHIFTING_FACTOR) ) );
                System.out.println("ANGLE AT TIME: " + i +  " IS: "  + avl);
                //r.write(Double.toString(avl).toCharArray());
                System.out.println("WAYPOINT INDEX: " + i + "ANGLE: " + ( -1 * (avl+ANGLE_SHIFTING_FACTOR) ) );
            }
            //r.close();
        //} catch(Exception e){
        //    e.printStackTrace();
        //}
        System.out.println("ANGLES HAVE BEEN ASSIGNED");
    }
  
  public void vectorized_path(){
     for(int i = 0; i < true_waypoints.size()-1; i++){
       float dx = velocity_profile.get(i).x * cos(radians((float)(angle_profile.get(i))));
       float dy = velocity_profile.get(i).y* sin(radians((float)(angle_profile.get(i))));
       line(true_waypoints.get(i).x,true_waypoints.get(i).y,true_waypoints.get(i).x+(dx),true_waypoints.get(i).y+(dy));
     }
     SHOULD_DRAW=true;
  }
  
  
  
}

 
