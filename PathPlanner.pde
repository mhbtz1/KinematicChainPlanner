
//iterative deepening A*
import java.util.*;



//the nice thing about using RRTs for making continuous stuff discrete is that the graph structure
//already encodes the obstacle space for us.

//this uses IDA* in order to plan a path GIVEN a valid RRT; although the RRT need not reach the goal necessarily (we use Bezier splines to refine some stuff)


class IDAComparator implements Comparator<GType>{
  public int compare(GType one, GType two){
     if(one.COST_TO_REACH + one.HEURISTIC_COST < two.COST_TO_REACH + two.HEURISTIC_COST){
       return 1;
     } else {
       return 0;
     }
  }
}

class PathPlanner{
    HashMap<PVector, Boolean> vis = new HashMap<PVector, Boolean>();
    HashMap<PVector, ArrayList<GType> > graph = new HashMap<PVector, ArrayList<GType> >();
    HashMap<PVector, Float> intermediate_results = new HashMap<PVector, Float>();
    //stores a list of integers which encode the positions they represent
    ArrayList<PVector> ASTAR_PATH;
    PVector start,goal;
    HashMap<PVector, PVector> parent_pointers = new HashMap<PVector, PVector>();
    ArrayList<PVector> classPath = new ArrayList<PVector>();
    public int hashCode(float x, float y){
      int hash = (int)( 1400*(y) + x );
      //print("HASH:" + hash); 
      return hash;
    }
    public float euclidean_heuristic_function(Location one, Location two){
      float f = dist(one.x,one.y,two.x,two.y);
      return f;
    }
    

    public PathPlanner(HashMap<PVector, ArrayList<GType> > graph, Location start, Location goal){
      this.start = new PVector(start.x,start.y);
      this.goal = new PVector(goal.x,goal.y);
      this.parent_pointers = new HashMap<PVector, PVector>();
      this.ASTAR_PATH = new ArrayList<PVector>();
      this.graph = graph;
    }
    //start off with heuristic value = 0
    public ArrayList<Location> IDA_Algo(Location start){
      return new ArrayList<Location>(); 
    }
    
    
    public ArrayList<PVector> backtracking(PVector fin){
      ArrayList<PVector> res = new ArrayList<PVector>();
      while(fin != null){
        println(fin.x + "  " + fin.y);
        res.add(fin);
        fin = parent_pointers.get(fin);
      }
      return res;
    }
    /*
    public ArrayList<Integer> backtrack(int end){
      println("START BACKTRACKING");
      ArrayList<Integer> res = new ArrayList<Integer>();
      while(true){
        println("CURRENT POSITION: " + end);
        res.add(end);
        if(!parent_pointers.containsKey(end)){break;}
        end= parent_pointers.get(end);
      }
      return res;
    }
    */
    
    public ArrayList<Location> iterative_deepening_astar(){
      return new ArrayList<Location>();
    }
    

    public ArrayList<PVector> IDA(){
       PriorityQueue<GType> pq = new PriorityQueue(new IDAComparator());
       GType seed = new GType(start, 0, dist(start.x,start.y,goal.x,goal.y));
       pq.add(seed);
       ArrayList<PVector> path = new ArrayList<PVector>();
       while(pq.size() > 0){
         GType frnt = pq.poll();
         float f_cost = frnt.COST_TO_REACH;
         if(!graph.containsKey(frnt.evec)){continue;}
         ArrayList<GType> adj = graph.get(frnt.evec);
         for(GType p : adj){
           if(intermediate_results.containsKey(p.evec)){continue;}
           float new_f = f_cost + dist(p.evec.x,p.evec.y,goal.x,goal.y);
           float new_g = dist(p.evec.x,p.evec.y,goal.x,goal.y);//heuristic for distance
           if(!intermediate_results.containsKey(p.evec)){
             intermediate_results.put(p.evec, new_f + new_g);
             GType g = new GType(p.evec, new_f, new_g);
             parent_pointers.put(p.evec, frnt.evec);
             pq.add(g);
           } else {
             if(Math.min(intermediate_results.get(p.evec), new_f + new_g) == new_f + new_g){
               intermediate_results.put(p.evec, new_f + new_g);
               GType ng = new GType(p.evec, new_f, new_g);
               pq.add(ng);
               parent_pointers.put(p.evec, frnt.evec);
             }
           }
         }
      }
       println("TERMINATE");
       path = backtracking(goal);
       this.classPath = path;
       println("OPTIMAL PATH LENGTH: " + intermediate_results.get(goal) );
       return path;
    }
    
    public ArrayList<PVector> augment_waypoints(float STEP){
      for(PVector p : this.classPath){
        println(p.x + " " + p.y);
      }
      println("-----------------------------------------------------------");
      if(GOAL_STATE_CHANGED){
        controlPoints = createWriter("controlPoints.txt");
      }
      ArrayList<PVector> RRT_waypoints = new ArrayList<PVector>();
      Collections.reverse(this.classPath);
      for(int i = 0; i < classPath.size()-1; i++){
        float total_dist = sqrt(dist(classPath.get(i).x,classPath.get(i).y,classPath.get(i+1).x,classPath.get(i+1).y));
        float slope = (float)(classPath.get(i).y-classPath.get(i+1).y)/(float)(classPath.get(i).x-classPath.get(i+1).x);
        float angle = atan(slope)+(PI/(float)(2));
        
        PVector param = new PVector(classPath.get(i).x,classPath.get(i).y);
        PVector cur_point = new PVector(classPath.get(i).x,classPath.get(i).y);
        cur_point.sub(classPath.get(i+1));
        cur_point.x *= -1;
        cur_point.y *= -1;
        float VL = 0;
        println(cur_point.x + ":" + cur_point.y);
        float eps = 0.02;
        while(VL <= 0.3){
          println("J VALUE: " + VL);
          if(fillPathInfo){
            controlPoints.println(param.x + ":" + param.y);
          }
          RRT_waypoints.add(param);
          param = new PVector( (float)(param.x) + (float)(cur_point.x*VL), (float)(param.y) + (float)(cur_point.y*VL) );
          VL += STEP; //when i do j += STEP in this line, the RRT flips out for some reason
        }     
      }
      if(fillPathInfo){
        for(int i = 0; i < MotionProfiler.ITER; i++){
          controlPoints.println(goal_loc.x + ":" + goal_loc.y);
        }
      }
      controlPoints.flush();
      controlPoints.close();
      println("RRT WAYPOINTS SIZE: " + RRT_waypoints.size());
      return RRT_waypoints;
    }   
}
