import java.io.*;
import java.util.*;

class PDist{
  PVector p;
  float dist;
  public PDist(PVector p, float dist){
    this.p = p;
    this.dist = dist;
  }
}


class PQComparator implements Comparator<PDist>{
  public int compare(PDist one, PDist two){
     if(one.dist >= two.dist){
       return 1;
     }
     return 0;
  }
}

class PQTwo implements Comparator<PVector>{
  public int compare(PVector one, PVector two){
    if(one.x > two.x){
      return 1;
    } else if(one.x < two.x){
      return -1;
    } else {
      if(one.y<two.y){
        return 1;
      } else if(one.y>two.y){
        return -1;
      } else {
        return 0;
      }
    }
  }
}


class RRT{
   HaltonSampler hq;
   PVector seed;
   float dq;
   float MAX_ITER;
   float INTERNAL_COUNTER;
   HashMap<PVector, ArrayList<GType> > graph;
   HashMap<PVector, Boolean> hset = new HashMap<PVector, Boolean>();
   ArrayList<PVector> seen_space = new ArrayList<PVector>();
   ArrayList<Location> obstacle_space= new ArrayList<Location>();
   public static final int K = 10;
   //we want to use some dispersion heuristic, so 
   //normal softmax won't work because the elements in vals are very big
   public ArrayList<Float> softmax(ArrayList<Float> vals){
      float sm = 0;
      ArrayList<Float> n = new ArrayList<Float>();
      for(int i = 0; i < vals.size(); i++){
         sm += vals.get(i);
      }
      for(int i = 0; i < vals.size(); i++){
         float nw = (float)(vals.get(i))/(float)(sm);
         n.add(nw);
      }
      return n;
  }
   public boolean vec_contains(PVector v){
     if(hset.containsKey(v)){return true;}return false;
   }
   
   public RRT(PVector seed, float dq, float MAX_ITER){
     this.seed = seed;
     this.dq = dq;
     this.MAX_ITER = MAX_ITER;
     graph = new HashMap<PVector, ArrayList<GType> >();
     seen_space.add(seed);
     this.INTERNAL_COUNTER = 0;
     hq = new HaltonSampler(1400,900);
     hq.genHalton(4000);
   }
   //this method is kind of computationally heavy
   public PVector nearest_point(PVector comp){
     float mdist  = 1000000000;
     PVector cur = new PVector(0,0);
     for(PVector v : seen_space){
       mdist = min(mdist, dist(comp.x,comp.y,v.x,v.y));
       if(mdist == dist(comp.x,comp.y,v.x,v.y)){
         mdist = dist(comp.x,comp.y,v.x,v.y);
         cur = new PVector(v.x,v.y);
       }
     }
     return cur;
   }
   
   //returns next node to expand our RRT on some node
   //the idea is that the max of the max dists in the KNN should approximate the voronoi region areas (i.e. nodes that are far away from other nodes
   //should have a higher probability of being expanded)
   public int k_nearest_neighbors(int K){
     ArrayList<PVector> pq = new ArrayList<PVector>();
     ArrayList<Float> MAX_DISTS = new ArrayList<Float>();
     for(int i = 0; i < seen_space.size(); i++){
       for(int j = 0; j < seen_space.size(); j++){
         if(i==j){continue;}
         pq.add(seen_space.get(j));
       }
       int idx = 0;
       float DIST = 0;
       ArrayList<PDist> augmented_dist = new ArrayList<PDist>();
       for(int j = 0; j < pq.size(); j++){
         augmented_dist.add(new PDist(pq.get(j), dist(pq.get(j).x,pq.get(j).y,seen_space.get(i).x,seen_space.get(i).y)) );
       }
       Collections.sort(augmented_dist, new PQComparator());
       DIST = augmented_dist.get(K).dist;
       MAX_DISTS.add( (float)(DIST) );
       pq.clear();
     }

     ArrayList<Float> nxt = softmax(MAX_DISTS);
     ArrayList<PVector> sample = new ArrayList<PVector>();
     for(int i = 0; i < nxt.size(); i++){
       sample.add(new PVector(nxt.get(i),i));
     }
     Collections.sort(sample, new PQTwo());
     int idx = (int)sample.get(sample.size()-1).y;

     for(int i = 0; i < sample.size(); i++){
       float f = random(0,1);
       if(sample.get(i).x >= f){
         idx = (int)sample.get(i).y;
         break;
       }
     }
     
     return idx;
   }
   
   //given a new node we want to add to our RRT and the node it is connected to
   //check if the edge overlaps with any part of the obstacles in our space (divide space into C(free) and C(obs) )
   //if so return FALSE, otherwise return TRUE
   //we'll just sample the points along the line and detect if some point exists within the convex polygon produced by some obstacle.
   public boolean IN_FREE_SPACE(PVector new_segment){
     double delta = 0.01;
     for(double itr = 0; itr <= 1; itr += delta){
       //check if some point along a segment lies in some convex hull
     }
     return true;
   }
   
   
   //this is basically deprecated; halton sequence works considerably better for what we need
   public boolean rrtVoronoiBias(){
     println("INTERNAL COUNTER: " + this.INTERNAL_COUNTER);
       float seed = random(0,1);
       if(this.INTERNAL_COUNTER <= this.MAX_ITER/2){
         PVector rp = new PVector( random(0,1400), random(0,900) );
         PVector closest = nearest_point(rp);
         float ang = atan( (float)(rp.y-closest.y)/(float)(rp.x-closest.x) );
         PVector new_pt = new PVector(closest.x + (dq*cos(ang)), closest.y + (dq*sin(ang)) );
         if(graph.containsKey(closest)){
           ArrayList<GType> tmp = graph.get(closest);
           tmp.add( new GType(new_pt, dist(closest.x,closest.y,new_pt.x,new_pt.y) ));
           graph.put(closest,tmp);
           println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
         } else {
           ArrayList<GType> tmp = new ArrayList<GType>();
           tmp.add( new GType(new_pt, dist(closest.x,closest.y,new_pt.x,new_pt.y)) );
           println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
           graph.put(closest, tmp);
         }
         /*
          if(graph.containsKey(new_pt)){
           ArrayList<GType> cur = graph.get(new_pt);
           cur.add(new GType(closest, dist(closest.x,closest.y,new_pt.x,new_pt.y)));
           graph.put(new_pt,cur);
         } else {
           ArrayList<GType> cur = new ArrayList<GType>();
           cur.add(new GType(closest, dist(closest.x,closest.y,new_pt.x,new_pt.y)));
           graph.put(new_pt,cur);
         }
         */
         seen_space.add(new_pt);
         this.INTERNAL_COUNTER++;
         return true;
       } else if(this.INTERNAL_COUNTER > this.MAX_ITER/2 && this.INTERNAL_COUNTER < this.MAX_ITER){
         int nxt = k_nearest_neighbors(K);
         println("INDEX: " + nxt);//issue is we are choosing the same node to expand upon too many times
         PVector corres = seen_space.get(nxt);
         PVector rp = new PVector(random(0,1400),random(0,900));
         //PVector closest = nearest_point(rp);
         float ang = atan( (float)(rp.y-corres.y)/(float)(rp.x-corres.x) );
         PVector new_pt = new PVector(corres.x + (dq*(cos(ang))), corres.y + (dq*(sin(ang))) ) ;
         if(graph.containsKey(corres)){
           ArrayList<GType> tmp = graph.get(corres);
           tmp.add( new GType(new_pt, dist(corres.x,corres.y,new_pt.x,new_pt.y) ));
           graph.put(corres,tmp);
           //println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
         } else {
           ArrayList<GType> tmp = new ArrayList<GType>();
           tmp.add( new GType(new_pt, dist(corres.x,corres.y,new_pt.x,new_pt.y)) );
           //println(closest.x + " " + closest.y + " " + new_pt.x + " " + new_pt.y);
           graph.put(corres, tmp);
         }
         /*
         if(graph.containsKey(new_pt)){
           ArrayList<GType> cur = graph.get(new_pt);
           cur.add(new GType(corres, dist(corres.x,corres.y,new_pt.x,new_pt.y)));
           graph.put(new_pt,cur);
         } else {
           ArrayList<GType> cur = new ArrayList<GType>();
           cur.add(new GType(corres, dist(corres.x,corres.y,new_pt.x,new_pt.y)));
           graph.put(new_pt,cur);
         }
         */
         seen_space.add(new_pt);
         this.INTERNAL_COUNTER++;
         return true;
      } else {
        return false;
      }
   }
   
   public boolean rrtHalton(){
     if(this.INTERNAL_COUNTER <= this.MAX_ITER){
       PVector loc = hq.HALTON_POINTS.get( (int)(this.INTERNAL_COUNTER) );
       PVector nearest = nearest_point(loc);
       float ang = atan( (float)(loc.y-nearest.y)/(float)(loc.x-nearest.x) );
       PVector new_pt = new PVector(nearest.x + (dq*cos(ang)), nearest.y + (dq*sin(ang)) );
       
     }
     return false;
   }
   
   
   
   public void displayRRT(PVector state){
     println("----------------------------------------------------------");
     LinkedList<PVector> qp = new LinkedList<PVector>();
     qp.add(state);
     //for(PVector p : seen_space){fill(255,0,0); circle(p.x,p.y,10);}
     ArrayList<GType> see = graph.get(state);
     //for(PVector p : see){println(p.x + " " + p.y);}
     circle(state.x,state.y,10);
     stroke(0,0,255);
     fill(0,0,255);
     while(qp.size() > 0){
       if(qp.size()==0){break;}
       PVector nxt = qp.poll();
       //println("SIZE: " + qp.size());
       ArrayList<GType> adj = new ArrayList<GType>();
       if(graph.containsKey(nxt)){
          adj = graph.get(nxt);
       } else {
         continue;
       }
       //println("CURRENT POSITION: " + nxt.x + " " + nxt.y);
       for(GType mvec : adj){
         if(!vec_contains(mvec.evec)){
           //println("ADDING POINT: " + mvec.evec.x + " " + mvec.evec.y);
           circle(mvec.evec.x,mvec.evec.y,10);
           //println("LINE DRAWN: " + nxt.x + " " + nxt.y + " " + mvec.evec.x +  " " + mvec.evec.y);
           line(nxt.x,nxt.y,mvec.evec.x,mvec.evec.y);
           hset.put(mvec.evec,true);
           qp.add(mvec.evec);
         } else {
           //println("ALREADY SEEN");
         }
       }
     }
     println("----------------------------------------------------------------------------");
   }
   
   public boolean detectObstacle(PVector one, PVector two){
     return true;
   }
   
   public void reset(){
     hset.clear();
   }
}
