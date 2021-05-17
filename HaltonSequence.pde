
import java.io.*;
import java.util.*;

class HaltonSampler{
  int sz_x, sz_y;
  ArrayList<PVector> HALTON_POINTS;
  public HaltonSampler(int sz_x, int sz_y){
    this.sz_x = sz_x;
    this.sz_y = sz_y;
    HALTON_POINTS = new ArrayList<PVector>();
  }
  
  public float generateHalton(int i, int b) {
    float r = 0;
    float f = 1;
    while(i > 0) {
      f = f/b;
      r = r + f*(i % b);
      i = i/b;
    }
    return r;
  }
  
  ArrayList<PVector> genHalton(int NUM_ITER){
    ArrayList<PVector> p = new ArrayList<PVector>();
    for(int i = 0; i < NUM_ITER; i ++) {
      float X = generateHalton(i,2);
      float Y = generateHalton(i,3);
      System.out.println(X + ", " + Y);
       p.add(new PVector(X*sz_x, Y*sz_y)); 
    }
    this.HALTON_POINTS = p;
    return p;
  }
}
