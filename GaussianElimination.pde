import java.io.*;
import java.util.*;

class PivotType{
  float pivot;
  int position;
  public PivotType(float pivot, int position){
    this.pivot = pivot;
    this.position = position;
  }
}


class RowType{
   float[] r;
   PivotType pivot;
   public RowType(float[] r){
     this.r = r;
     for(int i = 0; i < r.length; i++){if(r[i] != 0){pivot = new PivotType(r[i],i);}
   }
}



class PivotComparator implements Comparator<RowType>{
  public int compare(RowType one, RowType two){
    if(one.pivot > two.pivot && one.position < two.position){
      return 1;
    }
    return 0;
  }
}


class GaussianElimination{
  RowType[] WRAPPER_MATRIX;
  float[][] M;
  float[] B;
  
  public GaussianElimination(float[][] M, float[] B){
    this.M = M;
    this.B = B;
    WRAPPER_MATRIX = new RowType[M.length]
    for(int i = 0; i < M.length; i++){
      WRAPPER_MATRIX[i] = new RowType(M[i]);
    }
  }
  
  public float[] itersolve(){
    
  }
  
}
