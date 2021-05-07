import java.util.*;
import java.io.*;

TrajectoryPlanner trj;
PathPlanner p;
GaussianElimination g;
ForwardKinematics FK;
InverseKinematics IK;
ArrayList<ArmSegment> robotArm = new ArrayList<ArmSegment>();
MatrixOps matPipeline;


class MatrixOps{
  public float dp(float[] one, float[] two){
    float r = 0;
    for(int i = 0; i < one.length; i++){
      r += one[i]*two[i];
    }
    return r;
  }
  public float[][] slice_matrix(float[][] orig, int tx, int bx, int ty, int by){
    float[][] ret = new float[bx-tx][by-ty];
    int x = 0, y = 0;
    for(int i = tx; i < bx; i++){
      for(int j = ty; j < by; j++){
         ret[x][y] = orig[i][j];
         y++;
      }
      x++;
      y=0;
    }
    return ret;
  }
  public void mutate_matrix(float[][] orig, float[][] add, int tx, int bx, int ty, int by){
    int x = 0, y = 0;
    for(int i = tx; i < bx; i++){
      for(int j = ty; j < by; j++){
        orig[i][j] = add[x][y];
        y++;
      }
      x++;
      y=0;
    }
  }
  
  public float[][] matrixAddition(float[][] A, float[][] B){
    float[][] C = new float[A.length][A[0].length];
    for(int i = 0; i < A.length; i++){
      for(int j = 0; j < A[0].length; j++){
          C[i][j] = A[i][j] + B[i][j];
      }
    }
    return C;
  }
  
  public float[][] matrixSubtraction(float[][] A, float[][] B){
    float[][] C = new float[A.length][A[0].length];
    for(int i = 0; i < C.length; i++){
      for(int j = 0; j < C[0].length; i++){
        C[i][j] = A[i][j] - B[i][j];
      }
    }
    return C;
  }
  
  public float[][] strassen(float[][] a, float[][] b){
  if(a.length == 2 && b.length == 2){
    float[][] tmp = new float[2][2];
    tmp[0][0] = a[0][0] * b[0][0]  + a[0][1] * b[1][0];
    tmp[0][1] = a[0][0] * b[1][0] + a[0][1] * b[1][1];
    tmp[1][0] = a[1][0] * b[0][0] + a[1][1] * b[1][0];
    tmp[1][1] = a[1][0] * b[1][0] + a[1][1] * b[1][1];
    return tmp;
  }
  float[][] one = slice_matrix(a, 0, a.length/2, 0, a.length/2);
  float[][] two = slice_matrix(a, 0, a.length/2, a.length/2, a.length);
  float[][] three = slice_matrix(a, a.length/2, a.length, 0, a.length/2);
  float[][] four = slice_matrix(a, a.length/2, a.length, a.length/2, a.length);
  float[][] five = slice_matrix(b, 0, b.length/2, 0, b.length/2);
  float[][] six = slice_matrix(b, 0, b.length/2, b.length/2, b.length);
  float[][] seven = slice_matrix(b, b.length/2, b.length, 0, b.length/2);
  float[][] eight = slice_matrix(b, b.length/2, b.length, b.length/2, b.length);
  
  float[][] bc_one = strassen(matrixAddition(one,two),matrixAddition(five,six));
  float[][] bc_two = strassen(matrixAddition(three,four),five);
  float[][] bc_three = strassen(one, matrixSubtraction(five,seven));
  float[][] bc_four = strassen(three, matrixSubtraction(seven,five));
  float[][] bc_five = strassen(matrixAddition(one,two),eight);
  float[][] bc_six = strassen(matrixSubtraction(three,one),matrixAddition(five,six));
  float[][] bc_seven = strassen(matrixSubtraction(one,two),matrixAddition(seven,eight));
  
  
   float[][] C11 = matrixAddition(matrixSubtraction(matrixAddition(bc_one, bc_four),bc_five),bc_seven);
   float[][] C12 = matrixAddition(bc_three, bc_five);
   float[][] C21 = matrixAddition(bc_two, bc_four);
   float[][] C22 = matrixAddition(matrixAddition(matrixSubtraction(bc_one, bc_two),bc_three),bc_six);
   
   float[][] ret = new float[a.length][b[0].length];
   mutate_matrix(ret, C11, 0, a.length/2, 0, b[0].length/2);
   mutate_matrix(ret, C12, a.length/2, a.length, b[0].length/2, b[0].length);
   mutate_matrix(ret, C21, a.length/2, a.length, 0, b[0].length/2);
   mutate_matrix(ret, C22, a.length/2, a.length, b[0].length/2, b[0].length);
   return ret; 
 }
}


class ForwardKinematics{
  ArrayList<ArmSegment> kinematic_chain;
  ArrayList<float[][]> KINEMATIC_MATRICES;
  public ForwardKinematics(ArrayList<ArmSegment> kinematic_chain){
    this.kinematic_chain = kinematic_chain;
    this.KINEMATIC_MATRICES = new ArrayList<float[][]>();
  }
  
  public void ASSIGN_DENAVIT_HARTENBERG_MATRICES(){
    for(int i = 0; i < kinematic_chain.size(); i++){
      float[][] m = new float[4][4];
      m[0][0] = cos(kinematic_chain.get(i).angle);
      m[1][0] = sin(kinematic_chain.get(i).angle);
      m[0][1] = -sin(kinematic_chain.get(i).angle);
      m[1][1] = cos(kinematic_chain.get(i).angle);
      this.KINEMATIC_MATRICES.add(m);
    }
  }
}

class InverseKinematics{
  PVector end_effector_position;
  public InverseKinematics(PVector end_effector_position){
    this.end_effector_position = end_effector_position;
  }
  public ArrayList<Float> cyclic_coordinate_descent(){
    
    return new ArrayList<Float>();
  }
}

class ArmSegment{
  PVector start;
  PVector end;
  float angle;
  public ArmSegment(PVector one, PVector two){
    this.start = one;
    this.end = two;
    this.angle = atan( (two.y-one.y)/(two.x-one.x) );
  }
}

void GENERATE_CUSTOM_ARM(int LENGTH){
  PVector cur = new PVector( random(100,400), random(100,400) );
  for(int i = 0; i < LENGTH; i++){
    PVector nxt = new PVector(random(cur.x + 10,400), random(cur.y + 10,400) );
    println(cur.x + " " + cur.y + " " + nxt.x + " " + nxt.y);
    robotArm.add(new ArmSegment(cur, nxt) );
    cur = nxt;
  }
}

void setup(){
  size(800,800);
  matPipeline = new MatrixOps();
  float[][] A = { {1, 2, 3, 4}, {5, 6, 7, 8}, {1, 2, 3, 4}, {5, 6, 7, 8} };
  float[][] B = { {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1} };
  
  float[][] C = matPipeline.strassen(A, B);
  for(int i = 0; i < C.length; i++){
    for(int j = 0; j < C[0].length; j++){
      print(C[i][j] + " ");
    }
    println();
  }
  
    
  GENERATE_CUSTOM_ARM(5);//will modify this to generate "better" kinematic chains
}

void draw(){
  background(0);
  for(ArmSegment arm : robotArm){
    stroke(0,255,0);
    line(arm.start.x, arm.start.y, arm.end.x, arm.end.y);
    fill(255,0,0);
    noStroke();
    ellipse(arm.start.x,arm.start.y,8,8);
    ellipse(arm.end.x, arm.end.y, 8, 8);
  }
}
