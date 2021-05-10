import java.io.*;
import java.util.*;
import java.nio.*;

//implement some code for ops on matrices

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
    float[][] C = new float[A.length][B[0].length];
    for(int i = 0; i < A.length; i++){
      for(int j = 0; j < A[0].length; j++){
          C[i][j] = A[i][j] + B[i][j];
      }
    }
    return C;
  }
  
  public float[][] matrixSubtraction(float[][] A, float[][] B){
    float[][] C = new float[A.length][B[0].length];
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
