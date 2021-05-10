import java.io.*;
import java.util.*;

//matrix equation solving alg (found online)
public double[] solve(double[][] A, double[] b) {
        int n = b.length;

        for (int p = 0; p < n; p++) {
            // find pivot row and swap
            int max = p;
            for (int i = p + 1; i < n; i++) {
                if (Math.abs(A[i][p]) > Math.abs(A[max][p])) {
                    max = i;
                }
            }
            //swaps the pivot row into position, swaps out the original row. Does this with A and b.
            double[] temp = A[p]; A[p] = A[max]; A[max] = temp;
            double t = b[p]; b[p] = b[max]; b[max] = t;

            // singular or nearly singular
            if (Math.abs(A[p][p]) <= EPSILON) {
                throw new ArithmeticException("Matrix is singular or nearly singular");
            }

            // pivot within A and b
            for (int i = p + 1; i < n; i++) {
                double alpha = A[i][p] / A[p][p];
                b[i] -= alpha * b[p];
                for (int j = p; j < n; j++) {
                    A[i][j] -= alpha * A[p][j];
                }
            }
        }

        // back substitution
        double[] x = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            double sum = 0.0;
            for (int j = i + 1; j < n; j++) {
                sum += A[i][j] * x[j];
            }
            x[i] = (b[i] - sum) / A[i][i];
        }
        return x;
    }

//creates trajectory matrix
class produceMatrix
{
    public double[][] matrix(double Ti, double Tf, int n) {
      if(n == 3) {
        return { { 1, Ti,  Ti*Ti , Ti*Ti*Ti},
            { 1, Tf, Tf*Tf , Tf*Tf*Tf},
            { 0, 1, 2*Ti , 3*Ti*Ti},
            { 0, 1, 2*Tf , 3*Tf*Tf} };
      }
      else if(n == 5) {
        return {
          {1,Ti,Ti*Ti,Ti*Ti*Ti,Ti*Ti*Ti*Ti,Ti*Ti*Ti*Ti*Ti},
          {0,1,2*Ti,3*Ti*Ti,4*Ti*Ti*Ti,5*Ti*Ti*Ti*Ti},
          {0,0,2,6*Ti,12*Ti*Ti,20*Ti*Ti*Ti}, 
          {1,Tf,Tf*Tf,Tf*Tf*Tf,Tf*Tf*Tf*Tf,Tf*Tf*Tf*Tf*Tf},
          {0,1,2*Tf,3*Tf*Tf,4*Tf*Tf*Tf,5*Tf*Tf*Tf*Tf},
          {0,0,2,6*Tf,12*Tf*Tf,20*Tf*Tf*Tf} };
      }
        
    }
}
    
//produces and solves the trajectory matrix equation
class TrajecMatrixSolver {
  int n;
  produceMatrix prodMatrix = new produceMatrix();
  
  public TrajecMatrixSolver(double Ti, double Tf, double Pi, double Pf, double Vi, double Vf) {
       n = 3;
       
       double[] g = {Pi, Pf, Vi, Vf};
       
       double[][] T = prodMatrix.matrix(Ti,Tf,n);
       
       //would then implement the matrix solver function here
       double[] a = solve(T, g);
       
       //a contains, in order, the coefficients of the cubic polynomial
  }
  public TrajecMatrixSolver(double Ti, double Tf, double Pi, double Pf, double Vi, double Vf, double Ai, double Af) {
       n = 5;
       
       double[] g = {Pi, Pf, Vi, Vf, Ai, Af};
       
       double[][] T = prodMatrix.matrix(Ti,Tf,n);
       
       //would then implement the matrix solver function here
       double[] a = solve(T, g);
       
       //a contains, in order, the coefficients of the quintic polynomial
}
}
