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
class ProduceMatrix
{
    public double[][] matrix(double Ti, double Tf) {
        double[][] T = {
            { 1, Ti,  Ti*Ti , Ti*Ti*Ti},
            { 1, Tf, Tf*Tf , Tf*Tf*Tf},
            { 0, 1, 2*Ti , 3*Ti*Ti},
            { 0, 1, 2*Tf , 3*Tf*Tf} 
        };
        return T;
    }
}
    
//produces and solves the trajectory matrix equation
class TrajecMatrixSolver {
  
  public TrajecMatrixSolver(double Ti, double Tf, double Pi, double Pf, double Vi, double Vf) {
       produceMatrix prodMatrix = new produceMatrix();
       
       double[] g = {Pi, Pf, Vi, Vf};
       
       double[][] T = prodMatrix.matrix(Ti,Tf);
       
       double[] a = solve(T, g);
       
       //a contains, in order, the coefficients of the cubic polynomial
}
}
