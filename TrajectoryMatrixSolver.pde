import java.io.*;
import java.util.*;


//will store a bunch of TrajecMatrixSolver in order to characterise a full trajectory with quintic/cubic splines
class TrajectoryPlanner{
    ArrayList<TrajecMatrixSolver> quinticTraj;
    public TrajectoryPlanner(){
      this.quinticTraj = new ArrayList<TrajecMatrixSolver>();
    }
    public void constructParametrizedTraj(){
      return;
    }
}

    
//produces and solves the trajectory matrix equation
class TrajecMatrixSolver {
   double[] solution_vector;
   public double[][] cubicMatrix(double Ti, double Tf) {
        double[][] T = {
            { 1, Ti,  Ti*Ti , Ti*Ti*Ti},
            { 1, Tf, Tf*Tf , Tf*Tf*Tf},
            { 0, 1, 2*Ti , 3*Ti*Ti},
            { 0, 1, 2*Tf , 3*Tf*Tf} 
        };
        return T;
    }
    public double[][] quinticMatrix(double Ti, double Tf){
      double[][] T = {
        {1, Ti, Ti*Ti, Ti*Ti*Ti, Ti*Ti*Ti*Ti,  Ti*Ti*Ti*Ti*Ti },
        {1, Tf, Tf*Tf, Tf*Tf*Tf, Tf*Tf*Tf*Tf, Tf*Tf*Tf*Tf*Tf},
        {0, 1, 2*Ti, 3*Ti*Ti, 4*Ti*Ti*Ti, 5*Ti*Ti*Ti*Ti},
        {0, 1, 2*Tf, 3*Tf*Tf, 4*Tf*Tf*Tf, 5*Tf*Tf*Tf*Tf},
        {0, 0, 2, 6*Ti*Ti, 12*Ti*Ti, 20*Ti*Ti*Ti},
        {0, 0, 2, 6*Tf*Tf, 12*Tf*Tf, 20*Tf*Tf*Tf}
      };
      return T;
    }
    
    public TrajecMatrixSolver(double Ti, double Tf, double Pi, double Pf, double Vi, double Vf) {
       double[] g = {Pi, Pf, Vi, Vf};
       double[][] T = cubicMatrix(Ti,Tf);  
       solution_vector = solve(T, g);   
       //a contains, in order, the coefficients of the cubic polynomial
    }
    public TrajecMatrixSolver(double Ti, double Tf, double Pi, double Pf, double Vi, double Vf, double Ai, double Af){
      double[] g = {Pi, Pf, Vi, Vf, Ai, Af};
      double[][] T = quinticMatrix(Ti, Tf);
      solution_vector = solve(T,g);
    }
    //solves matrix equation Ax = b (this assumes the matrix is nonsingular, i.e. invertible)
    public double[] solve(double[][] A, double[] b) throws ArithmeticException{
        int n = b.length;
        for (int p = 0; p < n; p++){
            int max = p;
            for (int i = p + 1; i < n; i++) {
                if (Math.abs(A[i][p]) > Math.abs(A[max][p])) {
                    max = i;
                }
            }
            double[] temp = A[p]; A[p] = A[max]; A[max] = temp;
            double t = b[p]; b[p] = b[max]; b[max] = t;
            if (Math.abs(A[p][p]) <= EPSILON) {
                throw new ArithmeticException("Matrix is singular or nearly singular");
            }
            for (int i = p + 1; i < n; i++) {
                double alpha = A[i][p] / A[p][p];
                b[i] -= alpha * b[p];
                for (int j = p; j < n; j++) {
                    A[i][j] -= alpha * A[p][j];
                }
            }
        }
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
}
