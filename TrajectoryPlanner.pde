import java.io.*;
import java.util.*;


//will store a bunch of TrajecMatrixSolver in order to characterise a full trajectory with quintic/cubic splines
class TrajectoryPlanner{
    ArrayList< ArrayList<TrajecMatrixSolver> >quinticTraj;
    public TrajectoryPlanner(){
      this.quinticTraj = new ArrayList< ArrayList<TrajecMatrixSolver> >();
    }
    public void constructParametrizedTraj(ArrayList< ArrayList<Float> > JOINT_INTERPOLATION){
       ArrayList< ArrayList<TrajecMatrixSolver> > traj = new ArrayList< ArrayList<TrajecMatrixSolver> >();
       ArrayList<Float> current_pose = (ArrayList)IK2.angles.clone();
       ArrayList<Float> current_velocity = new ArrayList<Float>();
       ArrayList<Float> current_acceleration = new ArrayList<Float>();
       for(int i = 0; i < CHAIN_SIZE; i++){current_velocity.add(0.0); current_acceleration.add(0.0);}
       for(int i = 0; i < JOINT_INTERPOLATION.size(); i++){
         ArrayList<Float> tmp = JOINT_INTERPOLATION.get(i);
         ArrayList<TrajecMatrixSolver> cur_joint = new ArrayList<TrajecMatrixSolver>();
         for(int j = 0; j < tmp.size(); j++){
           float rv = random(0,1);
           float ra = random(0,1);
           TrajecMatrixSolver tms = new TrajecMatrixSolver(0, 3, current_pose.get(j), tmp.get(j), current_velocity.get(j), rv, 0, 0);
           double[][] d = tms.quinticMatrix(0,3);
           /*
           println("MATRIX: ");
           for(int k = 0; k < d.length; k++){
             for(int l = 0; l < d[0].length; l++){
               print(d[k][l] + " " );
             }
             println();
           }
           println("B VECTOR: " );
           print(0 + " " + 3 + " " + current_pose.get(j) + " " + tmp.get(j) + " " + current_velocity.get(j) + " " + rv + " " + 0 + " " + 0);
           println();
           println("SOLUTION: " );
           for(int l = 0; l < tms.solution_vector.length; l++){
             print(tms.solution_vector[l] + " ");
           }
           println();
           println();
           */
           cur_joint.add(tms);
           current_velocity.set(j,rv);
           current_acceleration.set(j,ra);
         }
         for(int j = 0; j < tmp.size(); j++){
           current_pose.set(j,tmp.get(j));
         }
         traj.add(cur_joint);
       }
       this.quinticTraj = traj;
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
        {1, Ti, Ti*Ti, Ti*Ti*Ti, Ti*Ti*Ti*Ti,  Ti*Ti*Ti*Ti*Ti},
        {1, Tf, Tf*Tf, Tf*Tf*Tf, Tf*Tf*Tf*Tf, Tf*Tf*Tf*Tf*Tf},
        {0, 1, 2*Ti, 3*Ti*Ti, 4*Ti*Ti*Ti, 5*Ti*Ti*Ti*Ti},
        {0, 1, 2*Tf, 3*Tf*Tf, 4*Tf*Tf*Tf, 5*Tf*Tf*Tf*Tf},
        {0, 0, 2, 6*Ti, 12*Ti*Ti, 20*Ti*Ti*Ti},
        {0, 0, 2, 6*Tf, 12*Tf*Tf, 20*Tf*Tf*Tf}
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
