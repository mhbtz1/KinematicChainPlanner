
public class TrajectoryMatrixSolver
{
    public static double[][] produceMatrix(double Ti, double Tf) {
        double[][] T = {
            { 1, Ti,  Ti*Ti , Ti*Ti*Ti},
            { 1, Tf, Tf*Tf , Tf*Tf*Tf},
            { 0, 1, 2*Ti , 3*Ti*Ti},
            { 0, 1, 2*Tf , 3*Tf*Tf} 
        };
        return T;
    }
    

   public static void main(String[] args) {
       double Ti = 0, Tf = 10, Pi = 0, Pf = 3, Vi = 0, Vf = 0;
       
       double[] g = {Pi, Pf, Vi, Vf};
       
       double[][] T = produceMatrix(Ti,Tf);
       
       //would then implement the matrix solver function here
       //double[] a = solveDatJawn(T, g);
       
       //a contains, in order, the coefficients of the cubic polynomial
       
    }
}
