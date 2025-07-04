using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ILOG.Concert;
using ILOG.CPLEX;


namespace CPLEX_TDTSPTW
{
    /* Class UserMILP is only constructed for the purpose of MILP,
     * that contains the user itself, but aditionaly the
     * indices of coresponding decision variables in array X
     */
    public class UserMILP
    {
        public UserMILP(User u)
        {
            this.u = u;
        }
        public User u;
        //Index of decision variable for rest load capacity
        public int restLoadVarInd;
        //Index of decision variable for rest energy capacity
        public int restEnergyVarInd;
        //Index of decision variable for begin time of service
        public int serviceStartTimeVarInd;
    }

    /* Basic model that contains equations for solving Ax=b,
     * with also lower and upper bounds on decision variables
     */
    public class BasicModel
    {
        //Lower and upper bounds arrays
        public double[] lb;
        public double[] ub;
        //Type of each decision variable (used are float and int)
        public NumVarType[] xtb;
        //Number of decision variables in problem (the size of arrays)
        public int numVars;

        //Lists for array constraints Ax=b
        public List<double[]> A;
        public List<double> b;
        //This list is used only to know if the equation is equal, lower, greater, lower or equal etc.
        public List<string> eqType;
        public BasicModel(int numVars)
        {
            //Upper and lower bounds
            lb = new double[numVars];
            ub = new double[numVars];
            xtb = new NumVarType[numVars];
            //Constraints
            A = new List<double[]>();
            b = new List<double>();
            eqType = new List<string>();
            //Number of variables
            this.numVars = numVars;
        }
    }


    public class MILPData
    {
        public string newOrOld;
        //Depots
        public UserMILP depot0;
        //Due to track of service begin times and departure times, we have to have multiple ending depots (in time-independet case this is not needed)
        public List<UserMILP> endingDepots;
        //Number of time buckets
        public int kBuckets;
        public int numVars;

        public Params p;
        public int numEndingDepots;
        public MILPData(Params p, int numEndingDepots,string newOrOld)
        {
            this.p = p;
            this.numEndingDepots = numEndingDepots;
            this.newOrOld= newOrOld;
        }
    }
}
