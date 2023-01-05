using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ILOG.Concert;
using ILOG.CPLEX;


namespace CPLEX_TDTSPTW
{
    /*Class only constructed for the purpose of MILP,
     * that contains the user itself, but aditionaly the
     * indices of coresponding decision variables in array X
     */
    class UserMILP
    {
        public UserMILP(User u)
        {
            this.u = u;
        }
        public User u;
        public int restLoadVarInd;
        public int restEnergyVarInd;
        public int serviceStartTimeVarInd;
    }

    /* Basic model contains equations for solving Ax=b,
     * with also lower and upper bounds on decision variables
     */
    public class BasicModel
    {
        public double[] lb;
        public double[] ub;
        public NumVarType[] xtb;
        public int numVars;

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
            //Equations
            A = new List<double[]>();
            b = new List<double>();
            eqType = new List<string>();
            this.numVars = numVars;
        }
    }


    class MILPData
    {
        //Dictionaries for arc variables
        public Dictionary<(UserMILP, UserMILP, int), int> Xijk;
        public Dictionary<(UserMILP, UserMILP, int), int> Tijk;
        public Dictionary<(UserMILP, UserMILP), int> Bij;

        //Depots
        public UserMILP depot0;
        public List<UserMILP> endingDepots;
        public int kBuckets;

        private List<UserMILP> F_;
        private List<UserMILP> V;
        public int numVars;

        /*Here go functions to get various list combinations that are used within MILP - see Schneider
         * 
         */
        public List<UserMILP> getF_()
        {
            return F_;
        }
        public List<UserMILP> getV()
        {
            return V;
        }

        public List<UserMILP> getMED()
        {
            return endingDepots;
        }

        public List<UserMILP> getF0_()
        {
            List<UserMILP> list = new List<UserMILP>(F_);
            list.Add(depot0);
            return list;
        }

        public List<UserMILP> getV_()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.AddRange(F_);
            return list;
        }

        public List<UserMILP> getV0()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.Add(depot0);
            return list;
        }

        public List<UserMILP> getV0_()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.AddRange(F_);
            list.Add(depot0);
            return list;
        }

        public List<UserMILP> getVNm_()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.AddRange(F_);
            list.AddRange(endingDepots);
            return list;
        }

        public List<UserMILP> getV0Nm_()
        {
            List<UserMILP> list = new List<UserMILP>(V);
            list.AddRange(F_);
            list.Add(depot0);
            list.AddRange(endingDepots);
            return list;
        }

        /*Function that returns all appropriate X indices for all arcs that exit user ui
         */
        public List<int> getAllVarIndicesExitArcsXIJK(UserMILP ui)
        {
            return this.Xijk.Where(x => x.Key.Item1==ui).Select(x => x.Value).ToList();
        }

        /*Function that returns all appropriate X indices for all arcs that exit user ui and enter user uj
        */
        public List<int> getAllVarIndicesExitArcsXIJK(UserMILP ui, UserMILP uj)
        {
            return this.Xijk.Where(x => x.Key.Item1 == ui && x.Key.Item2 == uj).Select(x => x.Value).ToList();
        }

        /*Function that returns all appropriate X indices for all arcs that enter user ui
        */
        public List<int> getAllVarIndicesEntryArcsXIJK(UserMILP ui)
        {
            return this.Xijk.Where(x => x.Key.Item2 == ui).Select(x => x.Value).ToList();
        }

        /*Function that returns all appropriate X indices for all arcs that exit user ui in variable Tijk
        */
        public List<int> getAllVarIndicesExitArcsTIJK(UserMILP ui)
        {
            return this.Tijk.Where(x => x.Key.Item1 == ui).Select(x => x.Value).ToList();
        }


        /*
         * Class for storing decision variables and vertices used in MILP program
         * */
        public MILPData(Params p,int numEndingDepots)
        {
            //Number of time intervals
            kBuckets = p.timeBuckets.Length;
            //Dictionary for storinf xijk variable index within MILP
            Xijk = new Dictionary<(UserMILP, UserMILP, int), int>();
            //Dictionary for storinf tijk variable index within MILP
            Tijk = new Dictionary<(UserMILP, UserMILP, int), int>();
            //Dictionary for storinf Bij variable index within MILP - this variable is only used to deterime max between service begin time and early time window
            //It is necessary only in TD cases when we want to know exact departure times
            Bij = new Dictionary<(UserMILP, UserMILP), int>();

            //List of CSs with virtual CS replication
            F_ = new List<UserMILP>();
            //List of customers
            V = new List<UserMILP>();
            //In case with variable bij, tmultiple ending depots have to be used to track the begin times at them
            endingDepots = new List<UserMILP>();
            //Load nodes from problem instance
            foreach (User u in p.users)
            {
                if (u.isDepot)
                {
                    //We have two instances of depot: 0-strating depot, N (or N+1) i ending depot
                    depot0 = new UserMILP(p.depot);
                    for (int i = 0; i < numEndingDepots; i++)
                    {
                        endingDepots.Add(new UserMILP(p.depot));
                    }
                }
                else if (u.isStation())
                {
                    //Stattions are coppied a numMultiCSVert times to allow multiple visit to same CS - see Schneider 2014
                    for (int i = 0; i < p.numMultiCSVert; i++)
                    {
                        UserMILP cs = new UserMILP(u);
                        F_.Add(cs);
                    }
                }
                else
                {
                    //Customers
                    UserMILP cust = new UserMILP(u);
                    V.Add(cust);
                }
            }

            /*Dictionary for decision variables
             * Xijk - binary variable indicating whether the arc i,j,k is used or not
             * Tijk - float variable indicating the departure time forim user i to user j in time priod k - to be clear only in one period k this value can be >0
             * Bij - binary variable for choosing max between begin of service and early time window (has to be ij)
             * */
            int varIndex = 0;
            foreach (UserMILP uI in getV0_())
            {
                foreach (UserMILP uJ in getVNm_())
                {
                    for (int k = 0; k < kBuckets; k++)
                    {
                        if (!p.infeas(uI.u, uJ.u))
                        {
                            Xijk.Add((uI, uJ, k), varIndex);
                            varIndex++;
                            Tijk.Add((uI, uJ, k), varIndex);
                            varIndex++;
                        }
                    }
                    if (!p.infeas(uI.u, uJ.u))
                    {
                        Bij.Add((uI, uJ), varIndex);
                        varIndex++;
                    }
                    }
            }

            /*For all vertices set decision variables
             * restLoadVarInd - rest load capacity at arrival at user i
             * restEnergyVarInd - rest energy level at ARRIVAL(!) at user I
             * serviceStartTimeVarInd - the start time of service at user i
             */
            foreach (UserMILP u in getV0Nm_())
            {
                u.restLoadVarInd = varIndex;
                varIndex++;
                u.restEnergyVarInd = varIndex;
                varIndex++;
                //Not sure if this decision variable can actually be removed??? (as we have departure time -tricky)
                u.serviceStartTimeVarInd = varIndex;
                varIndex++;
            }
            numVars = varIndex;

            /*Additional explanation
             * Basically we will have an array X that contains all of the decision variables.
             * The length of this array will be numVars, and when we will construct an equalites for the problem
             * we for each vertex in graph that can indicate (rest load, rest energy, or service start time) have to know
             * the exact index in the array X, that coresponds to the appropriate decsion variable.
             * The same goes for arcs. 
             */
        }
    }

}
